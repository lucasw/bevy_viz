use avian3d::prelude::*;
use bevy::color::palettes::css;
use bevy::prelude::*;
use bevy::render::{
    render_asset::RenderAssetUsages,
    render_resource::{Extent3d, TextureDimension, TextureFormat},
};
use bevy::window;

#[derive(Component)]
pub struct Car {
    // half dimensions of chassis
    xs: f32,
    ys: f32,
    zs: f32,
    lidar_position: Vec3,
    throttle: f32,
    brake: f32,
    steering_angle: f32,
    compression: [f32; 4],
    rec: rerun::RecordingStream,
}

fn bevy_vec3_to_rerun_vec3d(v: &Vec3) -> rerun::Vec3D {
    rerun::Vec3D::new(v.x, v.y, v.z)
}

impl Car {
    fn new(xs: f32, ys: f32, zs: f32, rec: rerun::RecordingStream) -> Self {
        let arrows =
            rerun::Arrows3D::from_vectors([[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]])
                .with_colors([[255, 0, 0], [0, 255, 0], [0, 0, 255]]);
        rec.log_static("world/car/xyz", &arrows).unwrap();

        rec.log(
            "world/car/camera_mount",
            &rerun::Transform3D::from_translation(rerun::Vec3D::new(0.0, ys * 1.6, -zs * 0.7)),
        )
        .unwrap();
        rec.log(
            "world/car/camera_mount/camera",
            &rerun::Pinhole::from_focal_length_and_resolution([3., 4.], [3., 3.])
                // TODO(lucasw) expected this to inherit from the world root but need to specify
                .with_camera_xyz(rerun::components::ViewCoordinates::RUB)
                .with_image_plane_distance(1.0),
        )
        .unwrap();

        rec.log(
            "world/car/camera_mount_selfie",
            &rerun::Transform3D::from_translation(rerun::Vec3D::new(0.0, ys * 3.0, zs * 3.0)),
        )
        .unwrap();
        rec.log(
            "world/car/camera_mount_selfie/camera",
            &rerun::Pinhole::from_focal_length_and_resolution([3., 4.], [3., 3.])
                // TODO(lucasw) expected this to inherit from the world root but need to specify
                .with_camera_xyz(rerun::components::ViewCoordinates::RUB)
                .with_image_plane_distance(1.0),
        )
        .unwrap();

        let lidar_position = Vec3::new(0.0, ys * 3.5, -zs * 1.1);
        rec.log(
            "world/car/lidar_position",
            &rerun::Transform3D::from_translation(bevy_vec3_to_rerun_vec3d(&lidar_position)),
        )
        .unwrap();
        rec.log_static("world/car/lidar_position/xyz", &arrows)
            .unwrap();

        rec.log(
            "world/car/chassis",
            &rerun::Boxes3D::from_half_sizes([(xs, ys, zs)]),
        )
        .unwrap();

        Car {
            xs,
            ys,
            zs,
            lidar_position,
            throttle: 0.0,
            brake: 0.0,
            steering_angle: 0.0,
            compression: [0.0, 0.0, 0.0, 0.0],
            rec,
        }
    }

    fn update(&mut self, car_tf: &GlobalTransform, external_force: &mut ExternalForce) {
        let arrow =
            rerun::Arrows3D::from_vectors([(1.0, 0.0, 0.0)]).with_origins([(0.0, 0.0, 0.0)]);
        self.rec.log("world", &arrow).unwrap();

        let forward = car_tf.forward();
        let right = car_tf.right();
        let translation = car_tf.translation();

        let rotation = car_tf.rotation();

        // TODO(lucasw) this rotation needs to be rotated
        let car_xyzw_raw = [rotation.x, rotation.y, rotation.z, rotation.w];
        let car_rotation_rr = rerun::Quaternion::from_xyzw(car_xyzw_raw);

        // let car_translation_raw = [translation.x, translation.y, translation.z];
        let car_translation_rr =
            rerun::components::Translation3D::new(translation.x, translation.y, translation.z);
        let car_transform_rr =
            rerun::Transform3D::from_translation_rotation(car_translation_rr, car_rotation_rr);
        // let car_transform_rr = rerun::Transform3D::from_translation(car_translation_rr);
        self.rec.log("world/car", &car_transform_rr).unwrap();
        self.rec.log("world/car/arrow", &arrow).unwrap();

        self.throttle = self.throttle.clamp(0.0, 1.0);
        self.brake = self.brake.clamp(0.0, 1.0);
        self.steering_angle = self.steering_angle.clamp(-1.0, 1.0);

        // TODO(lucasw) only apply this to wheels that are touching the ground?
        // TODO(lucasw) apply less force in proportion to velocity, most when at a standstill
        let force = forward * self.throttle * 25.0;
        // println!("car control force {force:?}");
        // TODO(lucasw) not ackermann at all, just get some relationship
        // between steer 'angle' and actually turning
        let point =
            translation + (car_tf.back() * self.zs * 0.9) + (right * self.steering_angle * 0.1);
        external_force.apply_force_at_point(force, point, translation);

        self.steering_angle *= 0.98;
        self.brake *= 0.94;
        self.throttle *= 0.97;

        self.rec
            .log(
                "world/car/steering_angle",
                &rerun::Scalar::new(self.steering_angle as f64),
            )
            .unwrap();
        self.rec
            .log(
                "world/car/throttle",
                &rerun::Scalar::new(self.throttle as f64),
            )
            .unwrap();
    }

    // do many ray casts all around the car
    fn lidar_update(
        &mut self,
        spatial_query: &SpatialQuery,
        car_tf: &GlobalTransform,
        // , car_filter: &SpatialQueryFilter) {
    ) {
        let max_range = 100.0;

        // TODO(lucasw) transform lidar_position with car_tf in one step?
        let lidar_rel_pos = car_tf.rotation() * self.lidar_position;
        let sensor_pos_in_world = car_tf.translation() + lidar_rel_pos;

        let mut points = Vec::new();

        for angle_degrees in (0..360).step_by(5) {
            let angle = (angle_degrees as f32).to_radians();
            for elevation_angle_degrees in (-40..20i32).step_by(5) {
                let elevation = (elevation_angle_degrees as f32).to_radians();

                let ray_vec_in_body = Vec3::new(
                    angle.cos() * elevation.cos(),
                    elevation.sin(),
                    angle.sin() * elevation.cos(),
                );

                let ray_vec_in_world = car_tf.rotation() * ray_vec_in_body;
                // TODO(lucasw) pre-compute all these rays
                let Ok(ray_dir_in_world) = Dir3::new(ray_vec_in_world) else {
                    continue;
                };

                // TODO(lucasw) rotate ray_dir with car_tf orientation
                // TODO(lucasw) this collides with collision objects, but would rather
                // collide with any mesh because that will have more interesting detail
                // so find a bevy ray casting method not an avian physics one.
                let hit = spatial_query.cast_ray(
                    // TODO(lucasw) get location from car outside the chassis collision volume
                    sensor_pos_in_world,
                    ray_dir_in_world,
                    max_range,
                    false,
                    &SpatialQueryFilter::default(),
                );
                // println!("translation: {:?}, down: {:?} -> {hit:?}", car_tf.translation(), car_tf.down());

                let Some(hit) = hit else {
                    continue;
                };
                // the intersection point in world coordinates
                let point = sensor_pos_in_world + ray_dir_in_world * hit.distance;
                // println!("{angle_degrees} {point:?}");
                points.push(rerun::Position3D::new(point.x, point.y, point.z));
            }
        }
        self.rec
            // TODO(lucasw) put it into the car frame
            // .log("world/car/point_cloud", &rerun::Points3D::new(points))
            .log("world/car_point_cloud", &rerun::Points3D::new(points))
            .unwrap();
    }
}

#[derive(Component)]
pub struct CarCamera {
    offset: Vec3,
}

fn main() {
    App::new()
        .insert_resource(ClearColor(Color::srgb(
            0xF9 as f32 / 255.0,
            0xF9 as f32 / 255.0,
            0xFF as f32 / 255.0,
        )))
        .add_plugins((
            DefaultPlugins.set(WindowPlugin {
                primary_window: Some(Window {
                    resolution: window::WindowResolution::new(1280., 720.)
                        .with_scale_factor_override(1.0),
                    ..default()
                }),
                ..default()
            }),
            PhysicsPlugins::default(),
        ))
        .add_systems(Startup, (setup_graphics, setup_physics))
        .add_systems(
            Update,
            (
                car_suspension,
                move_camera,
                control_car,
                update_car,
                camera_car_update,
            ),
        )
        .run();
}

pub fn setup_graphics(mut commands: Commands) {
    commands.spawn((
        Camera3d::default(),
        // Transform::from_xyz(-5.0, 7.0, 12.0).looking_at(Vec3::new(0.0, 2.5, 0.0), Vec3::Y),
        CarCamera {
            offset: Vec3::new(0.0, 5.0, 30.0),
        },
    ));
}

pub fn setup_physics(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut images: ResMut<Assets<Image>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    // TODO(lucasw) how to return Result from Startup functions?
    let rec = rerun::RecordingStreamBuilder::new("rerun_vehicle")
        .connect_tcp()
        .unwrap();

    // match bevy defaults X=Right, Y=Up, Z=Back
    rec.log_static("world", &rerun::ViewCoordinates::RUB)
        .unwrap(); // Set an up-axis
    rec.log_static(
        "world/xyz",
        &rerun::Arrows3D::from_vectors(
            [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]], //
        )
        .with_colors([[255, 0, 0], [0, 255, 0], [0, 0, 255]]),
    )
    .unwrap();

    // Some light to see something
    commands.spawn((
        PointLight {
            intensity: 45_000_000.,
            range: 500.,
            shadows_enabled: true,
            ..default()
        },
        Transform::from_xyz(100., 82., 40.),
    ));

    // ground
    {
        let ground_size = 1200.0;
        let ground_height = 0.1;

        let ground = commands.spawn((
            RigidBody::Static,
            Collider::cuboid(ground_size, ground_height, ground_size),
            Mesh3d(meshes.add(Plane3d::default().mesh().size(ground_size, ground_size))),
            MeshMaterial3d(materials.add(StandardMaterial {
                // TODO(lucasw) add uv coordinates to make this repeat
                base_color_texture: Some(images.add(uv_debug_texture())),
                ..default()
            })),
            Transform::from_xyz(0.0, -ground_height, 0.0),
        ));
        println!("spawned ground: {}", ground.id());

        // some bumps on the ground
        {
            for i in 0..10 {
                let i = i as f32;
                let radius = 3.0 + i / 2.0;
                let length = 300.0;
                commands.spawn((
                    RigidBody::Static,
                    Collider::capsule(radius, length),
                    Mesh3d(meshes.add(Capsule3d::new(radius, length))),
                    MeshMaterial3d(materials.add(Color::from(css::BEIGE))),
                    Transform::from_xyz(0.0, -radius * 0.75, -(i + 3.5) * 24.0).with_rotation(
                        Quat::from_rotation_z(std::f32::consts::PI / 2.0 + 0.002 * i),
                    ),
                ));
            }
        }
    }

    // car chassis
    {
        // y is vertical
        let x = 0.0;
        let y = 5.0;
        let z = 0.0;

        // half sizes
        let xs = 2.0; // left/right
        let ys = 0.75; // up/down
        let zs = 4.0; // forward/back
        let radius = ys * 0.9;

        let car = Car::new(xs, ys, zs, rec);
        let car_command = commands.spawn((
            RigidBody::Dynamic,
            Collider::round_cuboid(
                (xs - radius) * 2.0,
                (ys - radius) * 2.0,
                (zs - radius) * 2.0,
                radius,
            ),
            // ColliderDebugColor(Hsla::hsl(220.0, 1.0, 0.3)),
            Mesh3d(meshes.add(Cuboid::new(xs * 2.0, ys * 2.0, zs * 2.0))),
            MeshMaterial3d(materials.add(Color::from(css::BLUE))),
            Transform::from_xyz(x, y, z).with_rotation(Quat::from_rotation_x(0.2)),
            car,
            ExternalForce::new(Vec3::new(0.0, 0.0, 0.0)).with_persistence(false),
            ExternalImpulse::new(Vec3::new(0.0, 0.0, 0.0)).with_persistence(false),
        ));
        println!("spawned car id: {}", car_command.id());
    }
}

// camera control
pub fn move_camera(key_input: Res<ButtonInput<KeyCode>>, mut query: Query<&mut CarCamera>) {
    if let Ok(mut camera) = query.get_single_mut() {
        if key_input.pressed(KeyCode::KeyW) {
            camera.offset -= Vec3::Z * 0.3;
        }
        if key_input.pressed(KeyCode::KeyS) {
            camera.offset += Vec3::Z * 0.27;
        }

        if key_input.pressed(KeyCode::KeyA) {
            camera.offset -= Vec3::X * 0.3;
        }
        if key_input.pressed(KeyCode::KeyD) {
            camera.offset += Vec3::X * 0.3;
        }

        if key_input.pressed(KeyCode::KeyQ) {
            camera.offset += Vec3::Y * 0.3;
        }
        if key_input.pressed(KeyCode::KeyZ) {
            camera.offset -= Vec3::Y * 0.25;
        }
    }
}

pub fn camera_car_update(
    car: Query<&GlobalTransform, With<Car>>,
    mut camera: Query<(&CarCamera, &mut Transform)>,
) {
    let Ok(car) = car.get_single() else {
        return;
    };
    let Ok((car_camera, mut camera_transform)) = camera.get_single_mut() else {
        return;
    };

    let target = car_camera.offset + car.translation();
    let delta = target - camera_transform.translation;
    camera_transform.translation += delta * 0.02;
    // println!("camera {:?}", camera_transform);
}

pub fn control_car(key_input: Res<ButtonInput<KeyCode>>, mut query: Query<&mut Car>) {
    if let Ok(mut car) = query.get_single_mut() {
        if key_input.pressed(KeyCode::ArrowUp) {
            car.throttle += 0.1;
        }
        if key_input.pressed(KeyCode::ArrowDown) {
            car.brake += 0.1;
        }
        if key_input.pressed(KeyCode::ArrowLeft) {
            car.steering_angle += 0.05;
        }
        if key_input.pressed(KeyCode::ArrowRight) {
            car.steering_angle -= 0.05;
        }
    }
}

pub fn update_car(mut query: Query<(&mut Car, &mut ExternalForce, &GlobalTransform)>) {
    if let Ok((mut car, mut external_force, car_tf)) = query.get_single_mut() {
        car.update(car_tf, &mut external_force);
    }
}

pub fn car_suspension(
    // mut commands: Commands,
    // TODO(lucasw) try SpatialQueryPipeline
    spatial_query: SpatialQuery,
    mut query: Query<(&mut Car, Entity, &mut ExternalImpulse, &GlobalTransform)>,
) {
    let max_length = 1.4;
    for (mut car, car_entity, mut external_impulse, car_tf) in query.iter_mut() {
        // TODO(lucasw) car.update_suspension()
        let car_filter = SpatialQueryFilter::default().with_excluded_entities([car_entity]);
        car.lidar_update(&spatial_query, car_tf); // , &car_filter);
        let wheel_radius = 0.5;
        let wheel_y = car.ys - wheel_radius;

        // println!("{:?} {external_impulse:?}", car_tf.translation());
        // four wheels
        // TODO(lucasw) need to use Car size as set above in xs, ys, & zs
        let wheel_positions = vec![
            car_tf.translation()
                + car_tf.forward() * car.zs * 0.8
                + car_tf.down() * wheel_y
                + car_tf.left() * car.xs,
            car_tf.translation()
                + car_tf.forward() * car.zs * 0.8
                + car_tf.down() * wheel_y
                + car_tf.right() * car.xs,
            car_tf.translation()
                + car_tf.back() * car.zs * 0.8
                + car_tf.down() * wheel_y
                + car_tf.left() * car.xs,
            car_tf.translation()
                + car_tf.back() * car.zs * 0.8
                + car_tf.down() * wheel_y
                + car_tf.right() * car.xs,
        ];
        // println!("{:?}", car);
        for (ind, wheel_pos) in wheel_positions.into_iter().enumerate() {
            let down = *car_tf.down();
            let up = *car_tf.up();
            let dir = Dir3::new(down).unwrap();
            let hit = spatial_query.cast_ray(
                // TODO(lucasw) get location from car outside the chassis collision volume
                wheel_pos,
                dir,
                max_length, // f32::MAX,
                false,
                &car_filter,
            );
            // println!("translation: {:?}, down: {:?} -> {hit:?}", car_tf.translation(), car_tf.down());

            let filter_fr = 0.0;
            if let Some(hit) = hit {
                // println!("{intersection:?}, {car} {external_impulse:?}");
                // if hit.distance > 0.0
                let compression = max_length - hit.distance;
                // let compression_delta = compression - car.compression[ind];
                let compression_filtered =
                    compression * (1.0 - filter_fr) + car.compression[ind] * filter_fr;
                car.compression[ind] = compression_filtered;
                // if ind == 0 {
                //     println!("wheel: {ind}, {compression:.3} {compression_delta:.3} impulse {compression_filtered:.3}");
                // }
                // the spring only acts along the car up direction, ground normal doesn't matter
                // here (it does matter for skidding though)
                // let impulse = hit.normal * impulse_magnitude;
                let impulse = up * compression_filtered * 0.05;
                // external_impulse.apply_impulse(impulse);
                external_impulse.apply_impulse_at_point(impulse, wheel_pos, car_tf.translation());

                let contact_point = wheel_pos + dir * hit.distance;
                // println!("contact_point {contact_point} {impulse}");
                car.rec
                    .log(
                        format!("world/car_wheel_contact{ind}"),
                        &rerun::LineStrips3D::new([[
                            bevy_vec3_to_rerun_vec3d(&contact_point),
                            bevy_vec3_to_rerun_vec3d(&(contact_point + up * impulse * 30.0)),
                        ]]),
                    )
                    .unwrap();
                // Color in blue the entity we just hit.
                // Because of the query filter, only colliders attached to a dynamic body
                // will get an event.
                // let color = bevy::color::palettes::basic::BLUE.into();
                // commands.entity(hit_entity).insert(ColliderDebugColor(color));

                // TODO(lucasw) apply forces/impulses to wheel if it is moving sideways with
                // respect to the object it hit (for now assume hit object is static)
            } else {
                car.compression[ind] *= filter_fr;
            }

            {
                let pos = wheel_pos + (max_length - car.compression[ind] - wheel_radius) * dir;
                car.rec
                    .log(
                        format!("world/wheel{ind}"),
                        &rerun::Capsules3D::from_lengths_and_radii([0.2], [wheel_radius])
                            .with_translations([bevy_vec3_to_rerun_vec3d(&pos)]),
                    )
                    .unwrap();
            }
        }
        // println!("compression: {:?}, external impulse: {external_impulse:?}", car.compression);
    }
}

/// Creates a colorful test pattern
fn uv_debug_texture() -> Image {
    const TEXTURE_SIZE: usize = 8;

    let mut palette: [u8; 32] = [
        255, 102, 159, 255, 255, 159, 102, 255, 236, 255, 102, 255, 121, 255, 102, 255, 102, 255,
        198, 255, 102, 198, 255, 255, 121, 102, 255, 255, 236, 102, 255, 255,
    ];

    let mut texture_data = [0; TEXTURE_SIZE * TEXTURE_SIZE * 4];
    for y in 0..TEXTURE_SIZE {
        let offset = TEXTURE_SIZE * y * 4;
        texture_data[offset..(offset + TEXTURE_SIZE * 4)].copy_from_slice(&palette);
        palette.rotate_right(4);
    }

    Image::new_fill(
        Extent3d {
            width: TEXTURE_SIZE as u32,
            height: TEXTURE_SIZE as u32,
            depth_or_array_layers: 1,
        },
        TextureDimension::D2,
        &texture_data,
        TextureFormat::Rgba8UnormSrgb,
        RenderAssetUsages::default(),
    )
}
