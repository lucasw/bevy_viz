use avian3d::prelude::*;
use bevy::color::palettes::css::SILVER;
use bevy::prelude::*;

#[derive(Component)]
pub struct Car {
    compression: [f32; 4],
}

fn main() {
    App::new()
        .insert_resource(ClearColor(Color::srgb(
            0xF9 as f32 / 255.0,
            0xF9 as f32 / 255.0,
            0xFF as f32 / 255.0,
        )))
        .add_plugins((DefaultPlugins, PhysicsPlugins::default()))
        .add_systems(Startup, (setup_graphics, setup_physics))
        .add_systems(Update, cast_ray)
        .run();
}

pub fn setup_graphics(mut commands: Commands) {
    commands.spawn((
        Camera3d::default(),
        Transform::from_xyz(-5.0, 7.0, 12.0).looking_at(Vec3::new(0.0, 2.5, 0.0), Vec3::Y),
    ));
}

pub fn setup_physics(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    // Some light to see something
    commands.spawn((
        PointLight {
            intensity: 4_000_000.,
            range: 100.,
            shadows_enabled: true,
            ..default()
        },
        Transform::from_xyz(8., 22., 8.),
    ));

    // ground
    {
        let ground_size = 300.1;
        let ground_height = 0.1;

        let ground = commands.spawn((
            RigidBody::Static,
            Collider::cuboid(ground_size, ground_height, ground_size),
            Mesh3d(meshes.add(Plane3d::default().mesh().size(ground_size, ground_size))),
            MeshMaterial3d(materials.add(Color::from(SILVER))),
            Transform::from_xyz(0.0, -ground_height, 0.0),
        ));
        println!("spawned ground: {}", ground.id());
    }

    // car chassis
    {
        // y is vertical
        let x = 0.0;
        let y = 5.0;
        let z = 0.0;

        // half sizes
        let xs = 4.0;
        let ys = 0.5;
        let zs = 2.0;

        let car = Car {
            compression: [0.0, 0.0, 0.0, 0.0],
        };
        let car_command = commands.spawn((
            RigidBody::Dynamic,
            Collider::cuboid(xs, ys, zs),
            // ColliderDebugColor(Hsla::hsl(220.0, 1.0, 0.3)),
            Mesh3d(meshes.add(Cuboid::new(xs * 2.0, ys * 2.0, zs * 2.0))),
            MeshMaterial3d(materials.add(Color::from(SILVER))),
            Transform::from_xyz(x, y, z).with_rotation(Quat::from_rotation_x(0.2)),
            car,
            // ExternalForce::new(Vec3::new(0.0, 0.0, 0.0)).with_persistence(false),
            ExternalImpulse::new(Vec3::new(0.0, 0.0, 0.0)).with_persistence(false),
        ));
        println!("spawned car id: {}", car_command.id());
    }
}

pub fn cast_ray(
    // mut commands: Commands,
    spatial_query: SpatialQuery,
    mut query: Query<(&mut Car, Entity, &mut ExternalImpulse, &GlobalTransform)>,
) {
    let max_length = 0.4;
    let wheel_y = 0.15;
    for (mut car, car_entity, mut external_impulse, car_tf) in query.iter_mut() {
        // println!("{:?} {external_impulse:?}", car_tf.translation());
        // four wheels
        // TODO(lucasw) need to use Car size as set above in xs, ys, & zs
        let wheel_positions = vec![
            car_tf.translation()
                + car_tf.forward() * 0.8
                + car_tf.down() * wheel_y
                + car_tf.left() * 1.0,
            car_tf.translation()
                + car_tf.forward() * 0.8
                + car_tf.down() * wheel_y
                + car_tf.right() * 1.0,
            car_tf.translation()
                + car_tf.back() * 0.8
                + car_tf.down() * wheel_y
                + car_tf.left() * 1.0,
            car_tf.translation()
                + car_tf.back() * 0.8
                + car_tf.down() * wheel_y
                + car_tf.right() * 1.0,
        ];
        // println!("{:?}", car);
        for (ind, wheel_pos) in wheel_positions.into_iter().enumerate() {
            let down = *car_tf.down();
            let hit = spatial_query.cast_ray(
                // TODO(lucasw) get location from car outside the chassis collision volume
                wheel_pos,
                Dir3::new(down).unwrap(),
                max_length, // f32::MAX,
                false,
                &SpatialQueryFilter::default().with_excluded_entities([car_entity]),
                // TODO(lucasw) how to make a queryfilter to exclude the object the ray originates
                // from, the car?  But it ought to hit other cars if anyy as well.
                // SpatialQueryFilter::new(<Without<Car>>),
            );
            // println!("translation: {:?}, down: {:?} -> {hit:?}", car_tf.translation(), car_tf.down());

            if let Some(hit) = hit {
                // println!("{intersection:?}, {car} {external_impulse:?}");
                // if hit.distance > 0.0
                let compression = max_length - hit.distance;
                let compression_delta = compression - car.compression[ind];
                car.compression[ind] = compression;
                let impulse_magnitude = compression * 3.0; // + compression_delta * 3.0;
                if ind == 0 {
                    println!("wheel: {ind}, {compression:.3} {compression_delta:.3} impulse {impulse_magnitude:.3}");
                }
                // let impulse = hit.normal * impulse_magnitude;
                let impulse = car_tf.up() * impulse_magnitude;
                // external_impulse.apply_impulse(impulse);
                // TODO(lucasw) external impulse is unchanged by this
                external_impulse.apply_impulse_at_point(impulse, wheel_pos, car_tf.translation());

                // Color in blue the entity we just hit.
                // Because of the query filter, only colliders attached to a dynamic body
                // will get an event.
                // let color = bevy::color::palettes::basic::BLUE.into();
                // commands.entity(hit_entity).insert(ColliderDebugColor(color));

                // TODO(lucasw) apply forces/impulses to wheel if it is moving sideways with
                // respect to the object it hit (for now assume hit object is static)
            } else {
                car.compression[ind] = 0.0;
            }
        }
        // println!("compression: {:?}, external impulse: {external_impulse:?}", car.compression);
    }
}
