use avian3d::prelude::*;
use bevy::color::palettes::css::SILVER;
use bevy::prelude::*;

#[derive(Component)]
pub struct Car;

fn main() {
    App::new()
        .insert_resource(ClearColor(Color::srgb(
            0xF9 as f32 / 255.0,
            0xF9 as f32 / 255.0,
            0xFF as f32 / 255.0,
        )))
        .add_plugins((DefaultPlugins, PhysicsPlugins::default()))
        .add_systems(Startup, (setup_graphics, setup_physics))
        // .add_systems(Update, cast_ray)
        .run();
}

pub fn setup_graphics(mut commands: Commands) {
    commands.spawn((
        Camera3d::default(),
        Transform::from_xyz(-20.0, 30.0, 50.0).looking_at(Vec3::new(0.0, 10.0, 0.0), Vec3::Y),
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

        commands.spawn((
            RigidBody::Static,
            Collider::cuboid(ground_size, ground_height, ground_size),
            Mesh3d(meshes.add(Plane3d::default().mesh().size(ground_size, ground_size))),
            MeshMaterial3d(materials.add(Color::from(SILVER))),
            Transform::from_xyz(0.0, -1.0, 0.0),
        ));
    }

    // car chassis
    {
        // y is vertical
        let x = 0.0;
        let y = 20.0;
        let z = 0.0;

        // half sizes
        let xs = 4.0;
        let ys = 0.5;
        let zs = 2.0;

        commands.spawn((
            RigidBody::Dynamic,
            Collider::cuboid(xs, ys, zs),
            // ColliderDebugColor(Hsla::hsl(220.0, 1.0, 0.3)),
            Mesh3d(meshes.add(Cuboid::new(xs * 2.0, ys * 2.0, zs * 2.0))),
            MeshMaterial3d(materials.add(Color::from(SILVER))),
            Transform::from_xyz(x, y, z).with_rotation(Quat::from_rotation_x(0.2)),
            Car,
        ));
        // TODO(lucasw) this force is constant, applied all the time, can't be modified?
        // .insert(ExternalForce {
        //     force: Vec3::new(0.0, 0.0, 0.0),
        //     torque: Vec3::new(0.0, 0.0, 0.0),
        // });
    }
}

pub fn cast_ray(
    mut commands: Commands,
    // rapier_context: Res<RapierContext>,
    // mut query: Query<(Entity, &mut ExternalForce, &GlobalTransform), With<Car>>,
) {
    /*
    let max_length = 0.4;
    let mut force_torque = ExternalForce {
        force: Vec3::new(0.0, 0.0, 0.0),
        torque: Vec3::new(0.0, 0.0, 0.0),
    };
    for (car, mut external_force, car_tf) in query.iter_mut() {
        // four wheels
        // TODO(lucasw) need to use Car size as set above in xs, ys, & zs
        let wheel_positions = vec![
            car_tf.translation() + car_tf.forward() * 0.8 + car_tf.down() * 0.6 + car_tf.left() * 1.0,
            car_tf.translation() + car_tf.forward() * 0.8 + car_tf.down() * 0.6 + car_tf.right() * 1.0,
            car_tf.translation() + car_tf.back() * 0.8 + car_tf.down() * 0.6 + car_tf.left() * 1.0,
            car_tf.translation() + car_tf.back() * 0.8 + car_tf.down() * 0.6 + car_tf.right() * 1.0,
        ];
        for wheel_pos in wheel_positions {
            let hit = rapier_context.cast_ray_and_get_normal(
                // TODO(lucasw) get location from car outside the chassis collision volume
                wheel_pos,
                *car_tf.down(),
                max_length,  // f32::MAX,
                false,
                QueryFilter::default(),
                // TODO(lucasw) how to make a queryfilter to exclude the car?
                //QueryFilter::new(<Without<Car>>),
            );
            // println!("translation: {:?}, down: {:?} -> {hit:?}", car_tf.translation(), car_tf.down());

            if let Some((hit_entity, intersection)) = hit {
                // let dist = (intersection.point - wheel_pos).length();
                // println!("{intersection:?}, {car} {external_force:?}");
                let compression = max_length - intersection.time_of_impact;
                let force = intersection.normal * compression * 700.0;
                // TODO(lucasw) external force is unchanged by this
                force_torque += ExternalForce::at_point(force, wheel_pos, car_tf.translation());

                // println!("{external_force:?}");

                // *rigid_body.add_force(force, true);
                // Color in blue the entity we just hit.
                // Because of the query filter, only colliders attached to a dynamic body
                // will get an event.
                let color = bevy::color::palettes::basic::BLUE.into();
                commands.entity(hit_entity).insert(ColliderDebugColor(color));
            }
        }
        external_force.force = force_torque.force;
        external_force.torque = force_torque.torque;
    }
    */
}
