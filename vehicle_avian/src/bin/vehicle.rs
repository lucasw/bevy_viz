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

        let car = commands.spawn((
            RigidBody::Dynamic,
            Collider::cuboid(xs, ys, zs),
            // ColliderDebugColor(Hsla::hsl(220.0, 1.0, 0.3)),
            Mesh3d(meshes.add(Cuboid::new(xs * 2.0, ys * 2.0, zs * 2.0))),
            MeshMaterial3d(materials.add(Color::from(SILVER))),
            Transform::from_xyz(x, y, z).with_rotation(Quat::from_rotation_x(0.2)),
            Car,
            ExternalForce::new(Vec3::new(0.0, 0.0, 0.0)).with_persistence(false),
        ));
        println!("spawned car id: {}", car.id());
    }
}

pub fn cast_ray(
    // mut commands: Commands,
    spatial_query: SpatialQuery,
    mut query: Query<(Entity, &mut ExternalForce, &GlobalTransform), With<Car>>,
) {
    let max_length = 0.4;
    let wheel_y = 0.24;
    for (car, mut external_force, car_tf) in query.iter_mut() {
        // println!("{:?} {external_force:?}", car_tf.translation());
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
        for wheel_pos in wheel_positions {
            let down = *car_tf.down();
            let hit = spatial_query.cast_ray(
                // TODO(lucasw) get location from car outside the chassis collision volume
                wheel_pos,
                Dir3::new(down).unwrap(),
                max_length, // f32::MAX,
                true,
                &SpatialQueryFilter::default().with_excluded_entities([car]),
                // TODO(lucasw) how to make a queryfilter to exclude the object the ray originates
                // from, the car?  But it ought to hit other cars if anyy as well.
                // SpatialQueryFilter::new(<Without<Car>>),
            );
            // println!("translation: {:?}, down: {:?} -> {hit:?}", car_tf.translation(), car_tf.down());

            if let Some(hit) = hit {
                // println!("{intersection:?}, {car} {external_force:?}");
                // if hit.distance > 0.0
                let compression = max_length - hit.distance;
                let force = hit.normal * compression * 70.0;
                // external_force.apply_force(force);
                // println!("wheel: {wheel_pos:?}, {hit:?} {force:?}");
                // TODO(lucasw) external force is unchanged by this
                external_force.apply_force_at_point(force, wheel_pos, car_tf.translation());

                // *rigid_body.add_force(force, true);
                // Color in blue the entity we just hit.
                // Because of the query filter, only colliders attached to a dynamic body
                // will get an event.
                // let color = bevy::color::palettes::basic::BLUE.into();
                // commands.entity(hit_entity).insert(ColliderDebugColor(color));
            }
        }
        println!("external force: {external_force:?}");
    }
}
