use bevy::prelude::*;
use bevy_rapier3d::prelude::*;
// use rapier3d::prelude::*;

#[derive(Component)]
pub struct Car;

fn main() {
    App::new()
        .insert_resource(ClearColor(Color::srgb(
            0xF9 as f32 / 255.0,
            0xF9 as f32 / 255.0,
            0xFF as f32 / 255.0,
        )))
        .add_plugins((
            DefaultPlugins,
            RapierPhysicsPlugin::<NoUserData>::default(),
            RapierDebugRenderPlugin::default(),
        ))
        .add_systems(Startup, (setup_graphics, setup_physics))
        .add_systems(Update, cast_ray)
        .run();
}

pub fn setup_graphics(mut commands: Commands) {
    commands.spawn(Camera3dBundle {
        transform: Transform::from_xyz(-20.0, 30.0, 50.0)
            .looking_at(Vec3::new(0.0, 10.0, 0.0), Vec3::Y),
        ..Default::default()
    });
}

pub fn setup_physics(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    // Some light to see something
    commands.spawn(PointLightBundle {
        point_light: PointLight {
            intensity: 4_000_000.,
            range: 100.,
            shadows_enabled: true,
            ..default()
        },
        transform: Transform::from_xyz(8., 22., 8.),
        ..default()
    });

    // ground
    {
        let ground_size = 300.1;
        let ground_height = 0.1;

        commands.spawn((
            // TransformBundle::from(Transform::from_xyz(0.0, -ground_height, 0.0)),
            Collider::cuboid(ground_size, ground_height, ground_size),
            PbrBundle {
                mesh: meshes.add(Cuboid::new(
                    ground_size * 2.0,
                    ground_height * 2.0,
                    ground_size * 2.0,
                )),
                material: materials.add(Color::WHITE),
                transform: Transform::from_xyz(0.0, 0.0, 0.0),
                ..default()
            },
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

        commands
            .spawn((
                // TransformBundle::from(Transform::from_rotation(
                //     Quat::from_rotation_x(0.4) * Quat::from_rotation_z(0.2),
                // )),
                RigidBody::Dynamic,
                Collider::cuboid(xs, ys, zs),
                ColliderDebugColor(Hsla::hsl(220.0, 1.0, 0.3)),
                PbrBundle {
                    mesh: meshes.add(Cuboid::new(xs * 2.0, ys * 2.0, zs * 2.0)),
                    material: materials.add(Color::WHITE),
                    transform: Transform::from_xyz(x, y, z),
                    ..default()
                },
                Car,
            ))
            // TODO(lucasw) this force is constant, applied all the time, can't be modified?
            .insert(ExternalForce {
                force: Vec3::new(0.0, 0.0, 0.0),
                torque: Vec3::new(0.0, 0.0, 0.0),
            });
    }
}

pub fn cast_ray(
    mut commands: Commands,
    rapier_context: Res<RapierContext>,
    mut query: Query<(Entity, &mut ExternalForce, &GlobalTransform), With<Car>>,
) {
    let max_length = 1.0;
    for (car, mut external_force, car_transform) in query.iter_mut() {
        let wheel_pos = car_transform.translation() + (car_transform.down() * 0.6);
        let hit = rapier_context.cast_ray_and_get_normal(
            // TODO(lucasw) get location from car outside the chassis collision volume
            wheel_pos,
            *car_transform.down(),
            max_length,  // f32::MAX,
            false,
            QueryFilter::default(),
            // TODO(lucasw) how to make a queryfilter to exclude the car?
            //QueryFilter::new(<Without<Car>>),
        );
        // println!("translation: {:?}, down: {:?} -> {hit:?}", car_transform.translation(), car_transform.down());

        if let Some((hit_entity, intersection)) = hit {
            // let dist = (intersection.point - wheel_pos).length();
            // println!("{intersection:?}, {car} {external_force:?}");
            let compression = max_length - intersection.time_of_impact;
            let force = intersection.normal * compression * 1000.0;
            // TODO(lucasw) external force is unchanged by this
            external_force.force = force;

            // println!("{external_force:?}");

            // *rigid_body.add_force(force, true);
            // Color in blue the entity we just hit.
            // Because of the query filter, only colliders attached to a dynamic body
            // will get an event.
            let color = bevy::color::palettes::basic::BLUE.into();
            commands.entity(hit_entity).insert(ColliderDebugColor(color));
        }
    }
}
