use bevy::prelude::*;
use bevy_rapier3d::prelude::*;
// use rapier3d::prelude::*;

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
        .run();
}

pub fn setup_graphics(mut commands: Commands) {
    commands.spawn(Camera3dBundle {
        transform: Transform::from_xyz(-30.0, 30.0, 100.0)
            .looking_at(Vec3::new(0.0, 10.0, 0.0), Vec3::Y),
        ..Default::default()
    });
}

pub fn setup_physics(mut commands: Commands) {
    // ground
    {
        let ground_size = 200.1;
        let ground_height = 0.1;

        commands.spawn((
                TransformBundle::from(Transform::from_xyz(0.0, -ground_height, 0.0)),
                Collider::cuboid(ground_size, ground_height, ground_size),
        ));
    }

    // car chassis
    {
        // y is vertical
        let x = 0.0;
        let y = 20.0;
        let z = 0.0;

        commands
            .spawn(TransformBundle::from(Transform::from_rotation(
                        Quat::from_rotation_x(0.4) * Quat::from_rotation_z(0.2),
            )))
            .with_children(|child| {
                child.spawn((
                        TransformBundle::from(Transform::from_xyz(x, y, z)),
                        RigidBody::Dynamic,
                        Collider::cuboid(4.0, 0.5, 2.0),
                        ColliderDebugColor(Hsla::hsl(220.0, 1.0, 0.3)),
                ));
            });
    }
}
