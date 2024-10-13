//! Modify vertices of a cube made out of beams

use bevy::{
    prelude::*,
    render::mesh::{Indices, PrimitiveTopology, VertexAttributeValues},
};

#[derive(Component)]
struct AnimatedPosition;

struct ModifyVerts {
    i: u64,
}

impl ModifyVerts {
    fn update_mesh(
        &mut self,
        time: Res<Time>,
        query: Query<(&Transform, &Handle<Mesh>), With<AnimatedPosition>>,
        mut assets: ResMut<Assets<Mesh>>,
    ) {
        // if self.i % 60 == 0 {
        //     println!("update {:.3}", time.elapsed_seconds());
        // }
        for (_transform, handle) in &query {
            let mesh = assets.get_mut(handle.id());
            if let Some(mesh) = mesh {
                let positions = mesh.attribute(Mesh::ATTRIBUTE_POSITION);
                if let Some(VertexAttributeValues::Float32x3(positions)) = positions {
                    /*
                    if self.i % 180 == 0 {
                        println!("{positions:?}");
                    }
                    */
                    let mut new_positions = Vec::new();
                    for pos in positions {
                        new_positions.push([
                            pos[0] * (1.0 + 0.01 * time.elapsed_seconds().sin()),
                            pos[1],
                            pos[2],
                        ]);
                    }
                    mesh.insert_attribute(Mesh::ATTRIBUTE_POSITION, new_positions);
                }
            }
            // transform.translation.x = time.elapsed_seconds().sin();
            // transform.rotation = Quat::from_rotation_z(FRAC_PI_2 * time.elapsed_seconds().sin());
        }
        self.i += 1;
    }
}

fn main() {
    let mut modify_verts = ModifyVerts { i: 0 };

    App::new()
        .add_plugins(DefaultPlugins)
        .add_systems(Startup, setup)
        .add_systems(
            Update,
            (
                move |tm: Res<Time>,
                      qr: Query<(&Transform, &Handle<Mesh>), With<AnimatedPosition>>,
                      at: ResMut<Assets<Mesh>>| {
                    modify_verts.update_mesh(tm, qr, at)
                },
                bevy_test::camera_controller,
            ),
        )
        .run();
}

fn setup_rect(x0: f32, y0: f32, z0: f32, wd: f32, ht: f32, dp: f32) -> Vec<(Mesh, Color)> {
    let mut mesh_colors = Vec::new();

    {
        // top
        let pts = vec![
            [x0 + -wd, y0 + ht, z0 - dp],
            [x0 + -wd, y0 + ht, z0 + dp],
            [x0 + wd, y0 + ht, z0 - dp],
            [x0 + wd, y0 + ht, z0 + dp],
        ];
        let num_pts = pts.len();
        let mesh = Mesh::new(PrimitiveTopology::TriangleList)
            .with_inserted_attribute(Mesh::ATTRIBUTE_POSITION, pts)
            .with_inserted_attribute(Mesh::ATTRIBUTE_NORMAL, vec![[0.0, 1.0, 0.0]; num_pts])
            .with_indices(Some(Indices::U16(vec![0, 1, 2, 1, 3, 2])));

        mesh_colors.push((mesh, Color::rgb(0.5, 0.5, 0.0)));
    }

    {
        // front
        let pts = vec![
            [x0 + -wd, y0 - ht, z0 + dp],
            [x0 + wd, y0 - ht, z0 + dp],
            [x0 + -wd, y0 + ht, z0 + dp],
            [x0 + wd, y0 + ht, z0 + dp],
        ];
        let num_pts = pts.len();
        let mesh = Mesh::new(PrimitiveTopology::TriangleList)
            .with_inserted_attribute(Mesh::ATTRIBUTE_POSITION, pts)
            .with_inserted_attribute(Mesh::ATTRIBUTE_NORMAL, vec![[0.0, 0.0, 1.0]; num_pts])
            .with_indices(Some(Indices::U16(vec![0, 1, 2, 1, 3, 2])));

        mesh_colors.push((mesh, Color::rgb(0.2, 0.75, 0.0)));
    }

    {
        // right
        let pts = vec![
            [x0 + wd, y0 - ht, z0 + dp],
            [x0 + wd, y0 - ht, z0 - dp],
            [x0 + wd, y0 + ht, z0 + dp],
            [x0 + wd, y0 + ht, z0 - dp],
        ];
        let num_pts = pts.len();
        let mesh = Mesh::new(PrimitiveTopology::TriangleList)
            .with_inserted_attribute(Mesh::ATTRIBUTE_POSITION, pts)
            .with_inserted_attribute(Mesh::ATTRIBUTE_NORMAL, vec![[1.0, 0.0, 0.0]; num_pts])
            .with_indices(Some(Indices::U16(vec![0, 1, 2, 1, 3, 2])));

        mesh_colors.push((mesh, Color::rgb(0.8, 0.15, 0.0)));
    }

    {
        // back
        let pts = vec![
            [x0 + wd, y0 - ht, z0 - dp],
            [x0 + -wd, y0 - ht, z0 - dp],
            [x0 + wd, y0 + ht, z0 - dp],
            [x0 + -wd, y0 + ht, z0 - dp],
        ];
        let num_pts = pts.len();
        let mesh = Mesh::new(PrimitiveTopology::TriangleList)
            .with_inserted_attribute(Mesh::ATTRIBUTE_POSITION, pts)
            .with_inserted_attribute(Mesh::ATTRIBUTE_NORMAL, vec![[0.0, 0.0, -1.0]; num_pts])
            .with_indices(Some(Indices::U16(vec![0, 1, 2, 1, 3, 2])));

        mesh_colors.push((mesh, Color::rgb(0.2, 0.75, 0.0)));
    }

    {
        // left
        let pts = vec![
            [x0 + -wd, y0 - ht, z0 - dp],
            [x0 + -wd, y0 - ht, z0 + dp],
            [x0 + -wd, y0 + ht, z0 - dp],
            [x0 + -wd, y0 + ht, z0 + dp],
        ];
        let num_pts = pts.len();
        let mesh = Mesh::new(PrimitiveTopology::TriangleList)
            .with_inserted_attribute(Mesh::ATTRIBUTE_POSITION, pts)
            .with_inserted_attribute(Mesh::ATTRIBUTE_NORMAL, vec![[0.8, 0.2, 0.0]; num_pts])
            .with_indices(Some(Indices::U16(vec![0, 1, 2, 1, 3, 2])));

        mesh_colors.push((mesh, Color::rgb(0.8, 0.25, 0.0)));
    }
    mesh_colors
}

fn setup(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    // Create a camera
    commands.spawn((
        Camera3dBundle {
            transform: Transform::from_xyz(-2.0, 5.0, 5.0)
                .looking_at(Vec3::new(0.0, 3.0, 0.0), Vec3::Y),
            ..default()
        },
        bevy_test::CameraController::default(),
    ));

    // Some light to see something
    commands.spawn(PointLightBundle {
        point_light: PointLight {
            intensity: 7000.,
            range: 50.,
            shadows_enabled: true,
            ..default()
        },
        transform: Transform::from_xyz(8., 12., 8.),
        ..default()
    });

    // ground plane
    commands.spawn(PbrBundle {
        mesh: meshes.add(shape::Plane::from_size(50.).into()),
        material: materials.add(Color::SILVER.into()),
        ..default()
    });

    let mut mc = Vec::new();
    mc.push(setup_rect(-1.0, 3.0, -1.0, 0.05, 1.0, 0.1));
    mc.push(setup_rect(-1.0, 3.0, 1.0, 0.05, 1.0, 0.1));
    mc.push(setup_rect(1.0, 3.0, -1.0, 0.05, 1.0, 0.1));
    mc.push(setup_rect(1.0, 3.0, 1.0, 0.05, 1.0, 0.1));

    mc.push(setup_rect(0.0, 2.0, -1.0, 1.0, 0.05, 0.12));
    mc.push(setup_rect(0.0, 2.0, 1.0, 1.0, 0.05, 0.12));
    mc.push(setup_rect(0.0, 4.0, -1.0, 1.0, 0.05, 0.12));
    mc.push(setup_rect(0.0, 4.0, 1.0, 1.0, 0.05, 0.12));

    mc.push(setup_rect(-1.0, 2.0, 0.0, 0.05, 0.05, 1.0));
    mc.push(setup_rect(1.0, 2.0, 0.0, 0.05, 0.05, 1.0));
    mc.push(setup_rect(-1.0, 4.0, 0.0, 0.05, 0.05, 1.0));
    mc.push(setup_rect(1.0, 4.0, 0.0, 0.05, 0.05, 1.0));

    for mesh_colors in mc {
        for (mesh, color) in mesh_colors {
            let mesh = meshes.add(mesh);
            commands.spawn((
                AnimatedPosition,
                PbrBundle {
                    mesh,
                    material: materials.add(color.into()),
                    ..default()
                },
            ));
        }
    }
}
