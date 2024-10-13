//! Skinned mesh example with mesh and joints data defined in code.
//! Example taken from <https://github.com/KhronosGroup/glTF-Tutorials/blob/master/gltfTutorial/gltfTutorial_019_SimpleSkin.md>

use std::f32::consts::*;

use bevy::{
    prelude::*,
    render::mesh::{
        skinning::{SkinnedMesh, SkinnedMeshInverseBindposes},
        Indices, PrimitiveTopology, VertexAttributeValues,
    },
};
use rand::{rngs::StdRng, Rng, SeedableRng};

fn main() {
    App::new()
        .add_plugins(DefaultPlugins)
        .add_systems(Startup, setup)
        .add_systems(Update, (joint_animation, bevy_test::camera_controller))
        .run();
}

/// Used to mark a joint to be animated in the [`joint_animation`] system.
#[derive(Component)]
struct AnimatedJoint;

fn setup_rect(x0: f32, y0: f32, z0: f32, wd: f32, dp: f32, ht: f32) -> Vec<(Mesh, Color)> {
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
            [x0 + -wd, y0 + 0.0, z0 + dp],
            [x0 + wd, y0 + 0.0, z0 + dp],
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
            [x0 + wd, y0 + 0.0, z0 + dp],
            [x0 + wd, y0 + 0.0, z0 - dp],
            [x0 + wd, y0 + ht, z0 + dp],
            [x0 + wd, y0 + ht, z0 - dp],
        ];
        let num_pts = pts.len();
        let mesh = Mesh::new(PrimitiveTopology::TriangleList)
            .with_inserted_attribute(Mesh::ATTRIBUTE_POSITION, pts)
            .with_inserted_attribute(Mesh::ATTRIBUTE_NORMAL, vec![[1.0, 0.0, 0.0]; num_pts])
            .with_indices(Some(Indices::U16(vec![0, 1, 2, 1, 3, 2])));

        mesh_colors.push((mesh, Color::rgb(0.2, 0.75, 0.0)));
    }

    {
        // back
        let pts = vec![
            [x0 + wd, y0 + 0.0, z0 - dp],
            [x0 + -wd, y0 + 0.0, z0 - dp],
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
            [x0 + -wd, y0 + 0.0, z0 - dp],
            [x0 + -wd, y0 + 0.0, z0 + dp],
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

/// Construct a mesh and a skeleton with 2 joints for that mesh,
///   and mark the second joint to be animated.
/// It is similar to the scene defined in `models/SimpleSkin/SimpleSkin.gltf`
fn setup(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
    mut skinned_mesh_inverse_bindposes_assets: ResMut<Assets<SkinnedMeshInverseBindposes>>,
) {
    // Create a camera
    commands.spawn((
        Camera3dBundle {
            transform: Transform::from_xyz(-2.0, 2.5, 5.0).looking_at(Vec3::ZERO, Vec3::Y),
            ..default()
        },
        bevy_test::CameraController::default(),
    ));

    // Some light to see something
    commands.spawn(PointLightBundle {
        point_light: PointLight {
            intensity: 7000.,
            range: 100.,
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

    let mesh_colors = setup_rect(0.0, 2.0, 0.0, 0.1, 0.15, 1.0);
    for (mesh, color) in mesh_colors {
        let mesh = meshes.add(mesh);
        commands.spawn(PbrBundle {
            mesh,
            material: materials.add(color.into()),
            ..default()
        });
    }

    // Create inverse bindpose matrices for a skeleton consists of 2 joints
    let inverse_bindposes =
        skinned_mesh_inverse_bindposes_assets.add(SkinnedMeshInverseBindposes::from(vec![
            Mat4::from_translation(Vec3::new(-0.5, -1.0, 0.0)),
            Mat4::from_translation(Vec3::new(-0.5, -1.0, 0.0)),
        ]));

    let y0 = 2.0;
    let mesh = Mesh::new(PrimitiveTopology::TriangleList)
        // Set mesh vertex positions
        .with_inserted_attribute(
            Mesh::ATTRIBUTE_POSITION,
            vec![
                [0.0, y0 + 0.0, 0.0],
                [1.0, y0 + 0.0, 0.0],
                [0.0, y0 + 0.5, 0.1],
                [1.0, y0 + 0.5, 0.1],
                [0.0, y0 + 1.0, 0.2],
                [1.0, y0 + 1.0, 0.2],
                [0.0, y0 + 1.5, 0.3],
                [1.0, y0 + 1.5, 0.3],
                [0.0, y0 + 2.0, 0.4],
                [1.0, y0 + 2.0, 0.4],
            ],
        )
        // Set mesh vertex normals
        .with_inserted_attribute(Mesh::ATTRIBUTE_NORMAL, vec![[0.0, 0.0, 1.0]; 10])
        // Set mesh vertex joint indices for mesh skinning.
        // Each vertex gets 4 indices used to address the `JointTransforms` array in the vertex shader
        //  as well as `SkinnedMeshJoint` array in the `SkinnedMesh` component.
        // This means that a maximum of 4 joints can affect a single vertex.
        .with_inserted_attribute(
            Mesh::ATTRIBUTE_JOINT_INDEX,
            // Need to be explicit here as [u16; 4] could be either Uint16x4 or Unorm16x4.
            VertexAttributeValues::Uint16x4(vec![
                [0, 0, 0, 0],
                [0, 0, 0, 0],
                [0, 1, 0, 0],
                [0, 1, 0, 0],
                [0, 1, 0, 0],
                [0, 1, 0, 0],
                [0, 1, 0, 0],
                [0, 1, 0, 0],
                [0, 1, 0, 0],
                [0, 1, 0, 0],
            ]),
        )
        // Set mesh vertex joint weights for mesh skinning.
        // Each vertex gets 4 joint weights corresponding to the 4 joint indices assigned to it.
        // The sum of these weights should equal to 1.
        .with_inserted_attribute(
            Mesh::ATTRIBUTE_JOINT_WEIGHT,
            vec![
                [1.00, 0.00, 0.0, 0.0],
                [1.00, 0.00, 0.0, 0.0],
                [0.75, 0.25, 0.0, 0.0],
                [0.75, 0.25, 0.0, 0.0],
                [0.50, 0.50, 0.0, 0.0],
                [0.50, 0.50, 0.0, 0.0],
                [0.25, 0.75, 0.0, 0.0],
                [0.25, 0.75, 0.0, 0.0],
                [0.00, 1.00, 0.0, 0.0],
                [0.00, 1.00, 0.0, 0.0],
            ],
        )
        // Tell bevy to construct triangles from a list of vertex indices,
        //  where each 3 vertex indices form an triangle.
        .with_indices(Some(Indices::U16(vec![
            0, 1, 3, 0, 3, 2, 2, 3, 5, 2, 5, 4, 4, 5, 7, 4, 7, 6, 6, 7, 9, 6, 9, 8,
        ])));

    let mesh = meshes.add(mesh);

    let mut rng = StdRng::seed_from_u64(42);

    // for i in -2..2
    {
        let i = 0;
        // Create joint entities
        let joint_0 = commands
            .spawn(TransformBundle::from(Transform::from_xyz(
                i as f32 * 1.5,
                0.0,
                i as f32 * 0.1,
            )))
            .id();
        let joint_1 = commands
            .spawn((AnimatedJoint, TransformBundle::IDENTITY))
            .id();

        // Set joint_1 as a child of joint_0.
        commands.entity(joint_0).push_children(&[joint_1]);

        // Each joint in this vector corresponds to each inverse bindpose matrix in `SkinnedMeshInverseBindposes`.
        let joint_entities = vec![joint_0, joint_1];

        // Create skinned mesh renderer. Note that its transform doesn't affect the position of the mesh.
        commands.spawn((
            PbrBundle {
                mesh: mesh.clone(),
                material: materials.add(
                    Color::rgb(
                        rng.gen_range(0.0..1.0),
                        rng.gen_range(0.0..1.0),
                        rng.gen_range(0.0..1.0),
                    )
                    .into(),
                ),
                ..default()
            },
            SkinnedMesh {
                inverse_bindposes: inverse_bindposes.clone(),
                joints: joint_entities,
            },
        ));
    }
}

/// Animate the joint marked with [`AnimatedJoint`] component.
fn joint_animation(time: Res<Time>, mut query: Query<&mut Transform, With<AnimatedJoint>>) {
    for mut transform in &mut query {
        transform.rotation = Quat::from_rotation_z(FRAC_PI_2 * time.elapsed_seconds().sin());
    }
}
