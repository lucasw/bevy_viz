//! Modify vertices of a cube made out of beams

use bevy::{
    prelude::*,
    render::mesh::{Indices, PrimitiveTopology, VertexAttributeValues},
};
use nalgebra::base::{Vector3, Vector4};

#[derive(Component)]
struct AnimatedPosition;

struct Beams4D {
    _i: u64,
    // list of points that make up triangle lists
    points_4d: Vec<Vec<Vector4<f64>>>,
}

impl Beams4D {
    fn build_points(&mut self) {
        // each of these will be a plane made of four points
        let x0 = 0.0;
        let xsz = 0.2;

        let y0 = 4.0;
        let ysz = 0.3;

        let z0 = 0.0;
        let zsz = 0.4;

        let w0 = 1.0;
        let wsz = 1.0;

        let xs = [x0 - xsz, x0 + xsz];
        let ys = [y0 - ysz, y0 + ysz];
        let zs = [z0 - zsz, z0 + zsz];
        let ws = [w0 - wsz, w0 + wsz];

        // for every plane there are two dimensions, so 6 planes total:
        // xy, xz, xw, yz, yw, zw

        // TODO(lucasw) later make 4 of each, for all combinations of the other two dimensions
        // fixed
        // TODO(lucasw) could make a nested for loop for these if xs..ws were also in an array
        let xy0 = vec![
            Vector4::new(xs[0], ys[0], zs[0], ws[0]),
            Vector4::new(xs[1], ys[0], zs[0], ws[0]),
            Vector4::new(xs[0], ys[1], zs[0], ws[0]),
            Vector4::new(xs[1], ys[1], zs[0], ws[0]),
        ];
        let xy1 = vec![
            Vector4::new(xs[0], ys[0], zs[1], ws[0]),
            Vector4::new(xs[0], ys[1], zs[1], ws[0]),
            Vector4::new(xs[1], ys[0], zs[1], ws[0]),
            Vector4::new(xs[1], ys[1], zs[1], ws[0]),
        ];
        let xy2 = vec![
            Vector4::new(xs[0], ys[0], zs[0], ws[1]),
            Vector4::new(xs[1], ys[0], zs[0], ws[1]),
            Vector4::new(xs[0], ys[1], zs[0], ws[1]),
            Vector4::new(xs[1], ys[1], zs[0], ws[1]),
        ];
        let xy3 = vec![
            Vector4::new(xs[0], ys[0], zs[1], ws[1]),
            Vector4::new(xs[0], ys[1], zs[1], ws[1]),
            Vector4::new(xs[1], ys[0], zs[1], ws[1]),
            Vector4::new(xs[1], ys[1], zs[1], ws[1]),
        ];

        let xz0 = vec![
            Vector4::new(xs[0], ys[0], zs[0], ws[0]),
            Vector4::new(xs[0], ys[0], zs[1], ws[0]),
            Vector4::new(xs[1], ys[0], zs[0], ws[0]),
            Vector4::new(xs[1], ys[0], zs[1], ws[0]),
        ];
        let xz1 = vec![
            Vector4::new(xs[0], ys[1], zs[0], ws[0]),
            Vector4::new(xs[1], ys[1], zs[0], ws[0]),
            Vector4::new(xs[0], ys[1], zs[1], ws[0]),
            Vector4::new(xs[1], ys[1], zs[1], ws[0]),
        ];
        let xz2 = vec![
            Vector4::new(xs[0], ys[0], zs[0], ws[1]),
            Vector4::new(xs[0], ys[0], zs[1], ws[1]),
            Vector4::new(xs[1], ys[0], zs[0], ws[1]),
            Vector4::new(xs[1], ys[0], zs[1], ws[1]),
        ];
        let xz3 = vec![
            Vector4::new(xs[0], ys[1], zs[0], ws[1]),
            Vector4::new(xs[1], ys[1], zs[0], ws[1]),
            Vector4::new(xs[0], ys[1], zs[1], ws[1]),
            Vector4::new(xs[1], ys[1], zs[1], ws[1]),
        ];

        let xw0 = vec![
            Vector4::new(xs[0], ys[0], zs[0], ws[0]),
            Vector4::new(xs[0], ys[0], zs[0], ws[1]),
            Vector4::new(xs[1], ys[0], zs[0], ws[0]),
            Vector4::new(xs[1], ys[0], zs[0], ws[1]),
        ];
        let xw1 = vec![
            Vector4::new(xs[0], ys[0], zs[1], ws[0]),
            Vector4::new(xs[1], ys[0], zs[1], ws[0]),
            Vector4::new(xs[0], ys[0], zs[1], ws[1]),
            Vector4::new(xs[1], ys[0], zs[1], ws[1]),
        ];
        let xw2 = vec![
            Vector4::new(xs[0], ys[1], zs[0], ws[0]),
            Vector4::new(xs[1], ys[1], zs[0], ws[0]),
            Vector4::new(xs[0], ys[1], zs[0], ws[1]),
            Vector4::new(xs[1], ys[1], zs[0], ws[1]),
        ];
        let xw3 = vec![
            Vector4::new(xs[0], ys[1], zs[1], ws[0]),
            Vector4::new(xs[1], ys[1], zs[1], ws[0]),
            Vector4::new(xs[0], ys[1], zs[1], ws[1]),
            Vector4::new(xs[1], ys[1], zs[1], ws[1]),
        ];

        let yz0 = vec![
            Vector4::new(xs[0], ys[0], zs[0], ws[0]),
            Vector4::new(xs[0], ys[1], zs[0], ws[0]),
            Vector4::new(xs[0], ys[0], zs[1], ws[0]),
            Vector4::new(xs[0], ys[1], zs[1], ws[0]),
        ];
        let yz1 = vec![
            Vector4::new(xs[1], ys[0], zs[0], ws[0]),
            Vector4::new(xs[1], ys[0], zs[1], ws[0]),
            Vector4::new(xs[1], ys[1], zs[0], ws[0]),
            Vector4::new(xs[1], ys[1], zs[1], ws[0]),
        ];
        let yz2 = vec![
            Vector4::new(xs[0], ys[0], zs[0], ws[1]),
            Vector4::new(xs[0], ys[1], zs[0], ws[1]),
            Vector4::new(xs[0], ys[0], zs[1], ws[1]),
            Vector4::new(xs[0], ys[1], zs[1], ws[1]),
        ];
        let yz3 = vec![
            Vector4::new(xs[1], ys[0], zs[0], ws[1]),
            Vector4::new(xs[1], ys[0], zs[1], ws[1]),
            Vector4::new(xs[1], ys[1], zs[0], ws[1]),
            Vector4::new(xs[1], ys[1], zs[1], ws[1]),
        ];

        let yw0 = vec![
            Vector4::new(xs[0], ys[0], zs[0], ws[0]),
            Vector4::new(xs[0], ys[0], zs[0], ws[1]),
            Vector4::new(xs[0], ys[1], zs[0], ws[0]),
            Vector4::new(xs[0], ys[1], zs[0], ws[1]),
        ];

        let zw0 = vec![
            Vector4::new(xs[0], ys[0], zs[0], ws[0]),
            Vector4::new(xs[0], ys[0], zs[0], ws[1]),
            Vector4::new(xs[0], ys[0], zs[1], ws[0]),
            Vector4::new(xs[0], ys[0], zs[1], ws[1]),
        ];

        // self.points_4d = vec![xy0, xz0, xw0, yz0, yw0, zw0];
        self.points_4d = vec![
            xy0, xy1, xy2, xy3, xz0, xz1, xz2, xz3, yz0, yz1, yz2, yz3, xw0, xw1, xw2, xw3, yw0,
            zw0,
        ];
    }

    // get points and a normal
    fn points_to_3d(points_4d: &Vec<Vector4<f64>>) -> (Vec<Vec3>, Vec<f32>) {
        let mut points_3d_vec = Vec::new();
        let mut points_3d = Vec::new();

        for pt4 in points_4d {
            let sc = 1.0 / (1.0 + pt4[3].abs());
            let pt = Vector3::new(pt4[0] * sc, pt4[1] * sc, pt4[2] * sc);
            points_3d.push(Vec3::new(pt[0] as f32, pt[1] as f32, pt[2] as f32));
            points_3d_vec.push(pt);
        }

        let v0 = (points_3d_vec[1] - points_3d_vec[0]).normalize();
        let v1 = (points_3d_vec[2] - points_3d_vec[0]).normalize();
        // let normal = v0.cross(&v1);
        let normal = v1.cross(&v0);

        /*
        let points_3d = [
            points_3d[0],
            points_3d[1],
            points_3d[2],
            points_3d[3],
        ];
        */

        (
            points_3d,
            vec![normal[0] as f32, normal[1] as f32, normal[2] as f32],
        )
    }

    fn get_meshes(&self) -> Vec<(Mesh, Color)> {
        let mut meshes_colors = Vec::new();

        for (ind, pts4d) in self.points_4d.iter().enumerate() {
            let (points_3d, normal) = Beams4D::points_to_3d(pts4d);
            let num_pts = pts4d.len();
            let mesh = Mesh::new(PrimitiveTopology::TriangleList)
                .with_inserted_attribute(Mesh::ATTRIBUTE_POSITION, points_3d)
                .with_inserted_attribute(
                    Mesh::ATTRIBUTE_NORMAL,
                    vec![[normal[0], normal[1], normal[2]]; num_pts],
                )
                // .with_indices(Some(Indices::U16(vec![0, 1, 2, 1, 3, 2])));
                .with_indices(Some(Indices::U16(vec![0, 2, 1, 1, 2, 3])));

            let indf = ind as f32;
            let color = Color::rgb(indf * 0.1, 1.0 - indf * 0.05, indf * 0.01);
            println!("{ind} {indf} {color:?}");
            meshes_colors.push((mesh, color));
        }
        meshes_colors
    }

    /*
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
            // println!("{:?}", handle.id());
            if let Some(mesh) = mesh {
                let positions = mesh.attribute(Mesh::ATTRIBUTE_POSITION);
                if let Some(VertexAttributeValues::Float32x3(positions)) = positions {
                    // if self.i % 180 == 0 {
                    // }
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
    */
}

fn main() {
    let mut beams_4d = Beams4D {
        i: 0,
        points_4d: Vec::new(),
    };
    beams_4d.build_points();
    // TODO(lucasw) double clone seems wrong, but it works
    let raw_meshes = beams_4d.get_meshes();

    App::new()
        .add_plugins(DefaultPlugins)
        .add_systems(
            Startup,
            move |commands: Commands,
                  meshes: ResMut<Assets<Mesh>>,
                  materials: ResMut<Assets<StandardMaterial>>| {
                setup(raw_meshes.clone(), commands, meshes, materials);
            },
        )
        .add_systems(
            Update,
            (
                /*
                move |tm: Res<Time>,
                      qr: Query<(&Transform, &Handle<Mesh>), With<AnimatedPosition>>,
                      at: ResMut<Assets<Mesh>>| {
                    beams_4d.update_mesh(tm, qr, at)
                },
                */
                bevy_test::camera_controller,
            ),
        )
        .run();
}

fn setup(
    meshes_colors: Vec<(Mesh, Color)>,
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

    for (mesh, color) in meshes_colors {
        let mesh = meshes.add(mesh);

        // TODO(lucasw) modify_vert needs to know about these mesh ids,
        // but not sure how to get them across
        println!("id {:?}", mesh.id());
        let _ = commands
            .spawn((
                AnimatedPosition,
                PbrBundle {
                    mesh,
                    material: materials.add(StandardMaterial {
                        base_color: color,
                        diffuse_transmission: 0.6,
                        perceptual_roughness: 0.5,
                        reflectance: 1.0,
                        double_sided: true,
                        cull_mode: None,
                        ..Default::default()
                    }),
                    ..default()
                },
            ))
            .id();
    }

    // ground plane
    commands.spawn(PbrBundle {
        mesh: meshes.add(shape::Plane::from_size(50.).into()),
        material: materials.add(Color::SILVER.into()),
        ..default()
    });
}
