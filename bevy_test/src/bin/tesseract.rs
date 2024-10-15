//! Modify vertices of a cube made out of beams

use bevy::{
    prelude::*,
    render::mesh::{Indices, PrimitiveTopology, VertexAttributeValues},
};
use nalgebra::base::{Vector3, Vector4};

#[derive(Component)]
struct AnimatedPosition;

struct Beams4D {
    i: u64,
    // list of points that make up triangle lists
    points_4d: Vec<Vec<Vector4<f64>>>,
}

impl Beams4D {
    fn build_points(
        &mut self,
        x0: f64,
        xsz: f64,
        y0: f64,
        ysz: f64,
        z0: f64,
        zsz: f64,
        w0: f64,
        wsz: f64,
    ) {
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
        let yw1 = vec![
            Vector4::new(xs[1], ys[0], zs[0], ws[0]),
            Vector4::new(xs[1], ys[0], zs[0], ws[1]),
            Vector4::new(xs[1], ys[1], zs[0], ws[0]),
            Vector4::new(xs[1], ys[1], zs[0], ws[1]),
        ];
        let yw2 = vec![
            Vector4::new(xs[0], ys[0], zs[1], ws[0]),
            Vector4::new(xs[0], ys[0], zs[1], ws[1]),
            Vector4::new(xs[0], ys[1], zs[1], ws[0]),
            Vector4::new(xs[0], ys[1], zs[1], ws[1]),
        ];
        let yw3 = vec![
            Vector4::new(xs[1], ys[0], zs[1], ws[0]),
            Vector4::new(xs[1], ys[0], zs[1], ws[1]),
            Vector4::new(xs[1], ys[1], zs[1], ws[0]),
            Vector4::new(xs[1], ys[1], zs[1], ws[1]),
        ];

        let zw0 = vec![
            Vector4::new(xs[0], ys[0], zs[0], ws[0]),
            Vector4::new(xs[0], ys[0], zs[0], ws[1]),
            Vector4::new(xs[0], ys[0], zs[1], ws[0]),
            Vector4::new(xs[0], ys[0], zs[1], ws[1]),
        ];
        let zw1 = vec![
            Vector4::new(xs[1], ys[0], zs[0], ws[0]),
            Vector4::new(xs[1], ys[0], zs[0], ws[1]),
            Vector4::new(xs[1], ys[0], zs[1], ws[0]),
            Vector4::new(xs[1], ys[0], zs[1], ws[1]),
        ];
        let zw2 = vec![
            Vector4::new(xs[0], ys[1], zs[0], ws[0]),
            Vector4::new(xs[0], ys[1], zs[0], ws[1]),
            Vector4::new(xs[0], ys[1], zs[1], ws[0]),
            Vector4::new(xs[0], ys[1], zs[1], ws[1]),
        ];
        let zw3 = vec![
            Vector4::new(xs[1], ys[1], zs[0], ws[0]),
            Vector4::new(xs[1], ys[1], zs[0], ws[1]),
            Vector4::new(xs[1], ys[1], zs[1], ws[0]),
            Vector4::new(xs[1], ys[1], zs[1], ws[1]),
        ];

        let points_4d = vec![
            xy0, xy1, xy2, xy3, xz0, xz1, xz2, xz3, yz0, yz1, yz2, yz3, xw0, xw1, xw2, xw3, yw0,
            yw1, yw2, yw3, zw0, zw1, zw2, zw3,
        ];
        self.points_4d.extend(points_4d);
    }

    // get points and a normal
    fn points_to_3d(points_4d: &Vec<Vector4<f64>>, rot_xw: f64, rot_yw: f64, rot_zw: f64) -> (Vec<Vec3>, Vec<f32>) {
        let mut points_3d_vec = Vec::new();
        let mut points_3d = Vec::new();

        let mat_xw = nalgebra::base::Matrix4::new(
            rot_xw.cos(), 0., 0., rot_xw.sin(),
            0., 1.0, 0., 0.,
            0., 0., 1.0, 0.,
            -rot_xw.sin(), 0., 0., rot_xw.cos());

        let mat_yw = nalgebra::base::Matrix4::new(
            1.0, 0., 0., 0.,
            0., rot_xw.cos(),  0., rot_xw.sin(),
            0., 0., 1.0, 0.,
            0., -rot_xw.sin(), 0., rot_xw.cos());

        let mat_zw = nalgebra::base::Matrix4::new(
            1.0, 0., 0., 0.,
            0., 1.0, 0., 0.,
            0., 0., rot_xw.cos(), rot_xw.sin(),
            0., 0., -rot_xw.sin(), rot_xw.cos());

        for pt4 in points_4d {
            let pt4 = mat_xw * mat_yw * mat_zw * pt4;

            let sc = 3.0 / (3.0 + pt4[3].abs());
            let pt = Vector3::new(pt4[0] * sc, pt4[1] * sc, pt4[2] * sc);
            points_3d.push(Vec3::new(pt[0] as f32, pt[1] as f32, pt[2] as f32));
            points_3d_vec.push(pt);
        }

        let v0 = (points_3d_vec[1] - points_3d_vec[0]).normalize();
        let v1 = (points_3d_vec[2] - points_3d_vec[0]).normalize();
        // let normal = v0.cross(&v1);
        let normal = v1.cross(&v0);

        (
            points_3d,
            vec![normal[0] as f32, normal[1] as f32, normal[2] as f32],
        )
    }

    fn get_meshes(&self) -> Vec<(Mesh, Color)> {
        let mut meshes_colors = Vec::new();

        let rot_xw = 0.0;
        let rot_yw = 0.0;
        let rot_zw = 0.0;

        for (ind, pts4d) in self.points_4d.iter().enumerate() {
            let (points_3d, normal) = Beams4D::points_to_3d(pts4d, rot_xw, rot_yw, rot_zw);
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
            let color = Color::rgb((indf * 0.1) % 1.0, (1.0 - indf * 0.05) % 1.0, (indf * 0.01) % 1.0);
            // let color = Color::rgb(0.65, 0.6, 0.5);
            println!("{ind} {indf} {color:?}");
            meshes_colors.push((mesh, color));
        }
        meshes_colors
    }

    fn update_mesh(
        &mut self,
        time: Res<Time>,
        query: Query<(&Transform, &Handle<Mesh>), With<AnimatedPosition>>,
        mut assets: ResMut<Assets<Mesh>>,
    ) {
        // if self.i % 60 == 0 {
        //     println!("update {:.3}", time.elapsed_seconds());
        // }
        //
        let rot_xw = self.i as f64 * 0.005;
        let rot_yw = self.i as f64 * 0.000703;
        let rot_zw = self.i as f64 * 0.00031;
        println!("xw: {rot_xw:.2}, yw: {rot_yw:.2}, zw: {rot_zw:.2}");

        for (ind, (_transform, handle)) in query.iter().enumerate() {
            let mesh = assets.get_mut(handle.id());
            // println!("{:?}", handle.id());
            if let Some(mesh) = mesh {
                let positions = mesh.attribute(Mesh::ATTRIBUTE_POSITION);
                if let Some(VertexAttributeValues::Float32x3(positions)) = positions {
                    // if self.i % 180 == 0 {
                    // }
                    let pts4d = &self.points_4d[ind];
                    let (points_3d, normal) = Beams4D::points_to_3d(pts4d, rot_xw, rot_yw, rot_zw);
                    mesh.insert_attribute(Mesh::ATTRIBUTE_POSITION, points_3d);
                    // mesh.insert_attribute(Mesh::ATTRIBUTE_NORMAL, normal);
                }
            }
            // transform.translation.x = time.elapsed_seconds().sin();
            // transform.rotation = Quat::from_rotation_z(FRAC_PI_2 * time.elapsed_seconds().sin());
        }
        self.i += 1;
    }
}

fn main() {
    let mut beams_4d = Beams4D {
        i: 0,
        points_4d: Vec::new(),
    };

    let woff = 1.5;

    // build beams of a tesseract
    for yi in 0..2 {
        for zi in 0..2 {
            for wi in 0..2 {
                let x0 = 0.0; // xi as f64 * 2.0 - 1.0;
                let xsz = 1.0;
                let y0 = yi as f64 * 2.0 - 1.0;
                let ysz = 0.06;
                let z0 = zi as f64 * 2.0 - 1.0;
                let zsz = 0.07;
                let w0 = woff + wi as f64 * 2.0 - 1.0;
                let wsz = 0.08;
                beams_4d.build_points(x0, xsz, y0, ysz, z0, zsz, w0, wsz);
            }
        }
    }

    for xi in 0..2 {
        for zi in 0..2 {
            for wi in 0..2 {
                let x0 = xi as f64 * 2.0 - 1.0;
                let xsz = 0.05;
                let y0 = 0.0; // yi as f64 * 2.0 - 1.0;
                let ysz = 1.0;
                let z0 = zi as f64 * 2.0 - 1.0;
                let zsz = 0.07;
                let w0 = woff + wi as f64 * 2.0 - 1.0;
                let wsz = 0.08;
                beams_4d.build_points(x0, xsz, y0, ysz, z0, zsz, w0, wsz);
            }
        }
    }

    for xi in 0..2 {
        for yi in 0..2 {
            for wi in 0..2 {
                let x0 = xi as f64 * 2.0 - 1.0;
                let xsz = 0.05;
                let y0 = yi as f64 * 2.0 - 1.0;
                let ysz = 0.06;
                let z0 = 0.0;
                let zsz = 1.0;
                let w0 = woff + wi as f64 * 2.0 - 1.0;
                let wsz = 0.08;
                beams_4d.build_points(x0, xsz, y0, ysz, z0, zsz, w0, wsz);
            }
        }
    }

    for xi in 0..2 {
        for yi in 0..2 {
            for zi in 0..2 {
                let x0 = xi as f64 * 2.0 - 1.0;
                let xsz = 0.05;
                let y0 = yi as f64 * 2.0 - 1.0;
                let ysz = 0.06;
                let z0 = zi as f64 * 2.0 - 1.0;
                let zsz = 0.07;
                let w0 = woff;
                let wsz = 1.0;
                beams_4d.build_points(x0, xsz, y0, ysz, z0, zsz, w0, wsz);
            }
        }
    }

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
                move |tm: Res<Time>,
                      qr: Query<(&Transform, &Handle<Mesh>), With<AnimatedPosition>>,
                      at: ResMut<Assets<Mesh>>| {
                    beams_4d.update_mesh(tm, qr, at)
                },
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
            transform: Transform::from_xyz(-2.0, 0.0, 5.0)
                .looking_at(Vec3::new(0.0, 0.0, 0.0), Vec3::Y),
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
        mesh: meshes.add(shape::Plane::from_size(150.).into()),
        material: materials.add(Color::SILVER.into()),
        transform: Transform::from_xyz(0.0, -2.0, 0.0),
        ..default()
    });
}
