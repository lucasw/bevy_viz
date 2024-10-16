//! Modify vertices of a cube made out of beams

use bevy::{
    prelude::*,
    render::mesh::{Indices, PrimitiveTopology}, // , VertexAttributeValues},
};
use nalgebra::base::{Vector3, Vector4};
use std::collections::HashSet;

#[derive(Component)]
struct AnimatedPosition;

struct Beams4D {
    i: u64,
    // list of points that make up triangle lists
    points_4d: Vec<Vec<Vector4<f64>>>,
}

impl Beams4D {
    fn build_points(&mut self, xd: (f64, f64), yd: (f64, f64), zd: (f64, f64), wd: (f64, f64)) {
        let minmax = {
            let (x0, xsz) = xd;
            let (y0, ysz) = yd;
            let (z0, zsz) = zd;
            let (w0, wsz) = wd;
            let xs = [x0 - xsz, x0 + xsz];
            let ys = [y0 - ysz, y0 + ysz];
            let zs = [z0 - zsz, z0 + zsz];
            let ws = [w0 - wsz, w0 + wsz];
            let min = Vector4::new(xs[0], ys[0], zs[0], ws[0]);
            let max = Vector4::new(xs[1], ys[1], zs[1], ws[1]);
            vec![min, max]
        };

        // for every plane there are two dimensions, so 6 planes total:
        // xy, xz, xw, yz, yw, zw

        // make 4 of each, for all combinations of the other two dimensions fixed
        // TODO(lucasw) could make a nested for loop for these if xs..ws were also in an array
        // 4 x 3 x 2 x 1 different surfaces
        let mut used = HashSet::new();
        let mut points_4d = Vec::new();
        let dim = 4;
        for ind0 in 0..dim {
            for ind1 in 0..dim {
                if ind1 == ind0 {
                    continue;
                }
                for ind2 in 0..dim {
                    if ind2 == ind0 || ind2 == ind1 {
                        continue;
                    }
                    for ind3 in 0..dim {
                        if ind3 == ind0 || ind3 == ind1 || ind3 == ind2 {
                            continue;
                        }
                        if used.contains(&(ind0, ind1, ind2, ind3)) {
                            continue;
                        }

                        for k in 0..2 {
                            for l in 0..2 {
                                let mut p0 = Vector4::new(0., 0., 0., 0.);
                                p0[ind0] = minmax[0][ind0];
                                p0[ind1] = minmax[0][ind1];
                                p0[ind2] = minmax[k][ind2];
                                p0[ind3] = minmax[l][ind3];

                                let mut p1 = Vector4::new(0., 0., 0., 0.);
                                p1[ind0] = minmax[1][ind0];
                                p1[ind1] = minmax[0][ind1];
                                p1[ind2] = minmax[k][ind2];
                                p1[ind3] = minmax[l][ind3];

                                let mut p2 = Vector4::new(0., 0., 0., 0.);
                                p2[ind0] = minmax[0][ind0];
                                p2[ind1] = minmax[1][ind1];
                                p2[ind2] = minmax[k][ind2];
                                p2[ind3] = minmax[l][ind3];

                                let mut p3 = Vector4::new(0., 0., 0., 0.);
                                p3[ind0] = minmax[1][ind0];
                                p3[ind1] = minmax[1][ind1];
                                p3[ind2] = minmax[k][ind2];
                                p3[ind3] = minmax[l][ind3];

                                points_4d.push(vec![p0, p1, p2, p3]);
                            }
                        }

                        // these four are the same surfaces
                        used.insert((ind0, ind1, ind2, ind3));
                        used.insert((ind1, ind0, ind2, ind3));

                        used.insert((ind0, ind1, ind3, ind2));
                        used.insert((ind1, ind0, ind3, ind2));
                    }
                }
            }
        }
        // should be 24
        // println!("{} surfaces", points_4d.len());
        self.points_4d.extend(points_4d);
    }

    // get points and a normal
    fn points_to_3d(
        points_4d: &Vec<Vector4<f64>>,
        rot_mat: &nalgebra::base::Matrix4<f64>,
    ) -> (Vec<Vec3>, Vec<f32>) {
        let mut points_3d_vec = Vec::new();
        let mut points_3d = Vec::new();

        let w0 = 4.0;
        for pt4 in points_4d {
            let pt4 = rot_mat * pt4;
            let (x, y, z, w) = (pt4[0], pt4[1], pt4[2], pt4[3]);
            let w_sc = 1.0 / (w0 + w).max(0.9);
            // let w_sc = 0.25 * (w_sc * w_sc) + 0.75 * w_sc;
            let pt = Vector3::new(x * w_sc, y * w_sc, z * w_sc);
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

        let mat_xyzw = nalgebra::base::Matrix4::<f64>::identity();

        // TODO(lucasw) build much larger sets of triangles, probably that will
        // greatly improve performance
        let mut all_points = Vec::new();
        let mut all_normals = Vec::new();
        let mut all_indices = Vec::new();
        for (ind, pts4d) in self.points_4d.iter().enumerate() {
            let (points_3d, normal) =
                Beams4D::points_to_3d(pts4d, &mat_xyzw);
            let num_pts = points_3d.len();
            all_points.extend(points_3d);
            all_normals.extend(vec![[normal[0], normal[1], normal[2]]; num_pts]);
            let i = (ind * 4) as u16;
            all_indices.extend(vec![i, i + 2, i + 1, i + 1, i + 2, i + 3]);
        }

        let mesh = Mesh::new(PrimitiveTopology::TriangleList)
            .with_inserted_attribute(Mesh::ATTRIBUTE_POSITION, all_points)
            .with_inserted_attribute(Mesh::ATTRIBUTE_NORMAL, all_normals)
            .with_indices(Some(Indices::U16(all_indices)));

        /*
        // println!("{ind} {indf} {color:?}");
        let indf = (ind / 24) as f32;
        let color = Color::rgb(
            (indf * 0.16) % 1.0,
            (1.0 - indf * 0.05) % 1.0,
            (indf * 0.07) % 1.0,
        );
        */
        let color = Color::rgb(0.65, 0.6, 0.5);
        meshes_colors.push((mesh, color));
        meshes_colors
    }

    fn update_mesh(
        &mut self,
        _time: Res<Time>,
        query0: Query<&Handle<Mesh>, With<AnimatedPosition>>,
        query1: Query<&bevy_test::CameraController, With<Camera>>,
        mut assets: ResMut<Assets<Mesh>>,
    ) {
        let rot_mat = {
            if let Ok(camera) = query1.get_single() {
                camera.rot_mat
            } else {
                nalgebra::base::Matrix4::<f64>::identity()
            }
        };

        // if self.i % 60 == 0 {
        //     println!("update {:.3}", time.elapsed_seconds());
        // }

        // println!(
        //     "xw: {:.2}, yw: {rot_yw:.2}, zw: {rot_zw:.2}",
        //     rot_xw / (std::f64::consts::PI * 2.0)
        // );

        // TODO(lucasw) the correlation is loose here, and brittle- I know I've only
        // created one mesh with AnimatedPosition that corresponds to all my points4d
        if let Ok(handle) = query0.get_single() {
            let mesh = assets.get_mut(handle.id());
            if let Some(mesh) = mesh {
                let mut all_points = Vec::new();
                let mut all_normals = Vec::new();
                for pts4d in &self.points_4d {
                    let (points_3d, normal) = Beams4D::points_to_3d(pts4d, &rot_mat);
                    let num_pts = points_3d.len();
                    all_points.extend(points_3d);
                    all_normals.extend(vec![[normal[0], normal[1], normal[2]]; num_pts]);
                }

                mesh.insert_attribute(Mesh::ATTRIBUTE_POSITION, all_points);
                mesh.insert_attribute(Mesh::ATTRIBUTE_NORMAL, all_normals);
            }
        }

        self.i += 1;
    }

    fn build_tesseract(&mut self, xoff: f64, yoff: f64, zoff: f64, woff: f64) {
        let sz = 0.92;
        let wd = 0.04;

        // build beams of a tesseract
        for yi in 0..2 {
            for zi in 0..2 {
                for wi in 0..2 {
                    let x0 = xoff;
                    let xsz = sz;
                    let y0 = yoff + yi as f64 * 2.0 - 1.0;
                    let ysz = wd;
                    let z0 = zoff + zi as f64 * 2.0 - 1.0;
                    let zsz = wd;
                    let w0 = woff + wi as f64 * 2.0 - 1.0;
                    let wsz = wd;
                    self.build_points((x0, xsz), (y0, ysz), (z0, zsz), (w0, wsz));
                }
            }
        }

        for xi in 0..2 {
            for zi in 0..2 {
                for wi in 0..2 {
                    let x0 = xoff + xi as f64 * 2.0 - 1.0;
                    let xsz = wd;
                    let y0 = yoff;
                    let ysz = sz;
                    let z0 = zoff + zi as f64 * 2.0 - 1.0;
                    let zsz = wd;
                    let w0 = woff + wi as f64 * 2.0 - 1.0;
                    let wsz = wd;
                    self.build_points((x0, xsz), (y0, ysz), (z0, zsz), (w0, wsz));
                }
            }
        }

        for xi in 0..2 {
            for yi in 0..2 {
                for wi in 0..2 {
                    let x0 = xoff + xi as f64 * 2.0 - 1.0;
                    let xsz = wd;
                    let y0 = yoff + yi as f64 * 2.0 - 1.0;
                    let ysz = wd;
                    let z0 = zoff;
                    let zsz = sz;
                    let w0 = woff + wi as f64 * 2.0 - 1.0;
                    let wsz = wd;
                    self.build_points((x0, xsz), (y0, ysz), (z0, zsz), (w0, wsz));
                }
            }
        }

        for xi in 0..2 {
            for yi in 0..2 {
                for zi in 0..2 {
                    let x0 = xoff + xi as f64 * 2.0 - 1.0;
                    let xsz = wd;
                    let y0 = yoff + yi as f64 * 2.0 - 1.0;
                    let ysz = wd;
                    let z0 = zoff + zi as f64 * 2.0 - 1.0;
                    let zsz = wd;
                    let w0 = woff;
                    let wsz = sz;
                    self.build_points((x0, xsz), (y0, ysz), (z0, zsz), (w0, wsz));
                }
            }
        }
    }
}

fn main() {
    let mut beams_4d = Beams4D {
        i: 0,
        points_4d: Vec::new(),
    };

    beams_4d.build_tesseract(0.0, 0.0, 0.0, 0.0);

    /*
    // TODO(lucasw) can't have too many or exceed u16, and they stop showing up?
    for xi in -1..2 {
        for yi in -1..2 {
            beams_4d.build_tesseract(xi as f64 * 3.0, yi as f64 * 3.0, 0.0, 0.0);
        }
    }
    */

    let raw_meshes = beams_4d.get_meshes();

    App::new()
        .add_plugins(DefaultPlugins)
        .insert_resource(ClearColor(Color::rgb(0.02, 0.0, 0.0)))
        .add_systems(
            Startup,
            move |commands: Commands,
                  meshes: ResMut<Assets<Mesh>>,
                  materials: ResMut<Assets<StandardMaterial>>| {
                // clone here avoids "this closure implements `FnOnce`, not `FnMut`" error
                setup(raw_meshes.clone(), commands, meshes, materials);
            },
        )
        .add_systems(
            Update,
            (
                move |tm: Res<Time>,
                      query0: Query<&Handle<Mesh>, With<AnimatedPosition>>,
                      query1: Query<&bevy_test::CameraController, With<Camera>>,
                      at: ResMut<Assets<Mesh>>| {
                    beams_4d.update_mesh(tm, query0, query1, at)
                },
                bevy_test::camera_controller,
            ),
        )
        .run();
}

#[derive(Component)]
struct CameraMarker;

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
        CameraMarker,
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

    // TODO(lucasw) there are a lot of meshes, could they be added together?
    for (mesh, color) in meshes_colors {
        let mesh = meshes.add(mesh);

        // TODO(lucasw) modify_vert needs to know about these mesh ids,
        // but not sure how to get them across
        // println!("id {:?}", mesh.id());
        let _ = commands
            .spawn((
                AnimatedPosition,
                PbrBundle {
                    mesh,
                    material: materials.add(StandardMaterial {
                        base_color: color,
                        diffuse_transmission: 0.6,
                        perceptual_roughness: 0.5,
                        reflectance: 0.2,
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
        transform: Transform::from_xyz(0.0, -4.0, 0.0),
        ..default()
    });
}
