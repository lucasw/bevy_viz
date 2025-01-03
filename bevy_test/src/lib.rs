use bevy::{input::mouse::MouseMotion, prelude::*};
use std::f32::consts::PI;

#[rustfmt::skip]
fn get_rotation_matrix_4d(
    rot_xy: f64, rot_xz: f64, rot_yz: f64,
    rot_xw: f64, rot_yw: f64, rot_zw: f64,
) -> nalgebra::base::Matrix4<f64> {
    // normal 3d rotations
    let angle = rot_xy;
    let mat_xy = nalgebra::base::Matrix4::new(
        angle.cos(), angle.sin(), 0., 0.,
        -angle.sin(), angle.cos(), 0., 0.,
        0., 0., 1.0, 0.,
        0., 0., 0., 1.0,
    );

    let angle = rot_xz;
    let mat_xz = nalgebra::base::Matrix4::new(
        angle.cos(), 0., angle.sin(), 0.,
        0., 1., 0., 0.,
        -angle.sin(), 0., angle.cos(), 0.,
        0., 0., 0., 1.0,
    );

    let angle = rot_yz;
    let mat_yz = nalgebra::base::Matrix4::new(
        1., 0., 0., 0.,
        0., angle.cos(), angle.sin(), 0.,
        0., -angle.sin(), angle.cos(), 0.,
        0., 0., 0., 1.0,
    );

    // 4th dimension rotations
    let angle = rot_xw;
    let mat_xw = nalgebra::base::Matrix4::new(
        angle.cos(), 0., 0., angle.sin(),
        0., 1.0, 0., 0.,
        0., 0., 1.0, 0.,
        -angle.sin(), 0., 0., angle.cos(),
    );

    let angle = rot_yw;
    let mat_yw = nalgebra::base::Matrix4::new(
        1.0, 0., 0., 0.,
        0., angle.cos(),  0., angle.sin(),
        0., 0., 1.0, 0.,
        0., -angle.sin(), 0., angle.cos(),
    );

    let angle = rot_zw;
    let mat_zw = nalgebra::base::Matrix4::new(
        1.0, 0., 0., 0.,
        0., 1.0, 0., 0.,
        0., 0., angle.cos(), angle.sin(),
        0., 0., -angle.sin(), angle.cos(),
    );

    mat_xy * mat_xz * mat_yz * mat_xw * mat_yw * mat_zw
}

#[derive(Component)]
pub struct CameraController {
    pub enabled: bool,
    pub initialized: bool,
    pub sensitivity: f32,
    pub key_forward: KeyCode,
    pub key_back: KeyCode,
    pub key_left: KeyCode,
    pub key_right: KeyCode,
    pub key_up: KeyCode,
    pub key_down: KeyCode,
    pub key_run: KeyCode,
    pub mouse_key_enable_mouse: MouseButton,
    pub keyboard_key_enable_mouse: KeyCode,
    pub walk_speed: f32,
    pub run_speed: f32,
    pub friction: f32,
    pub pitch: f32,
    pub yaw: f32,
    pub rot_mat: nalgebra::base::Matrix4<f64>,
    pub velocity: Vec3,
}

impl Default for CameraController {
    fn default() -> Self {
        Self {
            enabled: true,
            initialized: false,
            sensitivity: 0.25,
            key_forward: KeyCode::KeyW,
            key_back: KeyCode::KeyS,
            key_left: KeyCode::KeyA,
            key_right: KeyCode::KeyD,
            key_up: KeyCode::KeyE,
            key_down: KeyCode::KeyQ,
            key_run: KeyCode::ShiftLeft,
            mouse_key_enable_mouse: MouseButton::Left,
            keyboard_key_enable_mouse: KeyCode::KeyM,
            walk_speed: 2.0,
            run_speed: 6.0,
            friction: 0.5,
            pitch: 0.0,
            yaw: 0.0,
            rot_mat: nalgebra::base::Matrix4::<f64>::identity(),
            velocity: Vec3::ZERO,
        }
    }
}

pub fn camera_controller(
    time: Res<Time>,
    mut mouse_events: EventReader<MouseMotion>,
    mouse_button_input: Res<ButtonInput<MouseButton>>,
    key_input: Res<ButtonInput<KeyCode>>,
    mut move_toggled: Local<bool>,
    mut query: Query<(&mut Transform, &mut CameraController), With<Camera>>,
) {
    let dt = time.delta_seconds();

    if let Ok((mut transform, mut options)) = query.get_single_mut() {
        if !options.initialized {
            let (yaw, pitch, _roll) = transform.rotation.to_euler(EulerRot::YXZ);
            options.yaw = yaw;
            options.pitch = pitch;
            options.initialized = true;
        }
        if !options.enabled {
            return;
        }

        // Handle key input
        let mut axis_input = Vec3::ZERO;
        if key_input.pressed(options.key_forward) {
            axis_input.z += 1.0;
        }
        if key_input.pressed(options.key_back) {
            axis_input.z -= 1.0;
        }
        if key_input.pressed(options.key_right) {
            axis_input.x += 1.0;
        }
        if key_input.pressed(options.key_left) {
            axis_input.x -= 1.0;
        }
        if key_input.pressed(options.key_up) {
            axis_input.y += 1.0;
        }
        if key_input.pressed(options.key_down) {
            axis_input.y -= 1.0;
        }
        if key_input.just_pressed(options.keyboard_key_enable_mouse) {
            *move_toggled = !*move_toggled;
        }

        // This is more than a camera controller, it controls the tesseract
        {
            let mut rot_xy = 0.0;
            let mut rot_xz = 0.0;
            let mut rot_yz = 0.0;
            let mut rot_xw = 0.0;
            let mut rot_yw = 0.0;
            let mut rot_zw = 0.0;

            if key_input.pressed(KeyCode::KeyR) {
                rot_xy += 0.01;
            }
            if key_input.pressed(KeyCode::KeyF) {
                rot_xy -= 0.009;
            }

            if key_input.pressed(KeyCode::KeyT) {
                rot_xz += 0.01;
            }
            if key_input.pressed(KeyCode::KeyG) {
                rot_xz -= 0.009;
            }

            if key_input.pressed(KeyCode::KeyY) {
                rot_yz += 0.01;
            }
            if key_input.pressed(KeyCode::KeyH) {
                rot_yz -= 0.009;
            }

            if key_input.pressed(KeyCode::KeyU) {
                rot_xw += 0.01;
            }
            if key_input.pressed(KeyCode::KeyJ) {
                rot_xw -= 0.009;
            }

            if key_input.pressed(KeyCode::KeyI) {
                rot_yw += 0.01;
            }
            if key_input.pressed(KeyCode::KeyK) {
                rot_yw -= 0.009;
            }

            if key_input.pressed(KeyCode::KeyO) {
                rot_zw += 0.01;
            }
            if key_input.pressed(KeyCode::KeyL) {
                rot_zw -= 0.009;
            }

            let rot_mat = get_rotation_matrix_4d(rot_xy, rot_xz, rot_yz, rot_xw, rot_yw, rot_zw);
            options.rot_mat = rot_mat * options.rot_mat;
            options.rot_mat.normalize_mut();
        }

        // Apply movement update
        if axis_input != Vec3::ZERO {
            let max_speed = if key_input.pressed(options.key_run) {
                options.run_speed
            } else {
                options.walk_speed
            };
            options.velocity = axis_input.normalize() * max_speed;
        } else {
            let friction = options.friction.clamp(0.0, 1.0);
            options.velocity *= 1.0 - friction;
            if options.velocity.length_squared() < 1e-6 {
                options.velocity = Vec3::ZERO;
            }
        }
        let forward = transform.forward();
        let right = transform.right();
        transform.translation += options.velocity.x * dt * right
            + options.velocity.y * dt * Vec3::Y
            + options.velocity.z * dt * forward;

        // Handle mouse input
        let mut mouse_delta = Vec2::ZERO;
        for mouse_event in mouse_events.read() {
            mouse_delta += mouse_event.delta;
        }

        let is_mouse_move =
            mouse_button_input.pressed(options.mouse_key_enable_mouse) || *move_toggled;
        if is_mouse_move {
            // Apply look update
            options.pitch = (options.pitch - mouse_delta.y * 0.5 * options.sensitivity * dt)
                .clamp(-PI / 2., PI / 2.);
            options.yaw -= mouse_delta.x * options.sensitivity * dt;
            transform.rotation = Quat::from_euler(EulerRot::ZYX, 0.0, options.yaw, options.pitch);
        }
        // println!("{transform:?}");
    }
}
