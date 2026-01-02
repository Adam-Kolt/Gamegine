//! Realtime swerve drive simulation with field-oriented omnidirectional control.
//! 
//! Features:
//! - Motor-driven steering with realistic dynamics
//! - Field-oriented control (translation independent of robot heading)
//! - Battery voltage monitoring
//!
//! Controls:
//! - W/S: Forward/backward (world frame)
//! - A/D: Left/right (world frame)
//! - Q/E: Rotate left/right
//! - Arrow keys: Alternative translation control

use electrical::battery::{Battery, BatteryConstant};
use electrical::motor::{MotorBank, MotorConstant};
use mechanics::tire::{TireManager, TireConstants};
use simcore::{
    ElectricalModel, MechanicsModel, MotorInput, MotorState, SimContext, SimState,
    TireState, WheelState,
};

use eframe::egui;
use egui_plot::{Legend, Line, Plot, PlotBounds, PlotPoints};
use std::collections::VecDeque;
use std::f64::consts::PI;
use std::time::{Duration, Instant};

// --- Constants ---
const DT_OUTER: f64 = 1e-3;
const DT_ELEC: f64 = 1e-4;
const WHEEL_RADIUS: f64 = 0.05;
const MASS: f64 = 50.0;
const IZZ: f64 = 5.0;
const G: f64 = 9.81;
const C_RR: f64 = 0.01;
const RHO_AIR: f64 = 1.225;
const C_DA: f64 = 0.3;

const DRIVE_GEAR_RATIO: f64 = 6.75;
const STEER_GEAR_RATIO: f64 = 12.8; // SDS MK4 steer ratio
const DRIVE_EFFICIENCY: f64 = 0.92;
const STEER_EFFICIENCY: f64 = 0.85;
const WHEEL_INERTIA: f64 = 0.01;
const STEER_INERTIA: f64 = 0.002;

const WHEELBASE: f64 = 0.5;
const TRACK_WIDTH: f64 = 0.5;
const ROBOT_LENGTH: f64 = 0.6;
const ROBOT_WIDTH: f64 = 0.6;

const PLOT_DT: f64 = 1e-2;

fn main() -> eframe::Result<()> {
    let options = eframe::NativeOptions {
        viewport: egui::ViewportBuilder::default()
            .with_inner_size([1400.0, 1000.0])
            .with_title("Swerve Drive - Field Oriented Control"),
        ..Default::default()
    };
    eframe::run_native(
        "Swerve Drive - Field Oriented Control",
        options,
        Box::new(|_cc| Ok(Box::new(App::new()))),
    )
}

struct ModuleState {
    target_angle: f64,
    current_angle: f64,
    steer_velocity: f64,
    target_speed: f64,
}

struct Trace {
    t: VecDeque<f64>,
    v: VecDeque<f64>,
    yaw_rate: VecDeque<f64>,
    battery_v: VecDeque<f64>,
    px: VecDeque<f64>,
    py: VecDeque<f64>,
    capacity: usize,
}

impl Trace {
    fn new(seconds: f64, sample_dt: f64) -> Self {
        let capacity = (seconds / sample_dt).ceil() as usize + 1;
        Self {
            t: VecDeque::with_capacity(capacity),
            v: VecDeque::with_capacity(capacity),
            yaw_rate: VecDeque::with_capacity(capacity),
            battery_v: VecDeque::with_capacity(capacity),
            px: VecDeque::with_capacity(capacity),
            py: VecDeque::with_capacity(capacity),
            capacity,
        }
    }

    fn push(&mut self, t: f64, v: f64, yaw_rate: f64, battery_v: f64, px: f64, py: f64) {
        self.t.push_back(t);
        self.v.push_back(v);
        self.yaw_rate.push_back(yaw_rate);
        self.battery_v.push_back(battery_v);
        self.px.push_back(px);
        self.py.push_back(py);
        self.trim();
    }

    fn trim(&mut self) {
        while self.t.len() > self.capacity { self.t.pop_front(); }
        while self.v.len() > self.capacity { self.v.pop_front(); }
        while self.yaw_rate.len() > self.capacity { self.yaw_rate.pop_front(); }
        while self.battery_v.len() > self.capacity { self.battery_v.pop_front(); }
        while self.px.len() > self.capacity { self.px.pop_front(); }
        while self.py.len() > self.capacity { self.py.pop_front(); }
    }

    fn line<'a>(points: &'a VecDeque<f64>, t: &'a VecDeque<f64>) -> PlotPoints<'a> {
        PlotPoints::from_iter(t.iter().copied().zip(points.iter().copied()).map(|(x, y)| [x, y]))
    }
}

struct App {
    batt: Battery,
    drive_motors: MotorBank,
    steer_motors: MotorBank,
    drive_bus: SimState,
    steer_bus: SimState,
    tires: TireManager,

    t: f64,
    paused: bool,
    last_frame: Instant,
    sim_speed: f64,

    modules: [ModuleState; 4],

    // Field-oriented inputs (world frame, normalized -1 to 1)
    input_vx_world: f64,
    input_vy_world: f64,
    input_omega: f64,

    // Chassis state
    x: f64,
    y: f64,
    yaw: f64,
    vx_body: f64,
    vy_body: f64,
    yaw_rate: f64,

    module_pos: [(f64, f64); 4],

    // Viewport
    view_scale: f32,
    view_follow: bool,
    view_show_grid: bool,
    view_show_path: bool,
    window_s: f64,

    trace: Trace,
}

impl App {
    fn new() -> Self {
        // 4 drive motors
        let mut drive_motors = MotorBank::default();
        for _ in 0..4 {
            drive_motors.add_motor(MotorConstant::kraken_x60());
        }

        // 4 steer motors (smaller, like NEO 550 or Falcon 500 steer)
        let mut steer_motors = MotorBank::default();
        for _ in 0..4 {
            steer_motors.add_motor(MotorConstant::neo());
        }

        let batt = Battery { constants: BatteryConstant::default() };

        // Drive bus (for drive motors)
        let mut drive_bus = SimState::default();
        drive_bus.control_input.motor_inputs = vec![MotorInput { duty_cycle_q: 0.0, duty_cycle_d: 0.0 }; 4];
        drive_bus.true_state.motors = vec![MotorState::default(); 4];
        drive_bus.true_state.wheel_states = (0..4)
            .map(|_| WheelState {
                driving_angular_velocity: 0.0,
                wheel_radius: WHEEL_RADIUS,
                turning_angular_velocity: 0.0,
                longitudinal_translational_velocity: 0.0,
                lateral_translational_velocity: 0.0,
                tire: TireState {
                    slip_angle: 0.0,
                    slip_ratio: 0.0,
                    longitudinal_force: 0.0,
                    lateral_force: 0.0,
                    tire_load: MASS * G / 4.0,
                },
                angle: 0.0,
            })
            .collect();

        // Steer bus (for steer motors)
        let mut steer_bus = SimState::default();
        steer_bus.control_input.motor_inputs = vec![MotorInput { duty_cycle_q: 0.0, duty_cycle_d: 0.0 }; 4];
        steer_bus.true_state.motors = vec![MotorState::default(); 4];
        // steer_bus doesn't need wheel_states

        let mut tires = TireManager::new();
        tires.tire_constants.clear();
        for _ in 0..4 {
            tires.add_tire(TireConstants::new(1.5, 1.0, 3000.0, 3000.0, 0.0, 0.0));
        }

        let module_pos = [
            (WHEELBASE / 2.0, TRACK_WIDTH / 2.0),
            (WHEELBASE / 2.0, -TRACK_WIDTH / 2.0),
            (-WHEELBASE / 2.0, TRACK_WIDTH / 2.0),
            (-WHEELBASE / 2.0, -TRACK_WIDTH / 2.0),
        ];

        let modules = [
            ModuleState { target_angle: 0.0, current_angle: 0.0, steer_velocity: 0.0, target_speed: 0.0 },
            ModuleState { target_angle: 0.0, current_angle: 0.0, steer_velocity: 0.0, target_speed: 0.0 },
            ModuleState { target_angle: 0.0, current_angle: 0.0, steer_velocity: 0.0, target_speed: 0.0 },
            ModuleState { target_angle: 0.0, current_angle: 0.0, steer_velocity: 0.0, target_speed: 0.0 },
        ];

        Self {
            batt,
            drive_motors,
            steer_motors,
            drive_bus,
            steer_bus,
            tires,
            t: 0.0,
            paused: false,
            last_frame: Instant::now(),
            sim_speed: 1.0,
            modules,
            input_vx_world: 0.0,
            input_vy_world: 0.0,
            input_omega: 0.0,
            x: 0.0,
            y: 0.0,
            yaw: 0.0,
            vx_body: 0.0,
            vy_body: 0.0,
            yaw_rate: 0.0,
            module_pos,
            view_scale: 120.0,
            view_follow: true,
            view_show_grid: true,
            view_show_path: true,
            window_s: 10.0,
            trace: Trace::new(10.0, PLOT_DT),
        }
    }

    fn reset(&mut self) {
        *self = Self::new();
    }

    /// Compute swerve module states from chassis velocity command (body frame)
    fn compute_swerve_kinematics(&mut self, vx_body: f64, vy_body: f64, omega: f64) {
        for i in 0..4 {
            let (rx, ry) = self.module_pos[i];
            
            let module_vx = vx_body - omega * ry;
            let module_vy = vy_body + omega * rx;
            
            let speed = (module_vx.powi(2) + module_vy.powi(2)).sqrt();
            
            if speed < 0.01 {
                // Don't change angle when not moving
                self.modules[i].target_speed = 0.0;
                continue;
            }
            
            let mut angle = module_vy.atan2(module_vx);
            
            // Optimize: flip 180° if faster
            let angle_diff = (angle - self.modules[i].current_angle + PI).rem_euclid(2.0 * PI) - PI;
            if angle_diff.abs() > PI / 2.0 {
                angle = (angle + PI).rem_euclid(2.0 * PI) - PI;
                self.modules[i].target_speed = -speed / WHEEL_RADIUS;
            } else {
                self.modules[i].target_speed = speed / WHEEL_RADIUS;
            }
            self.modules[i].target_angle = angle;
        }
    }

    fn update_sim(&mut self, sim_dt: f64) {
        let steps = (sim_dt / DT_OUTER).ceil().max(1.0) as usize;
        let outer_dt = sim_dt / steps as f64;

        // Field-oriented control: transform world-frame input to body-frame
        let cos_yaw = self.yaw.cos();
        let sin_yaw = self.yaw.sin();
        let vx_body_cmd = self.input_vx_world * cos_yaw + self.input_vy_world * sin_yaw;
        let vy_body_cmd = -self.input_vx_world * sin_yaw + self.input_vy_world * cos_yaw;

        let max_speed = 4.0;
        let max_omega = 2.0 * PI;
        self.compute_swerve_kinematics(
            vx_body_cmd * max_speed,
            vy_body_cmd * max_speed,
            self.input_omega * max_omega,
        );

        for _ in 0..steps {
            // --- Steer motor control ---
            for i in 0..4 {
                let angle_error = self.modules[i].target_angle - self.modules[i].current_angle;
                let angle_error = (angle_error + PI).rem_euclid(2.0 * PI) - PI;
                
                // PD controller for steer (moderate gains)
                let kp = 5.0;
                let kd = 0.5;
                let steer_cmd = kp * angle_error - kd * self.modules[i].steer_velocity;
                let duty = steer_cmd.clamp(-1.0, 1.0);
                self.steer_bus.control_input.motor_inputs[i] = MotorInput { duty_cycle_q: duty, duty_cycle_d: 0.0 };
            }

            // --- Drive motor control ---
            for i in 0..4 {
                let omega = self.drive_bus.true_state.wheel_states[i].driving_angular_velocity;
                let target = self.modules[i].target_speed;
                let error = target - omega;
                let duty = (error * 0.1).clamp(-1.0, 1.0);
                self.drive_bus.control_input.motor_inputs[i] = MotorInput { duty_cycle_q: duty, duty_cycle_d: 0.0 };
            }

            // --- Electrical stepping ---
            let mut t_inner = 0.0;
            while t_inner < outer_dt {
                let dt = (outer_dt - t_inner).min(DT_ELEC);
                
                // Drive motors
                for i in 0..4 {
                    let omega_wheel = self.drive_bus.true_state.wheel_states[i].driving_angular_velocity;
                    self.drive_bus.true_state.motors[i].mechanical_velocity = omega_wheel * DRIVE_GEAR_RATIO;
                }
                self.drive_motors.step_electrical(SimContext { dt, t: self.t + t_inner }, &mut self.drive_bus);
                
                // Steer motors
                for i in 0..4 {
                    self.steer_bus.true_state.motors[i].mechanical_velocity = self.modules[i].steer_velocity * STEER_GEAR_RATIO;
                }
                self.steer_motors.step_electrical(SimContext { dt, t: self.t + t_inner }, &mut self.steer_bus);
                
                t_inner += dt;
            }

            // Calculate total current draw from drive motors only
            // (steer motors excluded for now to avoid unrealistic drain)
            let mut i_total = 0.0;
            for i in 0..4 {
                // Drive motor current
                let dm = &self.drive_bus.true_state.motors[i];
                let du = self.drive_bus.control_input.motor_inputs[i];
                i_total += dm.current_q * du.duty_cycle_q + dm.current_d * du.duty_cycle_d;
            }
            self.drive_bus.true_state.battery_state.total_current_draw = i_total;

            // Battery step
            self.batt.step_electrical(SimContext { dt: outer_dt, t: self.t }, &mut self.drive_bus);

            // --- Steer dynamics ---
            for i in 0..4 {
                let tq_motor = self.steer_bus.true_state.motors[i].applied_torque;
                let steer_torque = tq_motor * STEER_GEAR_RATIO * STEER_EFFICIENCY;
                // Simple friction/damping
                let friction = 0.1 * self.modules[i].steer_velocity.signum() + 0.5 * self.modules[i].steer_velocity;
                let net_torque = steer_torque - friction;
                let d_omega = net_torque / STEER_INERTIA;
                self.modules[i].steer_velocity += d_omega * outer_dt;
                self.modules[i].current_angle += self.modules[i].steer_velocity * outer_dt;
                // Wrap angle
                self.modules[i].current_angle = (self.modules[i].current_angle + PI).rem_euclid(2.0 * PI) - PI;
                self.drive_bus.true_state.wheel_states[i].angle = self.modules[i].current_angle;
            }

            // --- Tire kinematics ---
            for i in 0..4 {
                let (mx, my) = self.module_pos[i];
                let angle = self.modules[i].current_angle;
                
                let v_chassis_x = self.vx_body - self.yaw_rate * my;
                let v_chassis_y = self.vy_body + self.yaw_rate * mx;
                
                let cos_a = angle.cos();
                let sin_a = angle.sin();
                let v_long = v_chassis_x * cos_a + v_chassis_y * sin_a;
                let v_lat = -v_chassis_x * sin_a + v_chassis_y * cos_a;
                
                let wheel = &mut self.drive_bus.true_state.wheel_states[i];
                wheel.longitudinal_translational_velocity = v_long;
                wheel.lateral_translational_velocity = v_lat;
                wheel.wheel_radius = WHEEL_RADIUS;
                wheel.tire.tire_load = MASS * G / 4.0;
            }

            self.tires.step_physics(SimContext { dt: outer_dt, t: self.t }, &mut self.drive_bus);

            // --- Chassis dynamics ---
            let mut fx_total = 0.0;
            let mut fy_total = 0.0;
            let mut mz_total = 0.0;

            for i in 0..4 {
                let fx_tire = self.drive_bus.true_state.wheel_states[i].tire.longitudinal_force;
                let fy_tire = self.drive_bus.true_state.wheel_states[i].tire.lateral_force;
                let angle = self.modules[i].current_angle;
                let v_long = self.drive_bus.true_state.wheel_states[i].longitudinal_translational_velocity;
                
                let fx_wheel = -fx_tire;
                let fy_wheel = if v_long >= 0.0 { fy_tire } else { -fy_tire };
                
                let cos_a = angle.cos();
                let sin_a = angle.sin();
                let fx_body = fx_wheel * cos_a - fy_wheel * sin_a;
                let fy_body = fx_wheel * sin_a + fy_wheel * cos_a;
                
                fx_total += fx_body;
                fy_total += fy_body;
                
                let (mx, my) = self.module_pos[i];
                mz_total += mx * fy_body - my * fx_body;

                // Drive wheel dynamics
                let tq_motor = self.drive_bus.true_state.motors[i].applied_torque;
                let wheel_torque = tq_motor * DRIVE_GEAR_RATIO * DRIVE_EFFICIENCY;
                let omega = self.drive_bus.true_state.wheel_states[i].driving_angular_velocity;
                let tire_reaction = fx_tire * WHEEL_RADIUS;
                let net_torque = wheel_torque + tire_reaction;
                let domega = net_torque / WHEEL_INERTIA;
                self.drive_bus.true_state.wheel_states[i].driving_angular_velocity = omega + domega * outer_dt;
            }

            // Resistances
            let v_mag = (self.vx_body.powi(2) + self.vy_body.powi(2)).sqrt();
            let f_rr = C_RR * MASS * G;
            let f_drag = 0.5 * RHO_AIR * C_DA * v_mag * v_mag;
            let resistance = if v_mag > 0.001 { f_rr + f_drag } else { 0.0 };
            
            if v_mag > 0.001 {
                fx_total -= resistance * self.vx_body / v_mag;
                fy_total -= resistance * self.vy_body / v_mag;
            }

            mz_total -= 0.5 * self.yaw_rate;

            let ax = fx_total / MASS;
            let ay = fy_total / MASS;
            let alpha = mz_total / IZZ;

            self.vx_body += ax * outer_dt;
            self.vy_body += ay * outer_dt;
            self.yaw_rate += alpha * outer_dt;

            let cos_yaw = self.yaw.cos();
            let sin_yaw = self.yaw.sin();
            let vx_world = self.vx_body * cos_yaw - self.vy_body * sin_yaw;
            let vy_world = self.vx_body * sin_yaw + self.vy_body * cos_yaw;
            
            self.x += vx_world * outer_dt;
            self.y += vy_world * outer_dt;
            self.yaw += self.yaw_rate * outer_dt;

            self.t += outer_dt;

            if (self.trace.t.back().copied().unwrap_or(0.0) + PLOT_DT) <= self.t {
                let v_mag = (self.vx_body.powi(2) + self.vy_body.powi(2)).sqrt();
                let vbat = self.drive_bus.true_state.battery_state.voltage;
                self.trace.push(self.t, v_mag, self.yaw_rate, vbat, self.x, self.y);
            }
        }
    }

    fn handle_keyboard(&mut self, ctx: &egui::Context) {
        ctx.input(|i| {
            self.input_vx_world = 0.0;
            self.input_vy_world = 0.0;
            self.input_omega = 0.0;

            // World-frame translation (WASD = field oriented)
            if i.key_down(egui::Key::W) { self.input_vx_world = 1.0; }
            if i.key_down(egui::Key::S) { self.input_vx_world = -1.0; }
            if i.key_down(egui::Key::A) { self.input_vy_world = 1.0; }
            if i.key_down(egui::Key::D) { self.input_vy_world = -1.0; }
            
            if i.key_down(egui::Key::Q) { self.input_omega = 1.0; }
            if i.key_down(egui::Key::E) { self.input_omega = -1.0; }
            
            if i.key_down(egui::Key::ArrowUp) { self.input_vx_world = 1.0; }
            if i.key_down(egui::Key::ArrowDown) { self.input_vx_world = -1.0; }
            if i.key_down(egui::Key::ArrowLeft) { self.input_vy_world = 1.0; }
            if i.key_down(egui::Key::ArrowRight) { self.input_vy_world = -1.0; }
        });
    }

    fn draw_viewport(&self, ui: &mut egui::Ui, height_px: f32) {
        let desired = egui::vec2(ui.available_width(), height_px);
        let (response, painter) = ui.allocate_painter(desired, egui::Sense::drag());

        static mut VIEW_CENTER: (f64, f64) = (0.0, 0.0);
        if self.view_follow {
            unsafe { VIEW_CENTER = (self.x, self.y); }
        }
        let (cx, cy) = unsafe { VIEW_CENTER };

        let to_screen = |wx: f64, wy: f64| -> egui::Pos2 {
            let sx = ((wx - cx) as f32) * self.view_scale + response.rect.center().x;
            let sy = response.rect.center().y - ((wy - cy) as f32) * self.view_scale;
            egui::pos2(sx, sy)
        };

        painter.rect_filled(response.rect, 4.0, ui.visuals().extreme_bg_color);

        // Grid
        if self.view_show_grid {
            let bg = ui.visuals().weak_text_color();
            let step_m = 0.25_f32;
            let half_w = response.rect.width() / 2.0;
            let half_h = response.rect.height() / 2.0;
            let n_x = (half_w / (self.view_scale * step_m)).ceil() as i32 + 2;
            let n_y = (half_h / (self.view_scale * step_m)).ceil() as i32 + 2;
            for ix in -n_x..=n_x {
                let wx = cx + ix as f64 * step_m as f64;
                let p1 = to_screen(wx, cy - n_y as f64 * step_m as f64);
                let p2 = to_screen(wx, cy + n_y as f64 * step_m as f64);
                painter.line_segment([p1, p2], egui::Stroke::new(1.0, bg));
            }
            for iy in -n_y..=n_y {
                let wy = cy + iy as f64 * step_m as f64;
                let p1 = to_screen(cx - n_x as f64 * step_m as f64, wy);
                let p2 = to_screen(cx + n_x as f64 * step_m as f64, wy);
                painter.line_segment([p1, p2], egui::Stroke::new(1.0, bg));
            }
        }

        // World frame indicator (shows field orientation)
        let origin = to_screen(0.0, 0.0);
        painter.line_segment([origin, to_screen(0.5, 0.0)], egui::Stroke::new(3.0, egui::Color32::RED));
        painter.line_segment([origin, to_screen(0.0, 0.5)], egui::Stroke::new(3.0, egui::Color32::GREEN));
        painter.text(to_screen(0.55, 0.0), egui::Align2::LEFT_CENTER, "X", egui::FontId::monospace(12.0), egui::Color32::RED);
        painter.text(to_screen(0.0, 0.55), egui::Align2::CENTER_BOTTOM, "Y", egui::FontId::monospace(12.0), egui::Color32::GREEN);

        // Path trace
        if self.view_show_path && self.trace.px.len() > 1 {
            let points: Vec<egui::Pos2> = self.trace.px.iter().copied()
                .zip(self.trace.py.iter().copied())
                .map(|(x, y)| to_screen(x, y))
                .collect();
            painter.add(egui::Shape::line(points, egui::Stroke::new(2.0, egui::Color32::LIGHT_BLUE)));
        }

        // Robot body
        let hl = ROBOT_LENGTH * 0.5;
        let hw = ROBOT_WIDTH * 0.5;
        let (c, s) = (self.yaw.cos(), self.yaw.sin());
        let body = [
            [ hl,  hw],
            [ hl, -hw],
            [-hl, -hw],
            [-hl,  hw],
        ];
        let poly: Vec<egui::Pos2> = body.into_iter()
            .map(|[bx, by]| {
                let wx = self.x + c * bx - s * by;
                let wy = self.y + s * bx + c * by;
                to_screen(wx, wy)
            })
            .collect();
        painter.add(egui::Shape::convex_polygon(
            poly.clone(),
            egui::Color32::from_rgba_unmultiplied(100, 100, 200, 120),
            egui::Stroke::new(2.0, egui::Color32::YELLOW),
        ));

        // Modules
        for i in 0..4 {
            let (mx, my) = self.module_pos[i];
            let angle = self.modules[i].current_angle;
            
            let wx = self.x + c * mx - s * my;
            let wy = self.y + s * mx + c * my;
            let center = to_screen(wx, wy);
            
            let wheel_angle = self.yaw + angle;
            let wheel_len = 0.08 * self.view_scale;
            let dx = wheel_angle.cos() as f32 * wheel_len;
            let dy = -wheel_angle.sin() as f32 * wheel_len;
            
            painter.line_segment(
                [egui::pos2(center.x - dx, center.y - dy), egui::pos2(center.x + dx, center.y + dy)],
                egui::Stroke::new(4.0, egui::Color32::GREEN),
            );
            painter.circle_filled(center, 4.0, egui::Color32::WHITE);
        }

        // Forward arrow
        let nose = to_screen(
            self.x + c * hl * 1.2,
            self.y + s * hl * 1.2,
        );
        let tail = to_screen(self.x, self.y);
        painter.line_segment([tail, nose], egui::Stroke::new(3.0, egui::Color32::from_rgb(255, 100, 100)));
    }
}

impl eframe::App for App {
    fn update(&mut self, ctx: &egui::Context, _frame: &mut eframe::Frame) {
        self.handle_keyboard(ctx);

        if !self.paused {
            let now = Instant::now();
            let wall_dt = now.duration_since(self.last_frame).as_secs_f64();
            self.last_frame = now;
            let sim_dt = (wall_dt * self.sim_speed).min(0.05);
            self.update_sim(sim_dt);
        } else {
            self.last_frame = Instant::now();
        }

        let bat_v = self.drive_bus.true_state.battery_state.voltage;

        egui::TopBottomPanel::top("controls").show(ctx, |ui| {
            ui.horizontal_wrapped(|ui| {
                if ui.button(if self.paused { "▶ Resume" } else { "⏸ Pause" }).clicked() {
                    self.paused = !self.paused;
                }
                if ui.button("⟲ Reset").clicked() { self.reset(); }

                ui.separator();
                ui.label("Sim speed");
                ui.add(egui::Slider::new(&mut self.sim_speed, 0.1..=4.0).logarithmic(true).suffix("×"));

                ui.separator();
                ui.label(format!("Pose: x={:.2} y={:.2} θ={:.1}°", self.x, self.y, self.yaw.to_degrees()));
                ui.label(format!("v={:.2} m/s  ω={:.2} rad/s", (self.vx_body.powi(2) + self.vy_body.powi(2)).sqrt(), self.yaw_rate));
                
                ui.separator();
                ui.colored_label(
                    if bat_v > 11.0 { egui::Color32::GREEN } else { egui::Color32::RED },
                    format!("Battery: {:.1}V", bat_v)
                );
            });
            ui.horizontal(|ui| {
                ui.label("🎮 Field-Oriented Control: W/S=forward/back (world), A/D=left/right (world), Q/E=rotate");
            });
        });

        egui::CentralPanel::default().show(ctx, |ui| {
            ui.horizontal(|ui| {
                ui.checkbox(&mut self.view_follow, "Follow");
                ui.checkbox(&mut self.view_show_grid, "Grid");
                ui.checkbox(&mut self.view_show_path, "Path");
                ui.add(egui::Slider::new(&mut self.view_scale, 40.0..=300.0).prefix("Zoom: ").suffix(" px/m"));
            });
            self.draw_viewport(ui, 350.0);

            ui.separator();
            
            ui.columns(2, |columns| {
                columns[0].heading("Velocity & Yaw Rate");
                Plot::new("dynamics_plot")
                    .legend(Legend::default())
                    .allow_scroll(false)
                    .height(150.0)
                    .show(&mut columns[0], |plot_ui| {
                        let x_min = (self.t - self.window_s).max(0.0);
                        let x_max = self.t.max(self.window_s * 0.1);
                        plot_ui.set_plot_bounds(PlotBounds::from_min_max([x_min, -5.0], [x_max, 5.0]));
                        plot_ui.line(Line::new("v (m/s)", Trace::line(&self.trace.v, &self.trace.t)));
                        plot_ui.line(Line::new("ω (rad/s)", Trace::line(&self.trace.yaw_rate, &self.trace.t)));
                    });
                
                columns[1].heading("Battery Voltage");
                Plot::new("battery_plot")
                    .legend(Legend::default())
                    .allow_scroll(false)
                    .height(150.0)
                    .show(&mut columns[1], |plot_ui| {
                        let x_min = (self.t - self.window_s).max(0.0);
                        let x_max = self.t.max(self.window_s * 0.1);
                        plot_ui.set_plot_bounds(PlotBounds::from_min_max([x_min, 8.0], [x_max, 14.0]));
                        plot_ui.line(Line::new("V_bat", Trace::line(&self.trace.battery_v, &self.trace.t)));
                    });
            });
        });

        ctx.request_repaint_after(Duration::from_millis(10));
    }
}
