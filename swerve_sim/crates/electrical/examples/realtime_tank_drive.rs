use electrical::battery::{Battery, BatteryConstant};
use electrical::motor::{MotorBank, MotorConstant};
use simcore::{ElectricalModel, MotorInput, MotorState, SimContext, SimState, WheelState, TireState, Model, MechanicsModel};
use mechanics::tire::{TireManager, TireConstants};

use egui_plot::{AxisHints, Legend, Line, Plot, PlotBounds, PlotPoints};
use std::collections::VecDeque;
use std::time::{Duration, Instant};

// Simulation base timesteps
const DT_OUTER: f64 = 1e-3; // outer loop step (mechanics + battery)
const DT_ELEC: f64 = 1e-4;  // electrical inner loop

// Vehicle parameters (simple tank model)
const WHEEL_RADIUS: f64 = 0.0508; // 3 inch wheel (m)
const TRACK_WIDTH: f64 = 0.6; // distance between left/right wheel centers (m)
const MASS: f64 = 50.0; // robot mass (kg)
const IZZ: f64 = 4.0; // yaw inertia (kg*m^2) — rough guess
const MU: f64 = 1.5; // friction coefficient
const G: f64 = 9.81; // gravity (m/s^2)
const C_RR: f64 = 0.015; // rolling resistance coefficient
const RHO_AIR: f64 = 1.225; // kg/m^3
const C_DA: f64 = 0.6 * 0.5; // lumped drag area (Cd*A) m^2

// Drivetrain gearing (motor -> wheel)
// Convention: wheel_omega = motor_omega / GEAR_RATIO
// Wheel torque = motor_torque * GEAR_RATIO * DRIVE_EFFICIENCY
const GEAR_RATIO: f64 = 5.0;
const DRIVE_EFFICIENCY: f64 = 0.92;
const WHEEL_INERTIA: f64 = 0.02; // kg*m^2, equivalent inertia per wheel (tunable)
const WHEEL_VISC_DAMP: f64 = 0.1; // N*m*s/rad, wheel rotational damping
const YAW_DAMPING: f64 = 0.5; // N*m*s yaw viscous damping

// Robot footprint for 2D viewport (meters)
const ROBOT_LENGTH: f64 = 0.8;
const ROBOT_WIDTH: f64 = 0.6;

// Plot window and scaling
const PLOT_DT: f64 = 1e-2; // downsampled plotting interval

// Fixed y-bounds for plots
const Y_LEFT_MIN: f64 = -400.0;
const Y_LEFT_MAX: f64 = 400.0;
const Y_RIGHT_MIN: f64 = -10.0;
const Y_RIGHT_MAX: f64 = 10.0;

// Display scaling for multi-axis overlay
const SOC_SCALE: f64 = 350.0;
const VOLT_SCALE: f64 = 400.0 / 14.0;
const VEL_SCALE: f64 = 2.0; // v (m/s) scaled to fit right plot
const YAW_SCALE: f64 = 2.0; // yaw rate (rad/s) scaled to fit

fn main() -> eframe::Result<()> {
    let options = eframe::NativeOptions {
        viewport: egui::ViewportBuilder::default()
            .with_inner_size([1280.0, 820.0])
            .with_title("Realtime Tank-Drive (4 motors)"),
        ..Default::default()
    };
    eframe::run_native(
        "Realtime Tank-Drive (4 motors)",
        options,
        Box::new(|_cc| Ok(Box::new(App::new()))),
    )
}

struct Trace {
    t: VecDeque<f64>,
    batt_v: VecDeque<f64>,
    soc: VecDeque<f64>,
    i_total: VecDeque<f64>,
    i_q_sum: VecDeque<f64>,
    i_d_sum: VecDeque<f64>,
    v: VecDeque<f64>,
    yaw_rate: VecDeque<f64>,
    left_omega: VecDeque<f64>,
    right_omega: VecDeque<f64>,
    px: VecDeque<f64>,
    py: VecDeque<f64>,
    // Tire forces per wheel
    fx_fl: VecDeque<f64>,
    fx_rl: VecDeque<f64>,
    fx_fr: VecDeque<f64>,
    fx_rr: VecDeque<f64>,
    fy_fl: VecDeque<f64>,
    fy_rl: VecDeque<f64>,
    fy_fr: VecDeque<f64>,
    fy_rr: VecDeque<f64>,
    capacity: usize,
}

impl Trace {
    fn new(seconds: f64, sample_dt: f64) -> Self {
        let capacity = (seconds / sample_dt).ceil() as usize + 1;
        Self {
            t: VecDeque::with_capacity(capacity),
            batt_v: VecDeque::with_capacity(capacity),
            soc: VecDeque::with_capacity(capacity),
            i_total: VecDeque::with_capacity(capacity),
            i_q_sum: VecDeque::with_capacity(capacity),
            i_d_sum: VecDeque::with_capacity(capacity),
            v: VecDeque::with_capacity(capacity),
            yaw_rate: VecDeque::with_capacity(capacity),
            left_omega: VecDeque::with_capacity(capacity),
            right_omega: VecDeque::with_capacity(capacity),
            px: VecDeque::with_capacity(capacity),
            py: VecDeque::with_capacity(capacity),
            fx_fl: VecDeque::with_capacity(capacity),
            fx_rl: VecDeque::with_capacity(capacity),
            fx_fr: VecDeque::with_capacity(capacity),
            fx_rr: VecDeque::with_capacity(capacity),
            fy_fl: VecDeque::with_capacity(capacity),
            fy_rl: VecDeque::with_capacity(capacity),
            fy_fr: VecDeque::with_capacity(capacity),
            fy_rr: VecDeque::with_capacity(capacity),
            capacity,
        }
    }

    fn set_window_seconds(&mut self, seconds: f64, sample_dt: f64) {
        self.capacity = (seconds / sample_dt).ceil() as usize + 1;
        self.trim_to_capacity();
    }

    fn push(
        &mut self,
        t: f64,
        batt_v: f64,
        soc: f64,
        i_total: f64,
        i_q_sum: f64,
        i_d_sum: f64,
        v: f64,
        yaw_rate: f64,
        left_omega: f64,
        right_omega: f64,
        px: f64,
        py: f64,
        fx: [f64; 4],
        fy: [f64; 4],
    ) {
        self.t.push_back(t);
        self.batt_v.push_back(batt_v);
        self.soc.push_back(soc);
        self.i_total.push_back(i_total);
        self.i_q_sum.push_back(i_q_sum);
        self.i_d_sum.push_back(i_d_sum);
        self.v.push_back(v);
        self.yaw_rate.push_back(yaw_rate);
        self.left_omega.push_back(left_omega);
        self.right_omega.push_back(right_omega);
        self.px.push_back(px);
        self.py.push_back(py);
        self.fx_fl.push_back(fx[0]);
        self.fx_rl.push_back(fx[1]);
        self.fx_fr.push_back(fx[2]);
        self.fx_rr.push_back(fx[3]);
        self.fy_fl.push_back(fy[0]);
        self.fy_rl.push_back(fy[1]);
        self.fy_fr.push_back(fy[2]);
        self.fy_rr.push_back(fy[3]);
        self.trim_to_capacity();
    }

    fn trim_to_capacity(&mut self) {
        let mut trim = |v: &mut VecDeque<f64>| while v.len() > self.capacity { v.pop_front(); };
        trim(&mut self.t);
        trim(&mut self.batt_v);
        trim(&mut self.soc);
        trim(&mut self.i_total);
        trim(&mut self.i_q_sum);
        trim(&mut self.i_d_sum);
        trim(&mut self.v);
        trim(&mut self.yaw_rate);
        trim(&mut self.left_omega);
        trim(&mut self.right_omega);
        trim(&mut self.px);
        trim(&mut self.py);
        trim(&mut self.fx_fl);
        trim(&mut self.fx_rl);
        trim(&mut self.fx_fr);
        trim(&mut self.fx_rr);
        trim(&mut self.fy_fl);
        trim(&mut self.fy_rl);
        trim(&mut self.fy_fr);
        trim(&mut self.fy_rr);
    }

    fn line<'a>(points: &'a VecDeque<f64>, t: &'a VecDeque<f64>) -> PlotPoints<'a> {
        PlotPoints::from_iter(t.iter().copied().zip(points.iter().copied()).map(|(x, y)| [x, y]))
    }

    fn line_scaled<'a>(points: &'a VecDeque<f64>, t: &'a VecDeque<f64>, scale: f64) -> PlotPoints<'a> {
        PlotPoints::from_iter(
            t.iter()
                .copied()
                .zip(points.iter().copied().map(|y| y * scale))
                .map(|(x, y)| [x, y]),
        )
    }

    fn line_xy<'a>(xs: &'a VecDeque<f64>, ys: &'a VecDeque<f64>) -> PlotPoints<'a> {
        PlotPoints::from_iter(xs.iter().copied().zip(ys.iter().copied()).map(|(x, y)| [x, y]))
    }
}

struct App {
    // models
    batt: Battery,
    motors: MotorBank,
    bus: SimState,
    tires: TireManager,

    // simulation time
    t: f64,
    paused: bool,
    last_frame: Instant,
    sim_speed: f64,

    // controls
    left_throttle: f64,  // -1..1
    right_throttle: f64, // -1..1
    window_s: f64,

    // viewport
    view_scale: f32,   // pixels per meter
    view_follow: bool,
    view_show_grid: bool,
    view_show_path: bool,

    // state (mechanics)
    x: f64,
    y: f64,
    yaw: f64,
    v: f64,        // forward velocity (body x)
    yaw_rate: f64, // rad/s

    // wheel layout (body frame positions x,y for 4 wheels: FL, RL, FR, RR)
    wheel_pos: [(f64, f64); 4],

    // trace
    trace: Trace,
}

impl App {
    fn new() -> Self {
        let batt = Battery { constants: BatteryConstant::default() };
        let mut motors = MotorBank::default();
        // 4 identical motors
        let mconst = MotorConstant::kraken_x60();
        for _ in 0..4 { motors.add_motor(mconst); }

        let mut bus = SimState::default();
        bus.control_input.motor_inputs = vec![MotorInput { duty_cycle_d: 0.0, duty_cycle_q: 0.0 }; 4];
        bus.true_state.motors = vec![MotorState::default(); 4];
        // Initialize 4 wheel states
        bus.true_state.wheel_states = (0..4).map(|_| WheelState {
            driving_angular_velocity: 0.0,
            wheel_radius: WHEEL_RADIUS,
            turning_angular_velocity: 0.0,
            longitudinal_translational_velocity: 0.0,
            lateral_translational_velocity: 0.0,
            tire: TireState { slip_angle: 0.0, slip_ratio: 0.0, longitudinal_force: 0.0, lateral_force: 0.0, tire_load: MASS * G / 4.0 },
            angle: 0.0,
        }).collect();

        // Tire manager with 4 identical tires
        let mut tires = TireManager::new();
        // Use moderately stiff tire parameters (tunable)
        let tire_template = TireConstants::new(
            1.5,  // longitudinal mu
            1.0,  // lateral mu
            3000.0, // cornering stiffness (N/rad)
            3000.0, // longitudinal stiffness (N)
            0.0, // longitudinal relaxation length (m)
            0.0, // lateral relaxation length (m)
        );
        // first default exists; add 3 more to make 4 total
        tires.tire_constants.clear();
        for _ in 0..4 { tires.add_tire(TireConstants::new(
            tire_template.longitudinal_coefficient_of_friction,
            tire_template.lateral_coefficient_of_friction,
            tire_template.cornering_stiffness,
            tire_template.longitudinal_stiffness,
            tire_template.longitudinal_relaxation_length,
            tire_template.lateral_relaxation_length,
        )); }

        let mut app = Self {
            batt,
            motors,
            bus,
            tires,
            t: 0.0,
            paused: false,
            last_frame: Instant::now(),
            sim_speed: 0.01,
            left_throttle: 0.0,
            right_throttle: 0.0,
            window_s: 10.0,
            view_scale: 120.0,
            view_follow: true,
            view_show_grid: true,
            view_show_path: true,
            x: 0.0,
            y: 0.0,
            yaw: 0.0,
            v: 0.0,
            yaw_rate: 0.0,
            // wheel positions: front/rear at +/- L/2, left/right at +/- W/2 (left negative y)
            wheel_pos: [
                (ROBOT_LENGTH * 0.5, -TRACK_WIDTH * 0.5), // FL -> motor 0
                (-ROBOT_LENGTH * 0.5, -TRACK_WIDTH * 0.5), // RL -> motor 1
                (ROBOT_LENGTH * 0.5, TRACK_WIDTH * 0.5), // FR -> motor 2
                (-ROBOT_LENGTH * 0.5, TRACK_WIDTH * 0.5), // RR -> motor 3
            ],
            trace: Trace::new(10.0, PLOT_DT),
        };
        app.sample();
        app
    }

    fn reset(&mut self) {
        self.batt = Battery { constants: BatteryConstant::default() };
        self.motors = {
            let mut m = MotorBank::default();
            let mc = MotorConstant::kraken_x60();
            for _ in 0..4 { m.add_motor(mc); }
            m
        };
        self.bus = SimState::default();
        self.bus.control_input.motor_inputs = vec![MotorInput { duty_cycle_d: 0.0, duty_cycle_q: 0.0 }; 4];
        self.bus.true_state.motors = vec![MotorState::default(); 4];
        self.bus.true_state.wheel_states = (0..4).map(|_| WheelState {
            driving_angular_velocity: 0.0,
            wheel_radius: WHEEL_RADIUS,
            turning_angular_velocity: 0.0,
            longitudinal_translational_velocity: 0.0,
            lateral_translational_velocity: 0.0,
            tire: TireState { slip_angle: 0.0, slip_ratio: 0.0, longitudinal_force: 0.0, lateral_force: 0.0, tire_load: MASS * G / 4.0 },
            angle: 0.0,
        }).collect();
        // reset tire manager
        self.tires.reset();
        self.tires.tire_constants.clear();
        for _ in 0..4 { self.tires.add_tire(TireConstants::new(1.0, 1.0, 3000.0, 3000.0, 0.02, 0.02)); }
        self.t = 0.0;
        self.x = 0.0;
        self.y = 0.0;
        self.yaw = 0.0;
        self.v = 0.0;
        self.yaw_rate = 0.0;
        self.trace = Trace::new(self.window_s, PLOT_DT);
    }

    fn update_sim(&mut self, sim_dt: f64) {
        let steps = (sim_dt / DT_OUTER).ceil().max(1.0) as usize;
        let outer_dt = sim_dt / steps as f64;

        for _ in 0..steps {
            // Map throttles to motor inputs (q-duty). Left motors: 0,1; Right motors: 2,3
            let l = self.left_throttle.clamp(-1.0, 1.0);
            let r = self.right_throttle.clamp(-1.0, 1.0);
            self.bus.control_input.motor_inputs[0] = MotorInput { duty_cycle_q: l, duty_cycle_d: 0.0 };
            self.bus.control_input.motor_inputs[1] = MotorInput { duty_cycle_q: l, duty_cycle_d: 0.0 };
            self.bus.control_input.motor_inputs[2] = MotorInput { duty_cycle_q: r, duty_cycle_d: 0.0 };
            self.bus.control_input.motor_inputs[3] = MotorInput { duty_cycle_q: r, duty_cycle_d: 0.0 };

            // Inner electrical steps
            let mut t_inner = 0.0;
            while t_inner < outer_dt {
                let dt = (outer_dt - t_inner).min(DT_ELEC);

                // Provide current mechanical velocities to the motor model from wheel rotational state
                // Map wheels: 0=FL,1=RL,2=FR,3=RR; motors index the same
                for i in 0..4 {
                    let omega_wheel = self.bus.true_state.wheel_states[i].driving_angular_velocity;
                    self.bus.true_state.motors[i].mechanical_velocity = omega_wheel * GEAR_RATIO;
                }

                // Step electrical model for all 4 motors
                self.motors
                    .step_electrical(SimContext { dt, t: self.t + t_inner }, &mut self.bus);

                t_inner += dt;
            }

            // Aggregate battery current draw from motors (q,d currents projected by duty)
            let mut i_total = 0.0;
            let mut i_q_sum = 0.0;
            let mut i_d_sum = 0.0;
            for i in 0..4 {
                let m = &self.bus.true_state.motors[i];
                let u = self.bus.control_input.motor_inputs[i];
                i_total += m.current_q * u.duty_cycle_q + m.current_d * u.duty_cycle_d;
                i_q_sum += m.current_q;
                i_d_sum += m.current_d;
            }
            self.bus.true_state.battery_state.total_current_draw = i_total;

            // Step battery
            self.batt
                .step_electrical(SimContext { dt: outer_dt, t: self.t }, &mut self.bus);

            // Update wheel kinematics for tire model
            for i in 0..4 {
                let (wx, wy) = self.wheel_pos[i];
                // Velocity at wheel contact point in body frame
                let v_point_x = self.v - self.yaw_rate * wy;
                let v_point_y = self.yaw_rate * wx;
                let wheel = &mut self.bus.true_state.wheel_states[i];
                wheel.longitudinal_translational_velocity = v_point_x;
                wheel.lateral_translational_velocity = v_point_y;
                wheel.wheel_radius = WHEEL_RADIUS;
                wheel.angle = 0.0; // tank drive wheels aligned with body x
                wheel.turning_angular_velocity = 0.0;
                wheel.tire.tire_load = MASS * G / 4.0;
            }

            // Step tire model (lateral/longitudinal forces with relaxation + ellipse)
            self.tires
                .step_physics(SimContext { dt: outer_dt, t: self.t }, &mut self.bus);

            // Use tire forces for chassis; use motor torque only to spin the wheels against tire friction
            let mut f_long_total = 0.0;
            let mut m_z = 0.0;
            for i in 0..4 {
                // Tire forces from Fiala model:
                // - fx_tire: uses braking convention (positive slip → negative force)
                //            For drive, we negate: chassis fx = -fx_tire
                // - fy_tire: opposes slip angle, but sign meaning changes with travel direction
                let fx_tire = self.bus.true_state.wheel_states[i].tire.longitudinal_force;
                let fy_tire = self.bus.true_state.wheel_states[i].tire.lateral_force;
                let v_long = self.bus.true_state.wheel_states[i].longitudinal_translational_velocity;
                
                // For chassis: 
                // - Negate fx for drive force (braking convention → drive convention)
                // - fy sign depends on travel direction: keep as-is for forward, negate for backward
                //   (because slip angle atan2 produces different quadrants for backward motion)
                let fx = -fx_tire;
                let fy = if v_long >= 0.0 { fy_tire } else { -fy_tire };
                
                f_long_total += fx;
                let (rx, ry) = self.wheel_pos[i];
                m_z += rx * fy - ry * fx;

                // Wheel rotational dynamics: motor torque accelerates wheel; tire creates reaction
                let tq_motor = self.bus.true_state.motors[i].applied_torque;
                let wheel_torque = tq_motor * GEAR_RATIO * DRIVE_EFFICIENCY;
                let omega = self.bus.true_state.wheel_states[i].driving_angular_velocity;
                
                // Tire force fx is the tractive force ON THE CHASSIS.
                // By Newton's 3rd law, the wheel experiences -fx at the contact patch.
                // The reaction TORQUE on the wheel is: -fx * radius
                // When fx > 0 (chassis pushed forward), wheel is slowed (negative torque for positive omega)
                // When fx < 0 (chassis braked), wheel is sped up (positive torque for positive omega)
                let tire_reaction = -fx * WHEEL_RADIUS;
                let net_torque = wheel_torque + tire_reaction;
                println!("wheel {}: tq_motor={:.2} Nm, wheel_tq={:.2} Nm, omega={:.2} rad/s, tire_react={:.2} Nm, net_tq={:.2} Nm, slip_ratio={:.2}", i, tq_motor, wheel_torque, omega, tire_reaction, net_torque, self.bus.true_state.wheel_states[i].tire.slip_ratio);
                let domega = net_torque / WHEEL_INERTIA;
                self.bus.true_state.wheel_states[i].driving_angular_velocity = omega + domega * outer_dt;
            }

            // Resistances
            let f_rr = C_RR * MASS * G * self.v.signum();
            let f_drag = 0.5 * RHO_AIR * C_DA * self.v * self.v * self.v.signum();

            // Net longitudinal and yaw dynamics
            let f_long = f_long_total - f_rr - f_drag;
            let a = f_long / MASS;
            // Yaw damping to prevent runaway spin
            m_z -= YAW_DAMPING * self.yaw_rate;
            let alpha = m_z / IZZ;

            self.v += a * outer_dt;
            self.yaw_rate += alpha * outer_dt;

            // Integrate pose
            self.x += self.v * self.yaw.cos() * outer_dt;
            self.y += self.v * self.yaw.sin() * outer_dt;
            self.yaw += self.yaw_rate * outer_dt;

            self.t += outer_dt;

            // Downsampled sample for plotting
            // Choose to sample every ~PLOT_DT seconds
            // We’ll check if enough time elapsed since last trace point
            // Simple approach: sample every N outer steps approximating PLOT_DT
            // Here we use time-based condition
            if (self.trace.t.back().copied().unwrap_or(0.0) + PLOT_DT) <= self.t {
                let omega_left = 0.5 * (self.bus.true_state.wheel_states[0].driving_angular_velocity + self.bus.true_state.wheel_states[1].driving_angular_velocity);
                let omega_right = 0.5 * (self.bus.true_state.wheel_states[2].driving_angular_velocity + self.bus.true_state.wheel_states[3].driving_angular_velocity);
                let fx = [
                    self.bus.true_state.wheel_states[0].tire.longitudinal_force,
                    self.bus.true_state.wheel_states[1].tire.longitudinal_force,
                    self.bus.true_state.wheel_states[2].tire.longitudinal_force,
                    self.bus.true_state.wheel_states[3].tire.longitudinal_force,
                ];
                let fy = [
                    self.bus.true_state.wheel_states[0].tire.lateral_force,
                    self.bus.true_state.wheel_states[1].tire.lateral_force,
                    self.bus.true_state.wheel_states[2].tire.lateral_force,
                    self.bus.true_state.wheel_states[3].tire.lateral_force,
                ];
                self.trace.push(
                    self.t,
                    self.bus.true_state.battery_state.voltage,
                    self.bus.true_state.battery_state.state_of_charge,
                    i_total,
                    i_q_sum,
                    i_d_sum,
                    self.v,
                    self.yaw_rate,
                    omega_left,
                    omega_right,
                    self.x,
                    self.y,
                    fx,
                    fy,
                );
            }
        }
    }

    fn sample(&mut self) {
        let omega_left = 0.5 * (self.bus.true_state.wheel_states[0].driving_angular_velocity + self.bus.true_state.wheel_states[1].driving_angular_velocity);
        let omega_right = 0.5 * (self.bus.true_state.wheel_states[2].driving_angular_velocity + self.bus.true_state.wheel_states[3].driving_angular_velocity);
        // aggregate for display
        let mut i_total = 0.0;
        let mut i_q_sum = 0.0;
        let mut i_d_sum = 0.0;
        for i in 0..4 {
            let m = &self.bus.true_state.motors[i];
            let u = self.bus.control_input.motor_inputs[i];
            i_total += m.current_q * u.duty_cycle_q + m.current_d * u.duty_cycle_d;
            i_q_sum += m.current_q;
            i_d_sum += m.current_d;
        }
        let fx = [
            self.bus.true_state.wheel_states[0].tire.longitudinal_force,
            self.bus.true_state.wheel_states[1].tire.longitudinal_force,
            self.bus.true_state.wheel_states[2].tire.longitudinal_force,
            self.bus.true_state.wheel_states[3].tire.longitudinal_force,
        ];
        let fy = [
            self.bus.true_state.wheel_states[0].tire.lateral_force,
            self.bus.true_state.wheel_states[1].tire.lateral_force,
            self.bus.true_state.wheel_states[2].tire.lateral_force,
            self.bus.true_state.wheel_states[3].tire.lateral_force,
        ];
        self.trace.push(
            self.t,
            self.bus.true_state.battery_state.voltage,
            self.bus.true_state.battery_state.state_of_charge,
            i_total,
            i_q_sum,
            i_d_sum,
            self.v,
            self.yaw_rate,
            omega_left,
            omega_right,
            self.x,
            self.y,
            fx,
            fy,
        );
    }
}

impl eframe::App for App {
    fn update(&mut self, ctx: &egui::Context, _frame: &mut eframe::Frame) {
        // Keyboard controls (held keys smoothly adjust throttles)
        self.handle_keyboard(ctx);
        // Advance sim in real-time unless paused
        if !self.paused {
            let now = Instant::now();
            let wall_dt = now.duration_since(self.last_frame).as_secs_f64();
            self.last_frame = now;
            let sim_dt = (wall_dt * self.sim_speed).min(0.05);
            self.update_sim(sim_dt);
        } else {
            self.last_frame = Instant::now();
        }

        egui::TopBottomPanel::top("controls").show(ctx, |ui| {
            ui.horizontal_wrapped(|ui| {
                if ui.button(if self.paused { "▶ Resume" } else { "⏸ Pause" }).clicked() {
                    self.paused = !self.paused;
                }
                if ui.button("⟲ Reset").clicked() { self.reset(); }

                ui.separator();
                ui.label("Sim speed");
                ui.add(egui::Slider::new(&mut self.sim_speed, 0.01..=8.0).logarithmic(true).suffix("×"));

                ui.separator();
                ui.label("Window");
                if ui.add(egui::Slider::new(&mut self.window_s, 2.0..=60.0).suffix(" s")).changed() {
                    self.trace.set_window_seconds(self.window_s, PLOT_DT);
                }

                ui.separator();
                ui.label("Left throttle (W/S, ←/→ spin)");
                ui.add(egui::Slider::new(&mut self.left_throttle, -1.0..=1.0));
                ui.label("Right throttle (I/K)");
                ui.add(egui::Slider::new(&mut self.right_throttle, -1.0..=1.0));

                ui.separator();
                ui.label(format!("Pose: x={:.2} m, y={:.2} m, yaw={:.1}°", self.x, self.y, self.yaw.to_degrees()));
                ui.label(format!("v={:.2} m/s, yaw_rate={:.2} rad/s", self.v, self.yaw_rate));
                ui.label("Keys: ↑/↓ both, ←/→ differential, space = zero");
            });
        });

        egui::CentralPanel::default().show(ctx, |ui| {
            // Top-down viewport first so it's always visible
            ui.heading("Top-Down View");
            self.viewport_controls(ui);
            self.draw_viewport(ui, 400.0);

            ui.separator();

            ui.columns(2, |cols| {
                cols[0].heading("Tire Forces Fx");
                Plot::new("tire_fx_plot")
                    .legend(Legend::default())
                    .allow_scroll(false)
                    .y_axis_min_width(48.0)
                .x_axis_label("Time (s)")
                .y_axis_label("Fx (N)")
                .show(&mut cols[0], |plot_ui| {
                    let x_min = (self.t - self.window_s).max(0.0);
                    let x_max = self.t.max(self.window_s * 0.1);
                    plot_ui.set_plot_bounds(PlotBounds::from_min_max([x_min, -300.0], [x_max, 300.0]));
                    plot_ui.line(Line::new("Fx FL", Trace::line(&self.trace.fx_fl, &self.trace.t)));
                    plot_ui.line(Line::new("Fx RL", Trace::line(&self.trace.fx_rl, &self.trace.t)));
                    plot_ui.line(Line::new("Fx FR", Trace::line(&self.trace.fx_fr, &self.trace.t)));
                    plot_ui.line(Line::new("Fx RR", Trace::line(&self.trace.fx_rr, &self.trace.t)));
                });

            cols[1].heading("Tire Forces Fy");
            Plot::new("tire_fy_plot")
                .legend(Legend::default())
                .allow_scroll(false)
                .y_axis_min_width(48.0)
                .x_axis_label("Time (s)")
                .y_axis_label("Fy (N)")
                .show(&mut cols[1], |plot_ui| {
                    let x_min = (self.t - self.window_s).max(0.0);
                    let x_max = self.t.max(self.window_s * 0.1);
                    plot_ui.set_plot_bounds(PlotBounds::from_min_max([x_min, -300.0], [x_max, 300.0]));
                    plot_ui.line(Line::new("Fy FL", Trace::line(&self.trace.fy_fl, &self.trace.t)));
                    plot_ui.line(Line::new("Fy RL", Trace::line(&self.trace.fy_rl, &self.trace.t)));
                    plot_ui.line(Line::new("Fy FR", Trace::line(&self.trace.fy_fr, &self.trace.t)));
                    plot_ui.line(Line::new("Fy RR", Trace::line(&self.trace.fy_rr, &self.trace.t)));
                });
            });

            ui.columns(2, |cols| {
                // Left: Battery and currents
                cols[0].heading("Battery & Currents");
                let y_axes_left = vec![
                    AxisHints::new_y().label("Current (A)"),
                    AxisHints::new_y()
                        .label("SoC / V")
                        .formatter(|mark, _| format!("{:.2}", mark.value / SOC_SCALE)),
                ];
                Plot::new("left_plot")
                    .legend(Legend::default())
                    .allow_scroll(false)
                    .y_axis_min_width(48.0)
                    .x_axis_label("Time (s)")
                    .y_axis_label("Current / SoC / V")
                    .custom_y_axes(y_axes_left)
                    .show(&mut cols[0], |plot_ui| {
                        let x_min = (self.t - self.window_s * 0.25).max(0.0);
                        let x_max = self.t.max(self.window_s * 0.1);
                        plot_ui.set_plot_bounds(PlotBounds::from_min_max([x_min, Y_LEFT_MIN], [x_max, Y_LEFT_MAX]));
                        plot_ui.line(Line::new("I_total (A)", Trace::line(&self.trace.i_total, &self.trace.t)));
                        plot_ui.line(Line::new("Σ I_q (A)", Trace::line(&self.trace.i_q_sum, &self.trace.t)));
                        plot_ui.line(Line::new("Σ I_d (A)", Trace::line(&self.trace.i_d_sum, &self.trace.t)));
                        plot_ui.line(Line::new("SoC", Trace::line_scaled(&self.trace.soc, &self.trace.t, SOC_SCALE)));
                        plot_ui.line(Line::new("V_batt", Trace::line_scaled(&self.trace.batt_v, &self.trace.t, VOLT_SCALE)));
                    });

                // Right: chassis dynamics
                cols[1].heading("Chassis Dynamics");
                let y_axes_right = vec![
                    AxisHints::new_y().label("Force/—"),
                    AxisHints::new_y().label("v / yaw rate")
                        .formatter(|mark, _| format!("{:.2}", mark.value / VEL_SCALE)),
                ];
                Plot::new("right_plot")
                    .legend(Legend::default())
                    .allow_scroll(false)
                    .y_axis_min_width(48.0)
                    .x_axis_label("Time (s)")
                    .y_axis_label("Dynamics")
                    .custom_y_axes(y_axes_right)
                    .show(&mut cols[1], |plot_ui| {
                        let x_min = (self.t - self.window_s).max(0.0);
                        let x_max = self.t.max(self.window_s * 0.1);
                        plot_ui.set_plot_bounds(PlotBounds::from_min_max([x_min, Y_RIGHT_MIN], [x_max, Y_RIGHT_MAX]));
                        // Scaled v and yaw rate
                        plot_ui.line(Line::new("v (m/s)", Trace::line_scaled(&self.trace.v, &self.trace.t, VEL_SCALE)));
                        plot_ui.line(Line::new("yaw_rate (rad/s)", Trace::line_scaled(&self.trace.yaw_rate, &self.trace.t, YAW_SCALE)));
                        // Also show left/right wheel speed (rad/s) unscaled but same axis
                        plot_ui.line(Line::new("ω_left (rad/s)", Trace::line(&self.trace.left_omega, &self.trace.t)));
                        plot_ui.line(Line::new("ω_right (rad/s)", Trace::line(&self.trace.right_omega, &self.trace.t)));
                    });


                    
            });

            ui.separator();

            

        });

        ctx.request_repaint_after(Duration::from_millis(10));
    }
}

// Helpers for keyboard and drawing
impl App {
    fn handle_keyboard(&mut self, ctx: &egui::Context) {
        let set = 0.5;
        let mut x = 0.0;
        let mut rot = 0.0;
        ctx.input(|i| {
            if i.key_down(egui::Key::ArrowUp) {x = 0.8;} 
            if i.key_down(egui::Key::ArrowDown) { x=-0.8; }
            if i.key_down(egui::Key::ArrowLeft) { rot =-0.2; }
            if i.key_down(egui::Key::ArrowRight) { rot = 0.2; }
            if i.key_pressed(egui::Key::Space) { x = 0.0; rot = 0.0; }
        });

        self.left_throttle = x - rot;
        self.right_throttle = x + rot;

    }

    fn viewport_controls(&mut self, ui: &mut egui::Ui) {
        ui.horizontal(|ui| {
            ui.checkbox(&mut self.view_follow, "Follow robot");
            ui.checkbox(&mut self.view_show_grid, "Grid");
            ui.checkbox(&mut self.view_show_path, "Path");
            ui.label("Zoom");
            ui.add(egui::Slider::new(&mut self.view_scale, 40.0..=300.0).suffix(" px/m"));
        });
    }

    fn draw_viewport(&mut self, ui: &mut egui::Ui, height_px: f32) {
        let desired = egui::vec2(ui.available_width(), height_px);
        let (response, painter) = ui.allocate_painter(desired, egui::Sense::drag());

        // pan with mouse drag
        static mut VIEW_CENTER: (f64, f64) = (0.0, 0.0);
        if self.view_follow {
            unsafe { VIEW_CENTER = (self.x, self.y); }
        } else if response.dragged() {
            let delta = response.drag_delta();
            unsafe {
                VIEW_CENTER.0 -= delta.x as f64 / self.view_scale as f64;
                VIEW_CENTER.1 += delta.y as f64 / self.view_scale as f64;
            }
        }
        let (cx, cy) = unsafe { VIEW_CENTER };

        // wheel scroll to zoom when hovered
        if response.hovered() {
            let scroll_y: f32 = ui.input(|i| i.raw_scroll_delta.y);
            if scroll_y.abs() > 0.0 {
                let factor: f32 = (1.0f32 + scroll_y / 600.0f32).clamp(0.5, 1.5);
                self.view_scale = (self.view_scale * factor).clamp(30.0, 600.0);
            }
        }

        // transforms: world (meters) <-> screen (pixels)
        let to_screen = |wx: f64, wy: f64| -> egui::Pos2 {
            let sx = ((wx - cx) as f32) * self.view_scale + response.rect.center().x;
            let sy = response.rect.center().y - ((wy - cy) as f32) * self.view_scale;
            egui::pos2(sx, sy)
        };

        // background
        painter.rect_filled(response.rect, 4.0, ui.visuals().extreme_bg_color);

        // grid
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

        // path
        if self.view_show_path && self.trace.px.len() > 1 {
            let points: Vec<egui::Pos2> = self
                .trace
                .px
                .iter()
                .copied()
                .zip(self.trace.py.iter().copied())
                .map(|(x, y)| to_screen(x, y))
                .collect();
            painter.add(egui::Shape::line(points, egui::Stroke::new(2.0, egui::Color32::LIGHT_BLUE)));
        }

        // robot footprint
        let hl = ROBOT_LENGTH * 0.5;
        let hw = ROBOT_WIDTH * 0.5;
        let (c, s) = (self.yaw.cos(), self.yaw.sin());
        let body = [
            [ hl,  hw],
            [ hl, -hw],
            [-hl, -hw],
            [-hl,  hw],
        ];
        let poly: Vec<egui::Pos2> = body
            .into_iter()
            .map(|[bx, by]| {
                let wx = self.x + c * bx - s * by;
                let wy = self.y + s * bx + c * by;
                to_screen(wx, wy)
            })
            .collect();
        painter.add(egui::Shape::closed_line(
            poly.clone(),
            egui::Stroke::new(2.0, egui::Color32::YELLOW),
        ));
        painter.add(egui::Shape::convex_polygon(
            poly,
            egui::Color32::from_rgba_unmultiplied(255, 255, 0, 24),
            egui::Stroke::NONE,
        ));

        // heading arrow
        let arrow_len = ROBOT_LENGTH * 0.6;
        let tip = to_screen(self.x + self.yaw.cos() * arrow_len, self.y + self.yaw.sin() * arrow_len);
        let base = to_screen(self.x, self.y);
        painter.line_segment([base, tip], egui::Stroke::new(3.0, egui::Color32::RED));
    }
}
