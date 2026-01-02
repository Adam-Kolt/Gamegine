//! Elevator PID Control Example
//!
//! Demonstrates position control with:
//! - Vertical elevator with mass and gravity
//! - Geared motor with battery model
//! - PID position control with velocity and current inner loops
//! - Real-time visualization

use control::{
    ControlMode, FocCommutation, MotorController, MotorControllerConfig, PidfConfig,
    TrapezoidalCommutation,
};
use electrical::battery::{Battery, BatteryConstant};
use electrical::motor::{MotorBank, MotorConstant};
use simcore::{ElectricalModel, MotorInput, MotorState, SimContext, SimState};

use egui_plot::{AxisHints, Legend, Line, Plot, PlotBounds, PlotPoints};
use std::collections::VecDeque;
use std::time::{Duration, Instant};

// Simulation timesteps
const DT_OUTER: f64 = 1e-3; // Outer loop (mechanics + battery)
const DT_ELEC: f64 = 1e-4; // Electrical inner loop

// Elevator physical parameters
const ELEVATOR_MASS: f64 = 40.0; // kg
const DRUM_RADIUS: f64 = 0.10; // m (2.5 cm drum)
const GEAR_RATIO: f64 = 20.0; // 20:1 reduction
const DRIVE_EFFICIENCY: f64 = 0.90;
const GRAVITY: f64 = 9.81; // m/s^2
const FRICTION: f64 = 2.0; // N of friction (constant)

// Cable/pulley inertia reflected to motor
const MOTOR_INERTIA: f64 = 0.0001; // kg*m^2
const CABLE_DAMPING: f64 = 0.0005; // Nm/(rad/s)

// Elevator limits
const MIN_HEIGHT: f64 = 0.0; // m
const MAX_HEIGHT: f64 = 2.0; // m

// Plot settings
const PLOT_DT: f64 = 1e-2;

// Plot Y-axis bounds
const Y_POS_MIN: f64 = -0.5;
const Y_POS_MAX: f64 = 2.5;
const Y_CURRENT_MIN: f64 = -80.0;
const Y_CURRENT_MAX: f64 = 80.0;

fn main() -> eframe::Result<()> {
    let options = eframe::NativeOptions {
        viewport: egui::ViewportBuilder::default()
            .with_inner_size([1280.0, 800.0])
            .with_title("Elevator PID Control"),
        ..Default::default()
    };
    eframe::run_native(
        "Elevator PID Control",
        options,
        Box::new(|_cc| Ok(Box::new(App::new()))),
    )
}

struct Trace {
    t: VecDeque<f64>,
    height: VecDeque<f64>,
    height_setpoint: VecDeque<f64>,
    velocity: VecDeque<f64>,
    current_q: VecDeque<f64>,
    torque: VecDeque<f64>,
    batt_v: VecDeque<f64>,
    soc: VecDeque<f64>,
    capacity: usize,
}

impl Trace {
    fn new(seconds: f64, sample_dt: f64) -> Self {
        let capacity = (seconds / sample_dt).ceil() as usize + 1;
        Self {
            t: VecDeque::with_capacity(capacity),
            height: VecDeque::with_capacity(capacity),
            height_setpoint: VecDeque::with_capacity(capacity),
            velocity: VecDeque::with_capacity(capacity),
            current_q: VecDeque::with_capacity(capacity),
            torque: VecDeque::with_capacity(capacity),
            batt_v: VecDeque::with_capacity(capacity),
            soc: VecDeque::with_capacity(capacity),
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
        height: f64,
        height_setpoint: f64,
        velocity: f64,
        current_q: f64,
        torque: f64,
        batt_v: f64,
        soc: f64,
    ) {
        self.t.push_back(t);
        self.height.push_back(height);
        self.height_setpoint.push_back(height_setpoint);
        self.velocity.push_back(velocity);
        self.current_q.push_back(current_q);
        self.torque.push_back(torque);
        self.batt_v.push_back(batt_v);
        self.soc.push_back(soc);
        self.trim_to_capacity();
    }

    fn trim_to_capacity(&mut self) {
        let cap = self.capacity;
        let trim = |v: &mut VecDeque<f64>| {
            while v.len() > cap {
                v.pop_front();
            }
        };
        trim(&mut self.t);
        trim(&mut self.height);
        trim(&mut self.height_setpoint);
        trim(&mut self.velocity);
        trim(&mut self.current_q);
        trim(&mut self.torque);
        trim(&mut self.batt_v);
        trim(&mut self.soc);
    }

    fn line<'a>(points: &'a VecDeque<f64>, t: &'a VecDeque<f64>) -> PlotPoints<'a> {
        PlotPoints::from_iter(
            t.iter()
                .copied()
                .zip(points.iter().copied())
                .map(|(x, y)| [x, y]),
        )
    }

    fn line_scaled<'a>(points: &'a VecDeque<f64>, t: &'a VecDeque<f64>, scale: f64) -> PlotPoints<'a> {
        PlotPoints::from_iter(
            t.iter()
                .copied()
                .zip(points.iter().copied().map(|y| y * scale))
                .map(|(x, y)| [x, y]),
        )
    }
}

struct App {
    // Models
    batt: Battery,
    motors: MotorBank,
    motor_controller: MotorController,
    bus: SimState,

    // Elevator state
    height: f64,         // m
    velocity: f64,       // m/s
    height_setpoint: f64, // m

    // Simulation
    t: f64,
    paused: bool,
    last_frame: Instant,
    sim_speed: f64,

    // Settings
    use_trapezoidal: bool,
    window_s: f64,

    // Trace
    trace: Trace,
}

impl App {
    fn new() -> Self {
        let motor = MotorConstant::kraken_x60();
        
        let batt = Battery {
            constants: BatteryConstant::default(),
        };
        
        let mut motors = MotorBank::default();
        motors.add_motor(motor);

        // Create motor controller with position control
        // Position mode now outputs duty directly (bypassing current loop)
        // Position controller: position error (rad) -> target velocity (rad/s)
        // Velocity controller: velocity error (rad/s) -> duty cycle (-1 to 1)
        let config = MotorControllerConfig::new(motor)
            .with_mode(ControlMode::Position)
            .with_position_controller(
                // Low P gain: position error in radians -> target velocity in rad/s
                // With motor positions ~800 rad for 1m, we want low gain
                PidfConfig::p(100.0).with_limits(-2000.0, 2000.0)
            )
            .with_velocity_controller(
                // Velocity controller outputs duty (-1 to 1)
                // At typical motor speeds (~100-500 rad/s), we need small gains
                PidfConfig::pi(0.002, 0.01).with_limits(-1.0, 1.0).with_i_max(0.5)
            )
            .with_max_current(60.0);

        let motor_controller = MotorController::with_commutation(config, Box::new(FocCommutation));

        let mut bus = SimState::default();
        bus.control_input.motor_inputs = vec![MotorInput {
            duty_cycle_d: 0.0,
            duty_cycle_q: 0.0,
        }];
        bus.true_state.motors = vec![MotorState::default()];

        let mut app = Self {
            batt,
            motors,
            motor_controller,
            bus,
            height: 0.0,
            velocity: 0.0,
            height_setpoint: 0.5,
            t: 0.0,
            paused: false,
            last_frame: Instant::now(),
            sim_speed: 1.0,
            use_trapezoidal: false,
            window_s: 10.0,
            trace: Trace::new(10.0, PLOT_DT),
        };
        
        app.sample();
        app
    }

    fn reset(&mut self) {
        let motor = MotorConstant::kraken_x60();
        self.batt = Battery {
            constants: BatteryConstant::default(),
        };
        self.motors = MotorBank::default();
        self.motors.add_motor(motor);
        
        self.bus = SimState::default();
        self.bus.control_input.motor_inputs = vec![MotorInput {
            duty_cycle_d: 0.0,
            duty_cycle_q: 0.0,
        }];
        self.bus.true_state.motors = vec![MotorState::default()];
        
        self.motor_controller.reset();
        self.height = 0.0;
        self.velocity = 0.0;
        self.t = 0.0;
        self.trace = Trace::new(self.window_s, PLOT_DT);
    }

    fn update_commutation(&mut self) {
        let motor = MotorConstant::kraken_x60();
        let config = MotorControllerConfig::new(motor)
            .with_mode(ControlMode::Position)
            .with_position_controller(
                PidfConfig::p(0.5).with_limits(-200.0, 200.0)
            )
            .with_velocity_controller(
                PidfConfig::pi(0.002, 0.01).with_limits(-1.0, 1.0).with_i_max(0.5)
            )
            .with_max_current(60.0);

        if self.use_trapezoidal {
            self.motor_controller = MotorController::with_commutation(
                config,
                Box::new(TrapezoidalCommutation::default()),
            );
        } else {
            self.motor_controller = MotorController::with_commutation(config, Box::new(FocCommutation));
        }
    }

    fn update_sim(&mut self, sim_dt: f64) {
        let steps = (sim_dt / DT_OUTER).ceil().max(1.0) as usize;
        let outer_dt = sim_dt / steps as f64;

        for _ in 0..steps {
            // Position control using motor controller
            // Motor position setpoint from height setpoint
            let motor_position_setpoint = self.height_setpoint * GEAR_RATIO / DRUM_RADIUS;
            self.motor_controller.set_setpoint(motor_position_setpoint);
            
            // Sync motor controller position with actual motor position
            let motor_position = self.height * GEAR_RATIO / DRUM_RADIUS;
            self.motor_controller.set_position(motor_position);

            // Motor velocity is rigidly coupled to elevator velocity through gearing
            let motor_velocity = self.velocity * GEAR_RATIO / DRUM_RADIUS;
            self.bus.true_state.motors[0].mechanical_velocity = motor_velocity;

            // Run motor controller (Position mode -> velocity controller -> duty)
            let motor_input = self.motor_controller.update(&self.bus.true_state.motors[0], outer_dt);
            self.bus.control_input.motor_inputs[0] = motor_input;

            // Inner electrical loop
            let mut t_inner = 0.0;
            while t_inner < outer_dt {
                let dt = (outer_dt - t_inner).min(DT_ELEC);
                
                // Keep motor velocity synchronized during electrical integration
                self.bus.true_state.motors[0].mechanical_velocity = 
                    self.velocity * GEAR_RATIO / DRUM_RADIUS;
                
                self.motors.step_electrical(
                    SimContext {
                        dt,
                        t: self.t + t_inner,
                    },
                    &mut self.bus,
                );
                t_inner += dt;
            }

            // Motor torque -> elevator dynamics
            let motor_torque = self.bus.true_state.motors[0].applied_torque;
            
            // Torque at drum after gearing (motor torque is amplified by gear ratio)
            let drum_torque = motor_torque * GEAR_RATIO * DRIVE_EFFICIENCY;
            
            // Force on elevator cable
            let cable_force = drum_torque / DRUM_RADIUS;
            
            // Equivalent inertia of motor reflected to elevator
            // J_motor_reflected = J_motor * (GEAR_RATIO / DRUM_RADIUS)^2
            let motor_reflected_inertia = MOTOR_INERTIA * (GEAR_RATIO / DRUM_RADIUS).powi(2);
            let total_mass = ELEVATOR_MASS + motor_reflected_inertia;
            
            // Net force: cable - gravity - friction (positive = up)
            let gravity_force = ELEVATOR_MASS * GRAVITY;
            let friction_force = if self.velocity.abs() > 0.001 {
                FRICTION * self.velocity.signum()
            } else {
                0.0  // No friction at rest to avoid oscillation
            };
            
            // Damping (proportional to velocity)
            let damping_force = CABLE_DAMPING * self.velocity * (GEAR_RATIO / DRUM_RADIUS).powi(2);
            
            let net_force = cable_force - gravity_force - friction_force - damping_force;
            
            // Elevator acceleration
            let accel = net_force / total_mass;
            
            // Integrate elevator state
            self.velocity += accel * outer_dt;
            self.height += self.velocity * outer_dt;
            
            // Clamp to physical limits (hit floor/ceiling)
            if self.height < MIN_HEIGHT {
                self.height = MIN_HEIGHT;
                if self.velocity < 0.0 {
                    self.velocity = 0.0;
                }
            }
            if self.height > MAX_HEIGHT {
                self.height = MAX_HEIGHT;
                if self.velocity > 0.0 {
                    self.velocity = 0.0;
                }
            }

            // Battery current draw
            let m = &self.bus.true_state.motors[0];
            let inp = &self.bus.control_input.motor_inputs[0];
            self.bus.true_state.battery_state.total_current_draw =
                m.current_q * inp.duty_cycle_q + m.current_d * inp.duty_cycle_d;

            // Step battery
            self.batt.step_electrical(SimContext { dt: outer_dt, t: self.t }, &mut self.bus);

            self.t += outer_dt;

            // Sample for plotting
            if self.trace.t.back().copied().unwrap_or(-1.0) + PLOT_DT <= self.t {
                self.sample();
            }
        }
    }

    fn sample(&mut self) {
        self.trace.push(
            self.t,
            self.height,
            self.height_setpoint,
            self.velocity,
            self.bus.true_state.motors[0].current_q,
            self.bus.true_state.motors[0].applied_torque,
            self.bus.true_state.battery_state.voltage,
            self.bus.true_state.battery_state.state_of_charge,
        );
    }
}

impl eframe::App for App {
    fn update(&mut self, ctx: &egui::Context, _frame: &mut eframe::Frame) {
        // Advance simulation
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
                if ui
                    .button(if self.paused { "▶ Resume" } else { "⏸ Pause" })
                    .clicked()
                {
                    self.paused = !self.paused;
                }
                if ui.button("⟲ Reset").clicked() {
                    self.reset();
                }

                ui.separator();
                ui.label("Sim speed");
                ui.add(
                    egui::Slider::new(&mut self.sim_speed, 0.1..=5.0)
                        .logarithmic(true)
                        .suffix("×"),
                );

                ui.separator();
                ui.label("Window");
                if ui
                    .add(egui::Slider::new(&mut self.window_s, 2.0..=60.0).suffix(" s"))
                    .changed()
                {
                    self.trace.set_window_seconds(self.window_s, PLOT_DT);
                }

                ui.separator();
                ui.label("Height Setpoint");
                ui.add(egui::Slider::new(&mut self.height_setpoint, 0.0..=2.0).suffix(" m"));

                ui.separator();
                let old_trap = self.use_trapezoidal;
                ui.checkbox(&mut self.use_trapezoidal, "Trapezoidal Commutation");
                if old_trap != self.use_trapezoidal {
                    self.update_commutation();
                }

                ui.separator();
                ui.label(format!(
                    "Height: {:.3} m | Velocity: {:.3} m/s",
                    self.height, self.velocity
                ));
            });
        });

        egui::CentralPanel::default().show(ctx, |ui| {
            ui.columns(2, |cols| {
                // Left: Position & Velocity
                cols[0].heading("Position & Velocity");
                
                let y_axes_pos = vec![
                    AxisHints::new_y().label("Height (m)"),
                    AxisHints::new_y()
                        .label("Velocity (m/s)")
                        .formatter(|mark, _| format!("{:.2}", mark.value)),
                ];

                Plot::new("position_plot")
                    .legend(Legend::default())
                    .allow_scroll(false)
                    .y_axis_min_width(48.0)
                    .x_axis_label("Time (s)")
                    .custom_y_axes(y_axes_pos)
                    .show(&mut cols[0], |plot_ui| {
                        let x_min = (self.t - self.window_s).max(0.0);
                        let x_max = self.t.max(self.window_s * 0.1);
                        plot_ui.set_plot_bounds(PlotBounds::from_min_max(
                            [x_min, Y_POS_MIN],
                            [x_max, Y_POS_MAX],
                        ));

                        plot_ui.line(Line::new(
                            "Height (m)",
                            Trace::line(&self.trace.height, &self.trace.t),
                        ));
                        plot_ui.line(Line::new(
                            "Setpoint (m)",
                            Trace::line(&self.trace.height_setpoint, &self.trace.t),
                        ));
                        plot_ui.line(Line::new(
                            "Velocity (m/s)",
                            Trace::line(&self.trace.velocity, &self.trace.t),
                        ));
                    });

                // Right: Current & Battery
                cols[1].heading("Current & Battery");

                let y_axes_curr = vec![
                    AxisHints::new_y().label("Current (A)"),
                    AxisHints::new_y()
                        .label("Voltage (V)")
                        .formatter(|mark, _| format!("{:.1}", mark.value / 5.0)),
                ];

                Plot::new("current_plot")
                    .legend(Legend::default())
                    .allow_scroll(false)
                    .y_axis_min_width(48.0)
                    .x_axis_label("Time (s)")
                    .custom_y_axes(y_axes_curr)
                    .show(&mut cols[1], |plot_ui| {
                        let x_min = (self.t - self.window_s).max(0.0);
                        let x_max = self.t.max(self.window_s * 0.1);
                        plot_ui.set_plot_bounds(PlotBounds::from_min_max(
                            [x_min, Y_CURRENT_MIN],
                            [x_max, Y_CURRENT_MAX],
                        ));

                        plot_ui.line(Line::new(
                            "I_q (A)",
                            Trace::line(&self.trace.current_q, &self.trace.t),
                        ));
                        plot_ui.line(Line::new(
                            "Torque (Nm)",
                            Trace::line(&self.trace.torque, &self.trace.t),
                        ));
                        plot_ui.line(Line::new(
                            "V_batt (V)",
                            Trace::line_scaled(&self.trace.batt_v, &self.trace.t, 5.0),
                        ));
                    });
            });

            // Elevator visualization
            ui.separator();
            ui.heading("Elevator Visualization");
            
            let desired_size = egui::vec2(200.0, 300.0);
            let (response, painter) = ui.allocate_painter(desired_size, egui::Sense::hover());
            let rect = response.rect;
            
            // Background
            painter.rect_filled(rect, 4.0, egui::Color32::from_rgb(40, 40, 50));
            
            // Shaft
            let shaft_rect = egui::Rect::from_min_size(
                rect.min + egui::vec2(70.0, 20.0),
                egui::vec2(60.0, 260.0),
            );
            painter.rect_stroke(shaft_rect, 2.0, egui::Stroke::new(2.0, egui::Color32::GRAY), egui::StrokeKind::Inside);
            
            // Elevator car
            let normalized_height = ((self.height - MIN_HEIGHT) / (MAX_HEIGHT - MIN_HEIGHT)) as f32;
            let car_y = shaft_rect.max.y - 50.0 - normalized_height * (shaft_rect.height() - 60.0);
            let car_rect = egui::Rect::from_min_size(
                egui::pos2(shaft_rect.min.x + 5.0, car_y),
                egui::vec2(50.0, 50.0),
            );
            painter.rect_filled(car_rect, 4.0, egui::Color32::from_rgb(100, 150, 200));
            
            // Setpoint marker
            let setpoint_normalized = ((self.height_setpoint - MIN_HEIGHT) / (MAX_HEIGHT - MIN_HEIGHT)) as f32;
            let setpoint_y = shaft_rect.max.y - 25.0 - setpoint_normalized * (shaft_rect.height() - 60.0);
            painter.line_segment(
                [
                    egui::pos2(shaft_rect.min.x - 10.0, setpoint_y),
                    egui::pos2(shaft_rect.max.x + 10.0, setpoint_y),
                ],
                egui::Stroke::new(2.0, egui::Color32::GREEN),
            );
            
            // Labels
            painter.text(
                egui::pos2(rect.min.x + 10.0, rect.min.y + 30.0),
                egui::Align2::LEFT_TOP,
                format!("2.0 m"),
                egui::FontId::default(),
                egui::Color32::WHITE,
            );
            painter.text(
                egui::pos2(rect.min.x + 10.0, rect.max.y - 30.0),
                egui::Align2::LEFT_BOTTOM,
                format!("0.0 m"),
                egui::FontId::default(),
                egui::Color32::WHITE,
            );
        });

        ctx.request_repaint_after(Duration::from_millis(10));
    }
}
