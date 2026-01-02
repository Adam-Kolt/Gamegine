use electrical::battery::{Battery, BatteryConstant};
use electrical::motor::{MotorBank, MotorConstant};
use simcore::{ElectricalModel, SimContext, SimState, MotorInput, MotorState};

use egui_plot::{
    AxisHints, Legend, Line, Plot, PlotBounds, PlotPoints,

};
use std::collections::VecDeque;
use std::time::{Duration, Instant};

const DT: f64 = 1e-4;          // simulation timestep (s)
const DT_MOTOR: f64 = 1e-4;    // motor inner loop timestep (s)
const MOTOR_LOAD_INERTIA: f64 = 0.01; // kg*m^2 (example)
const PLOT_DT: f64 = 1e-2;      // plot update timestep (s)

// ===== Fixed (non-autoscaling) bounds and per-series display scales =====

// Left plot (Battery & Currents) Y-range — all series must be scaled to fit this
const Y_LEFT_MIN: f64  = -400.0;
const Y_LEFT_MAX: f64  =  400.0;

// Right plot (Motor Dynamics) Y-range
const Y_RIGHT_MIN: f64 =  -15.0;
const Y_RIGHT_MAX: f64 =   15.0;

// Per-series display scales (data are multiplied by these before plotting)

const SOC_SCALE: f64 = 350.0;  // SoC 0..1 -> 0..350 so it’s visible on left plot
const VOLT_SCALE: f64 = 400.0/14.0;   // V_batt (V) * 0.10 so e.g. 500 V -> 50 on right plot
const VEL_SCALE: f64 = 15.0/700.0;   // ω (rad/s) * 0.10 so e.g. 500 rad/s -> 50 on right plot

fn main() -> eframe::Result<()> {
    let options = eframe::NativeOptions {
        viewport: egui::ViewportBuilder::default()
            .with_inner_size([1200.0, 800.0])
            .with_title("Realtime Motor + Battery Dashboard"),
        ..Default::default()
    };
    eframe::run_native(
        "Realtime Motor + Battery Dashboard",
        options,
        Box::new(|_cc| Ok(Box::new(App::new()))),
    )
}

struct Trace {
    t: VecDeque<f64>,
    batt_v: VecDeque<f64>,
    soc: VecDeque<f64>,
    i_total: VecDeque<f64>,
    i_q: VecDeque<f64>,
    i_d: VecDeque<f64>,
    mech_vel: VecDeque<f64>,
    torque: VecDeque<f64>,
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
            i_q: VecDeque::with_capacity(capacity),
            i_d: VecDeque::with_capacity(capacity),
            mech_vel: VecDeque::with_capacity(capacity),
            torque: VecDeque::with_capacity(capacity),
            capacity,
        }
    }

    fn set_window_seconds(&mut self, seconds: f64, sample_dt: f64) {
        self.capacity = (seconds / sample_dt).ceil() as usize + 1;
        self.trim_to_capacity();
    }

    fn push(&mut self, t: f64, batt_v: f64, soc: f64, i_total: f64, i_q: f64, i_d: f64, mech_vel: f64, torque: f64) {
        self.t.push_back(t);
        self.batt_v.push_back(batt_v);
        self.soc.push_back(soc);
        self.i_total.push_back(i_total);
        self.i_q.push_back(i_q);
        self.i_d.push_back(i_d);
        self.mech_vel.push_back(mech_vel);
        self.torque.push_back(torque);
        self.trim_to_capacity();
    }

    fn trim_to_capacity(&mut self) {
        let mut trim = |v: &mut VecDeque<f64>| while v.len() > self.capacity { v.pop_front(); };
        trim(&mut self.t);
        trim(&mut self.batt_v);
        trim(&mut self.soc);
        trim(&mut self.i_total);
        trim(&mut self.i_q);
        trim(&mut self.i_d);
        trim(&mut self.mech_vel);
        trim(&mut self.torque);
    }

    fn line<'a>(points: &'a VecDeque<f64>, t: &'a VecDeque<f64>) -> PlotPoints<'a> {
        PlotPoints::from_iter(
            t.iter()
                .copied()
                .zip(points.iter().copied())
                .map(|(x, y)| [x, y])
        )
    }

    // NEW: scaled line helper (for “independent” axes via data scaling)
    fn line_scaled<'a>(points: &'a VecDeque<f64>, t: &'a VecDeque<f64>, scale: f64) -> PlotPoints<'a> {
        PlotPoints::from_iter(
            t.iter()
                .copied()
                .zip(points.iter().copied().map(|y| y * scale))
                .map(|(x, y)| [x, y])
        )
    }
}

struct App {
    // Simulation
    batt: Battery,
    motor_bank: MotorBank,
    bus: SimState,
    t: f64,
    paused: bool,
    last_frame: Instant,
    sim_speed: f64,       // 1.0 = real-time, 2.0 = 2x, etc.

    // Control
    duty_q: f64,          // slider-controlled q-axis duty (0..1)
    duty_d: f64,          // keep at 0 for now, but exposed for completeness
    use_sine_input: bool, // optional: sine modulation if desired
    sine_period_s: f64,

    // Plotting
    trace: Trace,
    window_s: f64,        // rolling window size (s)
}

impl App {
    fn new() -> Self {
        // Battery & motors
        let batt = Battery { constants: BatteryConstant::default() };
        let mut motor_bank = MotorBank::default();
        let motor_constant = MotorConstant::kraken_x60();
        motor_bank.add_motor(motor_constant);

        // Shared bus
        let mut bus = SimState::default();
        bus.control_input.motor_inputs = vec![MotorInput { duty_cycle_d: 0.0, duty_cycle_q: 0.0 }; motor_bank.motor_constants.len()];
        bus.true_state.motors = vec![MotorState::default(); motor_bank.motor_constants.len()];

        let mut app = Self {
            batt,
            motor_bank,
            bus,
            t: 0.0,
            paused: false,
            last_frame: Instant::now(),
            sim_speed: 1.0,

            duty_q: 0.0,
            duty_d: 0.0,
            use_sine_input: false,
            sine_period_s: 2.5,

            window_s: 10.0,
            trace: Trace::new(10.0, PLOT_DT), // store at ~1 kHz for plotting
        };

        // Seed first sample
        app.sample();
        app
    }

    fn reset(&mut self) {
        self.batt = Battery { constants: BatteryConstant::default() };
        self.motor_bank = {
            let mut m = MotorBank::default();
            m.add_motor(MotorConstant::kraken_x60());
            m
        };
        self.bus = SimState::default();
        self.bus.control_input.motor_inputs = vec![MotorInput { duty_cycle_d: 0.0, duty_cycle_q: 0.0 }; self.motor_bank.motor_constants.len()];
        self.bus.true_state.motors = vec![MotorState::default(); self.motor_bank.motor_constants.len()];
        self.t = 0.0;
        self.trace = Trace::new(self.window_s, 1e-3);
    }

    fn update_sim(&mut self, sim_dt: f64) {
        // break sim_dt into fixed steps of DT
        let steps = (sim_dt / DT).ceil().max(1.0) as usize;
        let actual_dt = sim_dt / steps as f64;

        for i in 0..steps {
            // Control input (either fixed slider or sine)
            let (dq, dd) = if self.use_sine_input && self.sine_period_s > 0.0 {
                let w = 2.0 * std::f64::consts::PI / self.sine_period_s;
                ((self.t * w).sin(), 0.0)
            } else {
                (self.duty_q.clamp(-1.0, 1.0), self.duty_d.clamp(-1.0, 1.0))
            };
            self.bus.control_input.motor_inputs[0] = MotorInput { duty_cycle_q: dq, duty_cycle_d: dd };

            // Inner motor loop at DT_MOTOR
            let mut t_inner = 0.0;
            while t_inner < actual_dt {
                let inner_dt = (actual_dt - t_inner).min(DT_MOTOR);
                self.motor_bank.step_electrical(SimContext { dt: inner_dt, t: self.t + t_inner }, &mut self.bus);

                // Simple rigid-body integrator for mechanical velocity
                let m0 = &mut self.bus.true_state.motors[0];
                m0.mechanical_velocity += (m0.applied_torque / MOTOR_LOAD_INERTIA) * inner_dt;

                t_inner += inner_dt;
            }

            // Aggregate load -> total current draw (q/d weighted by duty)
            let m0 = &self.bus.true_state.motors[0];
            self.bus.true_state.battery_state.total_current_draw =
                m0.current_q * self.bus.control_input.motor_inputs[0].duty_cycle_q +
                m0.current_d * self.bus.control_input.motor_inputs[0].duty_cycle_d;

            // Battery step
            self.batt.step_electrical(SimContext { dt: actual_dt, t: self.t }, &mut self.bus);

            self.t += actual_dt;

            // Downsample for plotting at ~1 kHz
            let downsample_scale = (PLOT_DT / DT).round() as usize;
            if i % downsample_scale == 0 {
                self.sample();
            }
        }
        
    }

    fn sample(&mut self) {
        let bs = &self.bus.true_state.battery_state;
        let m0 = &self.bus.true_state.motors[0];
        self.trace.push(
            self.t,
            bs.voltage,
            bs.state_of_charge,
            bs.total_current_draw,
            m0.current_q,
            m0.current_d,
            m0.mechanical_velocity,
            m0.applied_torque,
        );
    }
}

impl eframe::App for App {
    fn update(&mut self, ctx: &egui::Context, _frame: &mut eframe::Frame) {
        // SIMULATION ADVANCE
        if !self.paused {
            let now = Instant::now();
            let wall_dt = now.duration_since(self.last_frame);
            self.last_frame = now;

            // simulate ahead by (wall time * speed)
            let sim_dt = (wall_dt.as_secs_f64() * self.sim_speed).min(0.050); // clamp to keep up
            self.update_sim(sim_dt);
        } else {
            self.last_frame = Instant::now(); // avoid large jump after unpausing
        }

        egui::TopBottomPanel::top("controls").show(ctx, |ui| {
            ui.horizontal_wrapped(|ui| {
                if ui.button(if self.paused { "▶ Resume" } else { "⏸ Pause" }).clicked() {
                    self.paused = !self.paused;
                }
                if ui.button("⟲ Reset").clicked() {
                    self.reset();
                }

                ui.separator();

                ui.label("Sim speed");
                ui.add(egui::Slider::new(&mut self.sim_speed, 0.1..=10.0).logarithmic(true).suffix("×"));

                ui.separator();

                ui.label("Window");
                if ui.add(egui::Slider::new(&mut self.window_s, 2.0..=120.0).suffix(" s")).changed() {
                    self.trace.set_window_seconds(self.window_s, 1e-3);
                }

                ui.separator();

                ui.checkbox(&mut self.use_sine_input, "Sine input");
                ui.add_enabled_ui(!self.use_sine_input, |ui| {
                    ui.label("Duty q");
                    ui.add(egui::Slider::new(&mut self.duty_q, -1.0..=1.0).suffix(""));
                    ui.label("Duty d");
                    ui.add(egui::Slider::new(&mut self.duty_d, -1.0..=1.0).suffix(""));
                });
                ui.add_enabled_ui(self.use_sine_input, |ui| {
                    ui.label("Sine period");
                    ui.add(egui::Slider::new(&mut self.sine_period_s, 0.01..=10.0).suffix(" s"));
                });
            });
        });

        egui::CentralPanel::default().show(ctx, |ui| {
            ui.columns(2, |cols| {
                // =========================
                // Left column: battery & currents
                // =========================
                cols[0].heading("Battery & Currents");

                // Define custom Y axes: left = currents (A), right = SoC (0..1 via formatter)
                let y_axes_left = vec![
                    AxisHints::new_y()
                        .label("Current (A)"),
                    AxisHints::new_y()
                        .label("SoC (0–1)")
                        // Undo SOC_SCALE in tick labels so users see 0..1
                        .formatter(|mark, _range| format!("{:.2}", mark.value / SOC_SCALE)),
                ];

                Plot::new("left_plot")
                    .legend(Legend::default())
                    .allow_scroll(false)
                    .y_axis_min_width(48.0)
                    .x_axis_label("Time (s)")
                    .y_axis_label("Current / SoC")
                    // Add our custom y-axes (visual axes share transform; we scale data per-series)
                    .custom_y_axes(y_axes_left)
                    .show(&mut cols[0], |plot_ui| {
                        // FIXED BOUNDS (no autoscaling):
                        // X is a sliding window [t - window_s, t]; Y is fixed by Y_LEFT_MIN/MAX
                        let x_min = (self.t - self.window_s*0.25).max(0.0);
                        let x_max = self.t.max(self.window_s * 0.1);
                        plot_ui.set_plot_bounds(PlotBounds::from_min_max([x_min, Y_LEFT_MIN], [x_max, Y_LEFT_MAX]));
                        // Some egui versions also expose set_auto_bounds; if available you can disable it:
                        // plot_ui.set_auto_bounds(egui::Vec2b::new(false, false));

                        // Currents (unscaled) -> left axis
                        plot_ui.line(Line::new("I_total (A)", Trace::line(&self.trace.i_total, &self.trace.t)).name("I_total (A)"));
                        plot_ui.line(Line::new("I_q (A)", Trace::line(&self.trace.i_q, &self.trace.t)).name("I_q (A)"));
                        plot_ui.line(Line::new("I_d (A)", Trace::line(&self.trace.i_d, &self.trace.t)).name("I_d (A)"));

                        // SoC (scaled) -> right axis (labels show real units via formatter above)
                        plot_ui.line(
                            Line::new("SoC (0–1)", Trace::line_scaled(&self.trace.soc, &self.trace.t, SOC_SCALE))
                        );

                        // Battery Voltage (scaled) -> right axis (labels show real units via formatter above)
                        plot_ui.line(
                            Line::new("V_batt (V)", Trace::line_scaled(&self.trace.batt_v, &self.trace.t, VOLT_SCALE))
                        );
                    });

                // =========================
                // Right column: motor torque & velocity
                // =========================
                cols[1].heading("Motor Dynamics");

                // Define custom Y axes: left = torque, right = ω (rad/s, scaled to fit)
                let y_axes_right = vec![
                    AxisHints::new_y()
                        .label("Torque (N·m)"),
                    AxisHints::new_y()
                        .label("ω_mech (rad/s)")
                        // Undo VEL_SCALE in tick labels so users see rad/s
                        .formatter(|mark, _range| format!("{:.0}", mark.value / VEL_SCALE)),
                ];

                Plot::new("right_plot")
                    .legend(Legend::default())
                    .allow_scroll(false)
                    .y_axis_min_width(48.0)
                    .x_axis_label("Time (s)")
                    .y_axis_label("Torque / Velocity")
                    .custom_y_axes(y_axes_right)
                    .show(&mut cols[1], |plot_ui| {
                        // FIXED BOUNDS (no autoscaling)
                        let x_min = (self.t - self.window_s).max(0.0);
                        let x_max = self.t.max(self.window_s * 0.1);
                        plot_ui.set_plot_bounds(PlotBounds::from_min_max([x_min, Y_RIGHT_MIN], [x_max, Y_RIGHT_MAX]));
                        // plot_ui.set_auto_bounds(egui::Vec2b::new(false, false)); // if available

                        // Torque (unscaled) -> left axis
                        plot_ui.line(Line::new("Torque (N·m)", Trace::line(&self.trace.torque, &self.trace.t)));

                        // ω (scaled) -> right axis
                        plot_ui.line(
                            Line::new("ω_mech (rad/s)", Trace::line_scaled(&self.trace.mech_vel, &self.trace.t, VEL_SCALE))
                        );
                    });
            });
        });

        // Request another frame to keep the plots live
        ctx.request_repaint_after(Duration::from_millis(10));
    }
}
