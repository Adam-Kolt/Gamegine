//! Drivetrain and simulation bindings with batch execution

use pyo3::prelude::*;
use pyo3::types::PyDict;
use numpy::ToPyArray;

use simcore::{
    ActuatorInput, BatteryState, BodyState, MotorInput, MotorState, SimContext, SimState,
    TireState, TrueState, WheelState, SensorBus,
};
use mechanics::{SwerveDrivetrain, SwerveDrivetrainConfig};
use mechanics::tire::{TireManager, TireConstants};
use electrical::motor::{MotorBank, MotorConstant};
use simcore::{ElectricalModel, MechanicsModel, Model};

/// Python-accessible swerve drivetrain configuration
#[pyclass]
#[derive(Clone)]
pub struct PySwerveDrivetrain {
    config: SwerveDrivetrainConfig,
}

#[pymethods]
impl PySwerveDrivetrain {
    /// Create a new swerve drivetrain
    /// 
    /// Args:
    ///     mass: Robot mass (kg)
    ///     moment_of_inertia: Yaw moment of inertia (kg*m^2)
    ///     module_positions: List of [x, y] module positions (m), default is square
    #[new]
    #[pyo3(signature = (mass=50.0, moment_of_inertia=5.0, module_positions=None))]
    fn new(mass: f64, moment_of_inertia: f64, module_positions: Option<Vec<[f64; 2]>>) -> Self {
        let positions = module_positions.unwrap_or_else(|| {
            let half_side = 0.3;
            vec![
                [half_side, half_side],
                [half_side, -half_side],
                [-half_side, half_side],
                [-half_side, -half_side],
            ]
        });

        PySwerveDrivetrain {
            config: SwerveDrivetrainConfig {
                module_positions: positions,
                mass,
                moment_of_inertia,
                wheel_inertia: 0.01,
                steer_inertia: 0.005,
            },
        }
    }

    /// Get robot mass
    fn mass(&self) -> f64 {
        self.config.mass
    }

    /// Get moment of inertia
    fn moment_of_inertia(&self) -> f64 {
        self.config.moment_of_inertia
    }

    /// Get number of modules
    fn num_modules(&self) -> usize {
        self.config.module_positions.len()
    }
}

/// Simulation result containing time series data
#[pyclass]
pub struct SimulationResult {
    times: Vec<f64>,
    positions_x: Vec<f64>,
    positions_y: Vec<f64>,
    headings: Vec<f64>,
    velocities_x: Vec<f64>,
    velocities_y: Vec<f64>,
    angular_velocities: Vec<f64>,
    battery_voltages: Vec<f64>,
}

#[pymethods]
impl SimulationResult {
    /// Get all data as a dictionary of numpy arrays
    fn to_dict<'py>(&self, py: Python<'py>) -> PyResult<Bound<'py, PyDict>> {
        let dict = PyDict::new_bound(py);
        dict.set_item("times", self.times.to_pyarray_bound(py))?;
        dict.set_item("x", self.positions_x.to_pyarray_bound(py))?;
        dict.set_item("y", self.positions_y.to_pyarray_bound(py))?;
        dict.set_item("heading", self.headings.to_pyarray_bound(py))?;
        dict.set_item("vx", self.velocities_x.to_pyarray_bound(py))?;
        dict.set_item("vy", self.velocities_y.to_pyarray_bound(py))?;
        dict.set_item("omega", self.angular_velocities.to_pyarray_bound(py))?;
        dict.set_item("battery_voltage", self.battery_voltages.to_pyarray_bound(py))?;
        Ok(dict)
    }
    
    /// Get final position as (x, y, heading)
    fn final_pose(&self) -> (f64, f64, f64) {
        (
            *self.positions_x.last().unwrap_or(&0.0),
            *self.positions_y.last().unwrap_or(&0.0),
            *self.headings.last().unwrap_or(&0.0),
        )
    }
}

/// High-fidelity swerve simulation with batched execution
#[pyclass]
pub struct PySimulator {
    state: SimState,
    drivetrain: SwerveDrivetrain,
    tire_manager: TireManager,
    motor_bank: MotorBank,
    time: f64,
}

#[pymethods]
impl PySimulator {
    /// Create a new simulator
    /// 
    /// Args:
    ///     drivetrain: Drivetrain configuration
    #[new]
    fn new(drivetrain: &PySwerveDrivetrain) -> Self {
        let config = drivetrain.config.clone();
        let num_modules = config.module_positions.len();
        let mass = config.mass;

        // Initialize wheel states
        let wheel_states: Vec<WheelState> = (0..num_modules)
            .map(|_| WheelState {
                driving_angular_velocity: 0.0,
                wheel_radius: 0.05,
                turning_angular_velocity: 0.0,
                longitudinal_translational_velocity: 0.0,
                lateral_translational_velocity: 0.0,
                tire: TireState {
                    slip_angle: 0.0,
                    slip_ratio: 0.0,
                    longitudinal_force: 0.0,
                    lateral_force: 0.0,
                    tire_load: mass * 9.81 / num_modules as f64,
                },
                angle: 0.0,
            })
            .collect();

        // Initialize motors
        let mut motor_bank = MotorBank::default();
        let motors: Vec<MotorState> = (0..num_modules)
            .map(|_| {
                motor_bank.add_motor(MotorConstant::kraken_x60());
                MotorState::default()
            })
            .collect();

        // Initialize tire manager
        let mut tire_manager = TireManager::new();
        for _ in 0..num_modules {
            tire_manager.add_tire(TireConstants::default());
        }

        let state = SimState {
            true_state: TrueState {
                wheel_states,
                body_state: BodyState::default(),
                motors,
                battery_state: BatteryState::default(),
            },
            control_input: ActuatorInput {
                motor_inputs: (0..num_modules)
                    .map(|_| MotorInput {
                        duty_cycle_q: 0.0,
                        duty_cycle_d: 0.0,
                    })
                    .collect(),
            },
            sensor_bus: SensorBus::default(),
        };

        PySimulator {
            state,
            drivetrain: SwerveDrivetrain::new(config),
            tire_manager,
            motor_bank,
            time: 0.0,
        }
    }

    /// Run simulation for specified duration - all steps executed in Rust
    /// 
    /// This is the primary API for batch simulation. All physics steps
    /// run in Rust without Python callbacks for maximum performance.
    /// 
    /// Args:
    ///     duration: Total simulation time (seconds)
    ///     dt: Time step (seconds)
    ///     duty_cycles: List of duty cycles for each module (0-1)
    ///     steer_angles: List of steering angles for each module (radians)
    /// 
    /// Returns:
    ///     SimulationResult with all time series data
    #[pyo3(signature = (duration, dt=0.001, duty_cycles=None, steer_angles=None))]
    fn run(
        &mut self,
        duration: f64,
        dt: f64,
        duty_cycles: Option<Vec<f64>>,
        steer_angles: Option<Vec<f64>>,
    ) -> SimulationResult {
        let num_modules = self.drivetrain.config.module_positions.len();
        
        // Set control inputs
        let duty = duty_cycles.unwrap_or_else(|| vec![0.0; num_modules]);
        let steers = steer_angles.unwrap_or_else(|| vec![0.0; num_modules]);
        
        for (i, &d) in duty.iter().enumerate() {
            if i < self.state.control_input.motor_inputs.len() {
                self.state.control_input.motor_inputs[i].duty_cycle_q = d;
            }
        }
        
        for (i, &s) in steers.iter().enumerate() {
            if i < self.state.true_state.wheel_states.len() {
                self.state.true_state.wheel_states[i].angle = s;
            }
        }

        // Pre-allocate result vectors
        let n_steps = (duration / dt).ceil() as usize;
        let mut times = Vec::with_capacity(n_steps);
        let mut positions_x = Vec::with_capacity(n_steps);
        let mut positions_y = Vec::with_capacity(n_steps);
        let mut headings = Vec::with_capacity(n_steps);
        let mut velocities_x = Vec::with_capacity(n_steps);
        let mut velocities_y = Vec::with_capacity(n_steps);
        let mut angular_velocities = Vec::with_capacity(n_steps);
        let mut battery_voltages = Vec::with_capacity(n_steps);

        // Run simulation loop entirely in Rust
        let end_time = self.time + duration;
        while self.time < end_time {
            // Record state
            times.push(self.time);
            positions_x.push(self.state.true_state.body_state.position[0]);
            positions_y.push(self.state.true_state.body_state.position[1]);
            headings.push(self.state.true_state.body_state.orientation[2]);
            velocities_x.push(self.state.true_state.body_state.velocity[0]);
            velocities_y.push(self.state.true_state.body_state.velocity[1]);
            angular_velocities.push(self.state.true_state.body_state.angular_velocity[2]);
            battery_voltages.push(self.state.true_state.battery_state.voltage);

            // Step simulation
            let ctx = SimContext { dt, t: self.time };
            self.motor_bank.step_electrical(ctx, &mut self.state);
            self.tire_manager.step_physics(ctx, &mut self.state);
            self.drivetrain.step_physics(ctx, &mut self.state);

            self.time += dt;
        }

        SimulationResult {
            times,
            positions_x,
            positions_y,
            headings,
            velocities_x,
            velocities_y,
            angular_velocities,
            battery_voltages,
        }
    }

    /// Get current simulation time
    fn time(&self) -> f64 {
        self.time
    }

    /// Get current position as (x, y, heading)
    fn pose(&self) -> (f64, f64, f64) {
        (
            self.state.true_state.body_state.position[0],
            self.state.true_state.body_state.position[1],
            self.state.true_state.body_state.orientation[2],
        )
    }

    /// Get current velocity as (vx, vy, omega)
    fn velocity(&self) -> (f64, f64, f64) {
        (
            self.state.true_state.body_state.velocity[0],
            self.state.true_state.body_state.velocity[1],
            self.state.true_state.body_state.angular_velocity[2],
        )
    }

    /// Reset simulation to initial state
    fn reset(&mut self) {
        self.time = 0.0;
        self.drivetrain.reset();
        self.tire_manager.reset();
        self.motor_bank.reset();

        let num_modules = self.drivetrain.config.module_positions.len();
        let mass = self.drivetrain.config.mass;

        self.state.true_state.body_state = BodyState::default();

        for wheel in &mut self.state.true_state.wheel_states {
            wheel.driving_angular_velocity = 0.0;
            wheel.longitudinal_translational_velocity = 0.0;
            wheel.lateral_translational_velocity = 0.0;
            wheel.tire.slip_angle = 0.0;
            wheel.tire.slip_ratio = 0.0;
            wheel.tire.longitudinal_force = 0.0;
            wheel.tire.lateral_force = 0.0;
            wheel.tire.tire_load = mass * 9.81 / num_modules as f64;
        }

        for motor in &mut self.state.true_state.motors {
            *motor = MotorState::default();
        }

        self.state.true_state.battery_state = BatteryState::default();
    }
}
