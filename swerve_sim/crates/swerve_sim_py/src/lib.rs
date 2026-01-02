use pyo3::prelude::*;
use pyo3::types::PyDict;

use simcore::{
    ActuatorInput, BatteryState, BodyState, MotorInput, MotorState, SimContext, SimState,
    TireState, TrueState, WheelState, SensorBus,
};
use mechanics::{SwerveDrivetrain, SwerveDrivetrainConfig};
use mechanics::tire::{TireManager, TireConstants};
use electrical::motor::{MotorBank, MotorConstant};

/// Python-accessible swerve drive simulator.
#[pyclass]
pub struct SwerveSim {
    state: SimState,
    drivetrain: SwerveDrivetrain,
    tire_manager: TireManager,
    motor_bank: MotorBank,
    time: f64,
}

#[pymethods]
impl SwerveSim {
    /// Create a new swerve simulator with the given configuration.
    /// 
    /// Args:
    ///     mass: Robot mass in kg.
    ///     moment_of_inertia: Moment of inertia about yaw axis in kg*m^2.
    ///     module_positions: List of [x, y] positions for each module in meters.
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

        let num_modules = positions.len();

        let config = SwerveDrivetrainConfig {
            module_positions: positions,
            mass,
            moment_of_inertia,
            wheel_inertia: 0.01,
            steer_inertia: 0.005,
        };

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

        // Initialize motors (one per module for drive)
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

        SwerveSim {
            state,
            drivetrain: SwerveDrivetrain::new(config),
            tire_manager,
            motor_bank,
            time: 0.0,
        }
    }

    /// Step the simulation forward by dt seconds.
    /// 
    /// Args:
    ///     dt: Time step in seconds.
    ///     motor_duty_cycles: List of (duty_cycle_q, duty_cycle_d) tuples for each motor.
    ///     steer_angles: Optional list of steering angles for each module in radians.
    #[pyo3(signature = (dt, motor_duty_cycles, steer_angles=None))]
    fn step(
        &mut self,
        dt: f64,
        motor_duty_cycles: Vec<(f64, f64)>,
        steer_angles: Option<Vec<f64>>,
    ) {
        // Update control inputs
        for (i, (dq, dd)) in motor_duty_cycles.iter().enumerate() {
            if i < self.state.control_input.motor_inputs.len() {
                self.state.control_input.motor_inputs[i].duty_cycle_q = *dq;
                self.state.control_input.motor_inputs[i].duty_cycle_d = *dd;
            }
        }

        // Update steering angles if provided
        if let Some(angles) = steer_angles {
            for (i, angle) in angles.iter().enumerate() {
                if i < self.state.true_state.wheel_states.len() {
                    self.state.true_state.wheel_states[i].angle = *angle;
                }
            }
        }

        let ctx = SimContext { dt, t: self.time };

        // Step each model in order:
        // 1. Electrical (motor currents and torques)
        use simcore::ElectricalModel;
        self.motor_bank.step_electrical(ctx, &mut self.state);

        // 2. Tire forces
        use simcore::MechanicsModel;
        self.tire_manager.step_physics(ctx, &mut self.state);

        // 3. Drivetrain dynamics
        self.drivetrain.step_physics(ctx, &mut self.state);

        self.time += dt;
    }

    /// Get the current simulation time.
    fn get_time(&self) -> f64 {
        self.time
    }

    /// Get the current robot position as [x, y, z].
    fn get_position(&self) -> [f64; 3] {
        self.state.true_state.body_state.position
    }

    /// Get the current robot velocity as [vx, vy, vz].
    fn get_velocity(&self) -> [f64; 3] {
        self.state.true_state.body_state.velocity
    }

    /// Get the current robot orientation as [roll, pitch, yaw].
    fn get_orientation(&self) -> [f64; 3] {
        self.state.true_state.body_state.orientation
    }

    /// Get the current battery voltage.
    fn get_battery_voltage(&self) -> f64 {
        self.state.true_state.battery_state.voltage
    }

    /// Get motor states as a list of dicts.
    fn get_motor_states(&self, py: Python<'_>) -> PyResult<PyObject> {
        let list = pyo3::types::PyList::empty_bound(py);
        for motor in &self.state.true_state.motors {
            let dict = PyDict::new_bound(py);
            dict.set_item("current_q", motor.current_q)?;
            dict.set_item("current_d", motor.current_d)?;
            dict.set_item("mechanical_velocity", motor.mechanical_velocity)?;
            dict.set_item("applied_torque", motor.applied_torque)?;
            list.append(dict)?;
        }
        Ok(list.into())
    }

    /// Get wheel states as a list of dicts.
    fn get_wheel_states(&self, py: Python<'_>) -> PyResult<PyObject> {
        let list = pyo3::types::PyList::empty_bound(py);
        for wheel in &self.state.true_state.wheel_states {
            let dict = PyDict::new_bound(py);
            dict.set_item("driving_angular_velocity", wheel.driving_angular_velocity)?;
            dict.set_item("angle", wheel.angle)?;
            dict.set_item("slip_angle", wheel.tire.slip_angle)?;
            dict.set_item("slip_ratio", wheel.tire.slip_ratio)?;
            dict.set_item("longitudinal_force", wheel.tire.longitudinal_force)?;
            dict.set_item("lateral_force", wheel.tire.lateral_force)?;
            list.append(dict)?;
        }
        Ok(list.into())
    }

    /// Reset the simulation to initial state.
    fn reset(&mut self) {
        use simcore::Model;
        
        self.time = 0.0;
        self.drivetrain.reset();
        self.tire_manager.reset();
        self.motor_bank.reset();

        // Reset state
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

/// Python module for the swerve drive simulator.
#[pymodule]
fn swerve_sim_py(m: &Bound<'_, PyModule>) -> PyResult<()> {
    m.add_class::<SwerveSim>()?;
    Ok(())
}
