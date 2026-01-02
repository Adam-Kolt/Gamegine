//! General-purpose mechanism simulation with batch execution
//!
//! Provides a flexible simulator that can model various mechanisms:
//! - Elevators (vertical against gravity)
//! - Arms (angle-dependent gravity)  
//! - Flywheels (pure rotational inertia)
//! - Horizontal systems (rollers, conveyors)
//!
//! Uses existing MechanicalLink for gear ratios, friction, and inertia coupling.
//! Uses steady-state motor model for numerical stability at reasonable time steps.

use pyo3::prelude::*;
use pyo3::types::PyDict;
use numpy::ToPyArray;

use mechanics::link::{MechanicalLink, LinkConfig, FrictionModel};

/// Load type for mechanism simulation
#[derive(Debug, Clone)]
pub enum LoadType {
    /// Vertical load against gravity (elevator)
    Vertical { mass_kg: f64 },
    /// Pure rotational load (flywheel)
    Flywheel { moment_of_inertia: f64 },
    /// Horizontal load (no gravity component)
    Horizontal { mass_kg: f64 },
}

impl LoadType {
    /// Compute external force/torque on load
    fn external_force(&self, _position: f64, _velocity: f64) -> f64 {
        match self {
            LoadType::Vertical { mass_kg } => -mass_kg * 9.81, // Gravity opposes upward motion
            LoadType::Flywheel { .. } => 0.0,
            LoadType::Horizontal { .. } => 0.0,
        }
    }
    
    /// Get load inertia (kg for linear, kg*m^2 for rotational)
    fn inertia(&self) -> f64 {
        match self {
            LoadType::Vertical { mass_kg } => *mass_kg,
            LoadType::Flywheel { moment_of_inertia } => *moment_of_inertia,
            LoadType::Horizontal { mass_kg } => *mass_kg,
        }
    }
}

/// Python-accessible link configuration
#[pyclass]
#[derive(Clone)]
pub struct PyLinkConfig {
    inner: LinkConfig,
}

#[pymethods]
impl PyLinkConfig {
    #[new]
    #[pyo3(signature = (gear_ratio=1.0, radius=0.0, efficiency=1.0, friction_viscous=0.0))]
    fn new(gear_ratio: f64, radius: f64, efficiency: f64, friction_viscous: f64) -> Self {
        let friction = if friction_viscous > 0.0 {
            FrictionModel::Viscous { damping: friction_viscous }
        } else {
            FrictionModel::None
        };
        
        PyLinkConfig {
            inner: LinkConfig {
                gear_ratio,
                radius,
                efficiency,
                load_inertia: 0.0, // Set from LoadType
                friction,
            }
        }
    }
}

/// Simulation result with time-series data
#[pyclass]
pub struct MechanismResult {
    times: Vec<f64>,
    positions: Vec<f64>,
    velocities: Vec<f64>,
    currents: Vec<f64>,
    torques: Vec<f64>,
    voltages: Vec<f64>,
    socs: Vec<f64>,
}

#[pymethods]
impl MechanismResult {
    /// Convert to dictionary of numpy arrays
    fn to_dict<'py>(&self, py: Python<'py>) -> PyResult<Bound<'py, PyDict>> {
        let dict = PyDict::new_bound(py);
        dict.set_item("times", self.times.to_pyarray_bound(py))?;
        dict.set_item("position", self.positions.to_pyarray_bound(py))?;
        dict.set_item("velocity", self.velocities.to_pyarray_bound(py))?;
        dict.set_item("current", self.currents.to_pyarray_bound(py))?;
        dict.set_item("torque", self.torques.to_pyarray_bound(py))?;
        dict.set_item("voltage", self.voltages.to_pyarray_bound(py))?;
        dict.set_item("soc", self.socs.to_pyarray_bound(py))?;
        Ok(dict)
    }
    
    /// Get final state values
    fn final_state(&self) -> (f64, f64, f64) {
        (
            *self.positions.last().unwrap_or(&0.0),
            *self.velocities.last().unwrap_or(&0.0),
            *self.currents.last().unwrap_or(&0.0),
        )
    }
}

/// General-purpose mechanism simulator
/// 
/// Composes Motor, Battery, and MechanicalLink to simulate any mechanism.
/// State persists between run() calls allowing Python to change inputs dynamically.
/// 
/// Uses steady-state motor model (V = IR + Ke*ω, T = Kt*I) which is appropriate
/// for mechanism dynamics analysis. This allows larger time steps (1ms) compared
/// to full electrical dynamics simulation (requires <100μs).
#[pyclass]
pub struct PyMechanismSimulator {
    // Simulation state
    time: f64,
    position: f64,  // Load position (linear m or rotational rad depending on config)
    velocity: f64,  // Load velocity
    
    // Motor constants (steady-state model)
    motor_kt: f64,         // Torque constant (Nm/A)
    motor_ke: f64,         // Back-EMF constant (V/(rad/s))
    motor_resistance: f64, // Winding resistance (Ohms)
    motor_inertia: f64,    // Rotor inertia (kg*m^2)
    
    // Battery model
    battery_capacity_ah: f64,
    battery_soc: f64,       // State of charge 0-1
    battery_voltage: f64,
    battery_r0: f64,        // Internal resistance
    
    // Mechanical model
    link: MechanicalLink,
    load_type: LoadType,
    
    // Current control input
    duty_cycle: f64,
}

#[pymethods]
impl PyMechanismSimulator {
    /// Create a new mechanism simulator
    /// 
    /// Args:
    ///     motor: Motor model (PyMotor)
    ///     battery: Battery model (PyBattery)
    ///     link_config: Mechanical link configuration
    ///     load_mass: Load mass in kg (for vertical/horizontal) or moment of inertia (for flywheel)
    ///     load_type: "vertical", "horizontal", or "flywheel"
    #[new]
    #[pyo3(signature = (motor, battery, link_config, load_mass, load_type="vertical"))]
    fn new(
        motor: &crate::motor::PyMotor,
        battery: &crate::battery::PyBattery,
        link_config: &PyLinkConfig,
        load_mass: f64,
        load_type: &str,
    ) -> Self {
        let load = match load_type {
            "vertical" => LoadType::Vertical { mass_kg: load_mass },
            "horizontal" => LoadType::Horizontal { mass_kg: load_mass },
            "flywheel" => LoadType::Flywheel { moment_of_inertia: load_mass },
            _ => LoadType::Vertical { mass_kg: load_mass },
        };
        
        // Create link config with load inertia
        let mut config = link_config.inner.clone();
        config.load_inertia = load.inertia();
        
        // Extract motor constants for steady-state model
        let motor_inner = motor.inner();
        let motor_kt = motor_inner.kt();
        let motor_ke = motor_inner.ke();
        let motor_resistance = motor_inner.resistance;
        
        // Extract battery parameters
        let battery_inner = battery.inner();
        let battery_capacity_ah = battery_inner.rated_capacity_ah;
        let battery_soc = 1.0; // Start fully charged
        let battery_voltage = (battery_inner.open_circuit_voltage_function)(battery_soc);
        let battery_r0 = (battery_inner.ohmic_resistance_function)(battery_soc);
        
        PyMechanismSimulator {
            time: 0.0,
            position: 0.0,
            velocity: 0.0,
            motor_kt,
            motor_ke,
            motor_resistance,
            motor_inertia: 0.0001, // Typical brushless motor rotor inertia
            battery_capacity_ah,
            battery_soc,
            battery_voltage,
            battery_r0,
            link: MechanicalLink::new(config),
            load_type: load,
            duty_cycle: 0.0,
        }
    }
    
    /// Set motor duty cycle (-1.0 to 1.0)
    fn set_duty_cycle(&mut self, duty: f64) {
        self.duty_cycle = duty.clamp(-1.0, 1.0);
    }
    
    /// Get current position (meters for linear output, radians for rotational)
    fn position(&self) -> f64 {
        self.position
    }
    
    /// Get current velocity
    fn velocity(&self) -> f64 {
        self.velocity
    }
    
    /// Get current time
    fn time(&self) -> f64 {
        self.time
    }
    
    /// Run simulation for specified duration
    /// 
    /// State persists between runs, allowing chained calls with different inputs.
    /// 
    /// Args:
    ///     duration: Simulation time (seconds)
    ///     dt: Time step (seconds), default 0.001 (1ms)
    /// 
    /// Returns:
    ///     MechanismResult with time-series data for this run segment
    #[pyo3(signature = (duration, dt=0.001))]
    fn run(&mut self, duration: f64, dt: f64) -> MechanismResult {
        let n_steps = (duration / dt).ceil() as usize;
        
        // Pre-allocate result vectors
        let mut times = Vec::with_capacity(n_steps);
        let mut positions = Vec::with_capacity(n_steps);
        let mut velocities = Vec::with_capacity(n_steps);
        let mut currents = Vec::with_capacity(n_steps);
        let mut torques = Vec::with_capacity(n_steps);
        let mut voltages = Vec::with_capacity(n_steps);
        let mut socs = Vec::with_capacity(n_steps);
        
        let end_time = self.time + duration;
        
        while self.time < end_time {
            // === Motor steady-state model ===
            // V_applied = duty_cycle * V_battery
            // V_applied = I * R + Ke * ω_motor
            // => I = (V_applied - Ke * ω_motor) / R
            // T_motor = Kt * I
            
            let motor_velocity = self.link.velocity_b_to_a(self.velocity);
            let v_applied = self.duty_cycle * self.battery_voltage;
            let back_emf = self.motor_ke * motor_velocity;
            let current = (v_applied - back_emf) / self.motor_resistance;
            let motor_torque = self.motor_kt * current;
            
            // Record state
            times.push(self.time);
            positions.push(self.position);
            velocities.push(self.velocity);
            currents.push(current);
            torques.push(motor_torque);
            voltages.push(self.battery_voltage);
            socs.push(self.battery_soc);
            
            // === Mechanical dynamics ===
            let external_force = self.load_type.external_force(self.position, self.velocity);
            
            let (acceleration, _net_force) = self.link.compute_load_acceleration(
                motor_torque,
                self.motor_inertia,
                self.velocity,
                external_force,
            );
            
            // Integrate mechanical state (semi-implicit Euler)
            self.velocity += acceleration * dt;
            self.position += self.velocity * dt;
            self.time += dt;
            
            // === Battery model ===
            // Simple Ah counting with voltage sag
            let amp_hours = current.abs() * (dt / 3600.0);
            self.battery_soc = (self.battery_soc - amp_hours / self.battery_capacity_ah).clamp(0.0, 1.0);
            
            // Voltage = OCV(SoC) - I * Rint
            // For simplicity, use linear approximation for OCV
            let ocv = 10.5 + 2.5 * self.battery_soc; // ~10.5V empty, ~13V full
            self.battery_r0 = 0.01 + 0.01 * (1.0 - self.battery_soc); // Resistance increases as depleted
            self.battery_voltage = ocv - current.abs() * self.battery_r0;
        }
        
        MechanismResult {
            times,
            positions,
            velocities,
            currents,
            torques,
            voltages,
            socs,
        }
    }
    
    /// Reset simulation to initial state
    fn reset(&mut self) {
        self.time = 0.0;
        self.position = 0.0;
        self.velocity = 0.0;
        self.duty_cycle = 0.0;
        self.battery_soc = 1.0;
        self.battery_voltage = 10.5 + 2.5 * self.battery_soc;
    }
    
    /// Set initial position
    fn set_position(&mut self, pos: f64) {
        self.position = pos;
    }
    
    /// Set initial velocity
    fn set_velocity(&mut self, vel: f64) {
        self.velocity = vel;
    }
}
