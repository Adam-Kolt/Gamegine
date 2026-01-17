//! Motor bindings with vectorized analysis APIs

use pyo3::prelude::*;
use pyo3::types::PyDict;
use numpy::ToPyArray;
use electrical::motor::MotorConstant;

/// Python-accessible motor representation with analysis functions
#[pyclass]
#[derive(Clone)]
pub struct PyMotor {
    inner: MotorConstant,
}

#[pymethods]
impl PyMotor {
    /// Create a Kraken X60 motor
    #[staticmethod]
    fn kraken_x60() -> Self {
        PyMotor {
            inner: MotorConstant::kraken_x60(),
        }
    }

    /// Create a NEO motor
    #[staticmethod]
    fn neo() -> Self {
        PyMotor {
            inner: MotorConstant::neo(),
        }
    }

    /// Create a Kraken X44 motor
    #[staticmethod]
    fn kraken_x44() -> Self {
        PyMotor {
            inner: MotorConstant::kraken_x44(),
        }
    }

    /// Create a custom motor from recalc values
    /// 
    /// Args:
    ///     kv: Motor velocity constant (RPM/V)
    ///     kt: Motor torque constant (Nm/A)
    ///     km: Motor constant (Nm/sqrt(W))
    #[staticmethod]
    fn from_recalc(kv: f64, kt: f64, km: f64) -> Self {
        PyMotor {
            inner: MotorConstant::from_recalc_values(kv, kt, km),
        }
    }

    /// Create a motor with full specification
    #[staticmethod]
    #[pyo3(signature = (pole_pairs=3, resistance=0.05, inductance_d=0.00001, inductance_q=0.00001, flux_linkage=0.01))]
    fn custom(
        pole_pairs: u32,
        resistance: f64,
        inductance_d: f64,
        inductance_q: f64,
        flux_linkage: f64,
    ) -> Self {
        PyMotor {
            inner: MotorConstant::new(pole_pairs, resistance, inductance_d, inductance_q, flux_linkage),
        }
    }

    /// Get torque constant Kt (Nm/A)
    fn kt(&self) -> f64 {
        self.inner.kt()
    }

    /// Get motor resistance (Ohms)
    fn resistance(&self) -> f64 {
        self.inner.resistance
    }

    /// Get back-EMF constant Ke (V/(rad/s))
    fn ke(&self) -> f64 {
        self.inner.ke()
    }

    /// Get free speed at given voltage (rad/s)
    fn free_speed(&self, voltage: f64) -> f64 {
        self.inner.free_speed(voltage)
    }

    /// Get free speed at given voltage (RPM)
    fn free_speed_rpm(&self, voltage: f64) -> f64 {
        self.inner.free_speed(voltage) * 60.0 / (2.0 * std::f64::consts::PI)
    }

    /// Get stall torque at given voltage (Nm)
    fn stall_torque(&self, voltage: f64) -> f64 {
        self.inner.stall_torque(voltage)
    }

    /// Get stall current at given voltage (A)
    fn stall_current(&self, voltage: f64) -> f64 {
        self.inner.stall_current(voltage)
    }

    /// Get maximum power at given voltage (W)
    fn max_power(&self, voltage: f64) -> f64 {
        self.inner.max_power(voltage)
    }

    /// Compute torque at given velocity and voltage
    fn torque_at_velocity(&self, velocity: f64, voltage: f64) -> f64 {
        self.inner.torque_at_velocity(velocity, voltage)
    }

    /// Compute efficiency at given velocity and voltage
    fn efficiency_at_velocity(&self, velocity: f64, voltage: f64) -> f64 {
        self.inner.efficiency_at_velocity(velocity, voltage)
    }

    /// Generate complete torque-velocity curve data
    /// 
    /// Computes all motor characteristics in a single Rust call for performance.
    /// Returns a dict with numpy arrays: velocities, torques, currents, powers, efficiencies
    /// 
    /// Args:
    ///     voltage: Supply voltage (V)
    ///     n_points: Number of sample points (default 100)
    #[pyo3(signature = (voltage, n_points=100))]
    fn torque_velocity_curve<'py>(&self, py: Python<'py>, voltage: f64, n_points: usize) -> PyResult<Bound<'py, PyDict>> {
        let result = self.inner.torque_velocity_curve(voltage, n_points);
        
        let dict = PyDict::new_bound(py);
        dict.set_item("velocities", result.velocities.to_pyarray_bound(py))?;
        dict.set_item("torques", result.torques.to_pyarray_bound(py))?;
        dict.set_item("currents", result.currents.to_pyarray_bound(py))?;
        dict.set_item("powers", result.powers.to_pyarray_bound(py))?;
        dict.set_item("efficiencies", result.efficiencies.to_pyarray_bound(py))?;
        
        Ok(dict)
    }

    /// Calculate optimal gearing for desired wheel speed
    /// 
    /// Returns (gear_ratio, actual_wheel_speed) tuple
    fn optimal_gearing(&self, voltage: f64, desired_wheel_speed: f64) -> (f64, f64) {
        self.inner.optimal_gearing(voltage, desired_wheel_speed)
    }

    fn __repr__(&self) -> String {
        format!(
            "Motor(kt={:.4} Nm/A, ke={:.4} V/(rad/s), R={:.4} Ω)",
            self.kt(),
            self.ke(),
            self.inner.resistance
        )
    }
}

impl PyMotor {
    pub fn inner(&self) -> &MotorConstant {
        &self.inner
    }
}
