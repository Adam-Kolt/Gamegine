//! Battery bindings with discharge analysis

use pyo3::prelude::*;
use numpy::ToPyArray;
use pyo3::types::PyDict;
use electrical::battery::{BatteryConstant, Peukert, RCBranch, default_ocv_from_soc, default_r0_from_soc};
use electrical::analysis::{simulate_battery_discharge, voltage_sag_analysis, effective_capacity_ah};

/// Python-accessible battery representation with analysis functions
#[pyclass]
#[derive(Clone)]
pub struct PyBattery {
    inner: BatteryConstant,
}

#[pymethods]
impl PyBattery {
    /// Create a standard FRC battery (12V, 11.2Ah)
    #[staticmethod]
    fn frc_standard() -> Self {
        PyBattery {
            inner: BatteryConstant::default(),
        }
    }

    /// Create a custom battery
    /// 
    /// Args:
    ///     capacity_ah: Rated capacity in amp-hours
    ///     r0_mid: Internal resistance at 50% SoC (ohms)
    ///     peukert_constant: Peukert exponent (typically 1.1-1.3)
    /// 
    /// Note: r0_mid is fixed at creation time. For custom R0-SoC curves,
    /// use the Rust API directly.
    #[staticmethod]
    #[pyo3(signature = (capacity_ah=11.2, peukert_constant=1.183))]
    fn custom(capacity_ah: f64, peukert_constant: f64) -> Self {
        // We can't capture r0_mid in a closure since BatteryConstant requires fn pointers.
        // Use the default resistance function. For custom resistance curves,
        // users should create batteries through the Rust API.
        let constants = BatteryConstant {
            peukert_constant: Peukert {
                constant: peukert_constant,
                reference_discharge_current: 0.9,
            },
            rated_capacity_ah: capacity_ah,
            open_circuit_voltage_function: default_ocv_from_soc,
            ohmic_resistance_function: |soc| default_r0_from_soc(soc, 0.008),
            fast_polarization_constants: RCBranch { resistance: 0.0027, capacitance: 741.0 },
            slow_polarization_constants: RCBranch { resistance: 0.0018, capacitance: 66667.0 },
        };
        PyBattery { inner: constants }
    }

    /// Get rated capacity in Ah
    fn capacity_ah(&self) -> f64 {
        self.inner.rated_capacity_ah
    }

    /// Get open circuit voltage at given state of charge (0.0-1.0)
    fn ocv_at_soc(&self, soc: f64) -> f64 {
        (self.inner.open_circuit_voltage_function)(soc)
    }

    /// Get internal resistance at given state of charge
    fn resistance_at_soc(&self, soc: f64) -> f64 {
        (self.inner.ohmic_resistance_function)(soc)
    }

    /// Calculate effective capacity at given discharge current
    /// 
    /// Accounts for Peukert effect: higher currents reduce effective capacity
    fn effective_capacity(&self, discharge_current: f64) -> f64 {
        effective_capacity_ah(&self.inner, discharge_current)
    }

    /// Simulate battery discharge at constant current
    /// 
    /// Runs entire simulation in Rust for performance.
    /// Returns a dict with numpy arrays: times, voltages, soc, power
    /// 
    /// Args:
    ///     current: Constant discharge current (A)
    ///     duration_s: Total simulation time (seconds)
    ///     dt: Time step (seconds), default 0.01
    fn simulate_discharge<'py>(&self, py: Python<'py>, current: f64, duration_s: f64, dt: Option<f64>) -> PyResult<Bound<'py, PyDict>> {
        let dt = dt.unwrap_or(0.01);
        let result = simulate_battery_discharge(&self.inner, current, duration_s, dt);
        
        let dict = PyDict::new_bound(py);
        dict.set_item("times", result.times.to_pyarray_bound(py))?;
        dict.set_item("voltages", result.voltages.to_pyarray_bound(py))?;
        dict.set_item("soc", result.soc.to_pyarray_bound(py))?;
        dict.set_item("power", result.power.to_pyarray_bound(py))?;
        
        Ok(dict)
    }

    /// Analyze voltage sag under peak load
    /// 
    /// Returns (min_voltage, soc_at_min_voltage)
    fn voltage_sag(&self, peak_current: f64) -> (f64, f64) {
        voltage_sag_analysis(&self.inner, peak_current)
    }

    fn __repr__(&self) -> String {
        format!(
            "Battery(capacity={:.1} Ah, OCV@100%={:.2} V)",
            self.inner.rated_capacity_ah,
            (self.inner.open_circuit_voltage_function)(1.0)
        )
    }
}

impl PyBattery {
    pub fn inner(&self) -> &BatteryConstant {
        &self.inner
    }
}
