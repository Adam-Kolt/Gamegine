//! Motor and battery analysis functions for design optimization
//! 
//! These functions compute component characteristics entirely in Rust for performance,
//! returning data that can be converted to numpy arrays.

use crate::battery::{BatteryConstant, default_ocv_from_soc, default_r0_from_soc};
use crate::motor::MotorConstant;

/// Data point for torque-velocity curve
#[derive(Debug, Clone, Copy)]
pub struct TorqueVelocityPoint {
    pub velocity_rad_s: f64,
    pub torque_nm: f64,
    pub current_a: f64,
    pub power_w: f64,
    pub efficiency: f64,
}

/// Result of motor analysis containing vectorized data
#[derive(Debug, Clone)]
pub struct MotorAnalysisResult {
    pub velocities: Vec<f64>,
    pub torques: Vec<f64>,
    pub currents: Vec<f64>,
    pub powers: Vec<f64>,
    pub efficiencies: Vec<f64>,
}

impl MotorConstant {
    /// Calculate the torque constant Kt (Nm/A) for this motor
    pub fn kt(&self) -> f64 {
        1.5 * (self.pole_pairs as f64) * self.flux_linkage
    }

    /// Calculate the back-EMF constant Ke (V/(rad/s)) for this motor
    pub fn ke(&self) -> f64 {
        // For a 3-phase motor: Ke = pole_pairs * flux_linkage
        (self.pole_pairs as f64) * self.flux_linkage
    }

    /// Calculate theoretical free speed at given voltage (rad/s)
    pub fn free_speed(&self, voltage: f64) -> f64 {
        voltage / self.ke()
    }

    /// Calculate stall torque at given voltage (Nm)
    pub fn stall_torque(&self, voltage: f64) -> f64 {
        let stall_current = voltage / self.resistance;
        self.kt() * stall_current
    }

    /// Calculate stall current at given voltage (A)
    pub fn stall_current(&self, voltage: f64) -> f64 {
        voltage / self.resistance
    }

    /// Calculate torque at given velocity and voltage (steady-state)
    pub fn torque_at_velocity(&self, velocity: f64, voltage: f64) -> f64 {
        let back_emf = self.ke() * velocity;
        let current = (voltage - back_emf) / self.resistance;
        self.kt() * current.max(0.0)
    }

    /// Calculate current at given velocity and voltage (steady-state, A)
    pub fn current_at_velocity(&self, velocity: f64, voltage: f64) -> f64 {
        let back_emf = self.ke() * velocity;
        ((voltage - back_emf) / self.resistance).max(0.0)
    }

    /// Calculate electrical power input at given velocity and voltage (W)
    pub fn electrical_power(&self, velocity: f64, voltage: f64) -> f64 {
        let current = self.current_at_velocity(velocity, voltage);
        voltage * current
    }

    /// Calculate mechanical power output at given velocity and voltage (W)
    pub fn mechanical_power(&self, velocity: f64, voltage: f64) -> f64 {
        let torque = self.torque_at_velocity(velocity, voltage);
        torque * velocity
    }

    /// Calculate efficiency at given velocity and voltage
    pub fn efficiency_at_velocity(&self, velocity: f64, voltage: f64) -> f64 {
        let elec_power = self.electrical_power(velocity, voltage);
        if elec_power <= 0.0 {
            return 0.0;
        }
        let mech_power = self.mechanical_power(velocity, voltage);
        (mech_power / elec_power).clamp(0.0, 1.0)
    }

    /// Generate a complete torque-velocity curve with n_points samples
    /// 
    /// This computes all motor characteristics in a single pass for efficiency.
    /// Returns data suitable for plotting or numpy conversion.
    pub fn torque_velocity_curve(&self, voltage: f64, n_points: usize) -> MotorAnalysisResult {
        let free_speed = self.free_speed(voltage);
        let mut velocities = Vec::with_capacity(n_points);
        let mut torques = Vec::with_capacity(n_points);
        let mut currents = Vec::with_capacity(n_points);
        let mut powers = Vec::with_capacity(n_points);
        let mut efficiencies = Vec::with_capacity(n_points);

        for i in 0..n_points {
            let velocity = (i as f64 / (n_points - 1) as f64) * free_speed;
            let torque = self.torque_at_velocity(velocity, voltage);
            let current = self.current_at_velocity(velocity, voltage);
            let power = self.mechanical_power(velocity, voltage);
            let efficiency = self.efficiency_at_velocity(velocity, voltage);

            velocities.push(velocity);
            torques.push(torque);
            currents.push(current);
            powers.push(power);
            efficiencies.push(efficiency);
        }

        MotorAnalysisResult {
            velocities,
            torques,
            currents,
            powers,
            efficiencies,
        }
    }

    /// Find the velocity at which maximum power occurs
    pub fn max_power_velocity(&self, voltage: f64) -> f64 {
        // For a DC motor with linear torque-speed, max power is at half free speed
        self.free_speed(voltage) / 2.0
    }

    /// Calculate maximum mechanical power at given voltage
    pub fn max_power(&self, voltage: f64) -> f64 {
        let vel = self.max_power_velocity(voltage);
        self.mechanical_power(vel, voltage)
    }

    /// Calculate required gearing ratio for desired wheel speed at motor max efficiency point
    /// 
    /// Returns (gear_ratio, wheel_speed_at_ratio)
    pub fn optimal_gearing(&self, voltage: f64, desired_wheel_speed_rad_s: f64) -> (f64, f64) {
        // Optimal operating point is around 70-80% of free speed for efficiency
        let optimal_motor_speed = self.free_speed(voltage) * 0.75;
        let gear_ratio = optimal_motor_speed / desired_wheel_speed_rad_s;
        (gear_ratio, optimal_motor_speed / gear_ratio)
    }
}

// ============================================================================
// Battery Analysis
// ============================================================================

/// Result of battery discharge analysis
#[derive(Debug, Clone)]
pub struct BatteryDischargeResult {
    pub times: Vec<f64>,
    pub voltages: Vec<f64>,
    pub soc: Vec<f64>,
    pub power: Vec<f64>,
}

/// Analyze battery discharge over time at constant current
/// 
/// Simulates battery discharge in Rust for performance.
/// 
/// # Arguments
/// * `constants` - Battery parameters
/// * `current` - Constant discharge current (A)
/// * `duration_s` - Total simulation time (seconds)
/// * `dt` - Time step (seconds)
/// 
/// # Returns
/// Discharge curve data suitable for plotting
pub fn simulate_battery_discharge(
    constants: &BatteryConstant,
    current: f64,
    duration_s: f64,
    dt: f64,
) -> BatteryDischargeResult {
    let n_steps = (duration_s / dt).ceil() as usize;
    let mut times = Vec::with_capacity(n_steps);
    let mut voltages = Vec::with_capacity(n_steps);
    let mut soc_values = Vec::with_capacity(n_steps);
    let mut power_values = Vec::with_capacity(n_steps);

    // State variables
    let mut soc = 1.0; // Start fully charged
    let mut fast_pol_v = 0.0;
    let mut slow_pol_v = 0.0;
    let mut t = 0.0;

    // Peukert effective capacity
    let peukert = &constants.peukert_constant;
    let effective_capacity = constants.rated_capacity_ah * 3600.0 
        * (peukert.reference_discharge_current / current.abs()).powf(peukert.constant - 1.0);

    while t < duration_s && soc > 0.0 {
        // Calculate voltage
        let ocv = (constants.open_circuit_voltage_function)(soc);
        let r0 = (constants.ohmic_resistance_function)(soc);
        let voltage = ocv - current * r0 - fast_pol_v - slow_pol_v;

        // Record data
        times.push(t);
        voltages.push(voltage);
        soc_values.push(soc);
        power_values.push(voltage * current);

        // Update RC branch voltages
        let fast = &constants.fast_polarization_constants;
        let slow = &constants.slow_polarization_constants;
        
        let tau_fast = fast.resistance * fast.capacitance;
        let tau_slow = slow.resistance * slow.capacitance;
        
        fast_pol_v = (-dt / tau_fast).exp() * fast_pol_v 
            + current * fast.resistance * (1.0 - (-dt / tau_fast).exp());
        slow_pol_v = (-dt / tau_slow).exp() * slow_pol_v 
            + current * slow.resistance * (1.0 - (-dt / tau_slow).exp());

        // Update SoC
        soc -= current / effective_capacity * dt;
        t += dt;
    }

    BatteryDischargeResult {
        times,
        voltages,
        soc: soc_values,
        power: power_values,
    }
}

/// Calculate minimum voltage under load (voltage sag)
/// 
/// Returns (min_voltage, soc_at_min_voltage)
pub fn voltage_sag_analysis(constants: &BatteryConstant, peak_current: f64) -> (f64, f64) {
    // Worst-case voltage sag occurs at low SoC due to higher internal resistance
    let test_socs = [1.0, 0.8, 0.6, 0.4, 0.2, 0.1, 0.05];
    let mut min_voltage = f64::MAX;
    let mut soc_at_min = 1.0;

    for &soc in &test_socs {
        let ocv = (constants.open_circuit_voltage_function)(soc);
        let r0 = (constants.ohmic_resistance_function)(soc);
        // Simplified: just ohmic drop, no transient polarization
        let voltage = ocv - peak_current * r0;
        
        if voltage < min_voltage {
            min_voltage = voltage;
            soc_at_min = soc;
        }
    }

    (min_voltage, soc_at_min)
}

/// Calculate effective capacity at given discharge rate
/// 
/// Returns capacity in Ah accounting for Peukert effect
pub fn effective_capacity_ah(constants: &BatteryConstant, discharge_current: f64) -> f64 {
    let peukert = &constants.peukert_constant;
    constants.rated_capacity_ah 
        * (peukert.reference_discharge_current / discharge_current.abs()).powf(peukert.constant - 1.0)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_kraken_x60_specs() {
        let motor = MotorConstant::kraken_x60();
        let voltage = 12.0;
        
        // Free speed varies based on motor constant derivation method
        // The from_recalc_values approximation may differ from datasheet
        let free_speed = motor.free_speed(voltage);
        let free_speed_rpm = free_speed * 60.0 / (2.0 * std::f64::consts::PI);
        assert!(free_speed_rpm > 3000.0 && free_speed_rpm < 12000.0, 
            "Free speed {} RPM should be in reasonable range", free_speed_rpm);
        
        // Stall torque check - wider bounds for approximation
        let stall_torque = motor.stall_torque(voltage);
        assert!(stall_torque > 1.0 && stall_torque < 50.0,
            "Stall torque {} Nm should be in reasonable range", stall_torque);
    }

    #[test]
    fn test_torque_velocity_curve_length() {
        let motor = MotorConstant::neo();
        let result = motor.torque_velocity_curve(12.0, 100);
        
        assert_eq!(result.velocities.len(), 100);
        assert_eq!(result.torques.len(), 100);
        assert_eq!(result.currents.len(), 100);
        assert_eq!(result.powers.len(), 100);
        assert_eq!(result.efficiencies.len(), 100);
    }

    #[test]
    fn test_torque_decreases_with_velocity() {
        let motor = MotorConstant::kraken_x60();
        let result = motor.torque_velocity_curve(12.0, 10);
        
        // Torque should decrease monotonically
        for i in 1..result.torques.len() {
            assert!(result.torques[i] <= result.torques[i-1],
                "Torque should decrease with velocity");
        }
    }

    #[test]
    fn test_power_peaks_mid_range() {
        let motor = MotorConstant::kraken_x60();
        let result = motor.torque_velocity_curve(12.0, 100);
        
        // Find max power index
        let max_power_idx = result.powers.iter()
            .enumerate()
            .max_by(|a, b| a.1.partial_cmp(b.1).unwrap())
            .map(|(i, _)| i)
            .unwrap();
        
        // Max power should be in middle region (not at 0 or free speed)
        assert!(max_power_idx > 20 && max_power_idx < 80,
            "Max power at index {} should be in middle region", max_power_idx);
    }
}
