//! PIDF (Proportional-Integral-Derivative-Feedforward) Controller
//!
//! A configurable closed-loop controller with anti-windup and output saturation.

use serde::{Deserialize, Serialize};

/// Configuration for a PIDF controller
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PidfConfig {
    /// Proportional gain
    pub kp: f64,
    /// Integral gain
    pub ki: f64,
    /// Derivative gain
    pub kd: f64,
    /// Feedforward gain
    pub kf: f64,
    /// Integral zone: only accumulate integral when |error| < i_zone (None = always)
    pub i_zone: Option<f64>,
    /// Maximum integral accumulator magnitude (anti-windup)
    pub i_max: f64,
    /// Minimum output value
    pub output_min: f64,
    /// Maximum output value
    pub output_max: f64,
}

impl Default for PidfConfig {
    fn default() -> Self {
        Self {
            kp: 0.0,
            ki: 0.0,
            kd: 0.0,
            kf: 0.0,
            i_zone: None,
            i_max: f64::MAX,
            output_min: f64::NEG_INFINITY,
            output_max: f64::INFINITY,
        }
    }
}

impl PidfConfig {
    /// Create a P-only controller
    pub fn p(kp: f64) -> Self {
        Self { kp, ..Default::default() }
    }

    /// Create a PI controller
    pub fn pi(kp: f64, ki: f64) -> Self {
        Self { kp, ki, ..Default::default() }
    }

    /// Create a PID controller
    pub fn pid(kp: f64, ki: f64, kd: f64) -> Self {
        Self { kp, ki, kd, ..Default::default() }
    }

    /// Create a PIDF controller with all gains
    pub fn pidf(kp: f64, ki: f64, kd: f64, kf: f64) -> Self {
        Self { kp, ki, kd, kf, ..Default::default() }
    }

    /// Set output limits
    pub fn with_limits(mut self, min: f64, max: f64) -> Self {
        self.output_min = min;
        self.output_max = max;
        self
    }

    /// Set integral anti-windup limit
    pub fn with_i_max(mut self, i_max: f64) -> Self {
        self.i_max = i_max;
        self
    }

    /// Set integral zone
    pub fn with_i_zone(mut self, i_zone: f64) -> Self {
        self.i_zone = Some(i_zone);
        self
    }
}

/// PIDF Controller with state
#[derive(Debug, Clone)]
pub struct PidfController {
    config: PidfConfig,
    integral: f64,
    prev_measurement: Option<f64>,
    setpoint: f64,
}

impl PidfController {
    /// Create a new controller with the given configuration
    pub fn new(config: PidfConfig) -> Self {
        Self {
            config,
            integral: 0.0,
            prev_measurement: None,
            setpoint: 0.0,
        }
    }

    /// Set the target setpoint
    pub fn set_setpoint(&mut self, setpoint: f64) {
        self.setpoint = setpoint;
    }

    /// Get the current setpoint
    pub fn setpoint(&self) -> f64 {
        self.setpoint
    }

    /// Reset the controller state (integral and derivative)
    pub fn reset(&mut self) {
        self.integral = 0.0;
        self.prev_measurement = None;
    }

    /// Update the controller with a new measurement and return the control output
    ///
    /// Uses derivative-on-measurement to avoid derivative kick on setpoint changes.
    pub fn update(&mut self, measurement: f64, dt: f64) -> f64 {
        let error = self.setpoint - measurement;

        // Proportional term
        let p_term = self.config.kp * error;

        // Integral term with I-zone and anti-windup
        let in_i_zone = self.config.i_zone
            .map(|zone| error.abs() < zone)
            .unwrap_or(true);
        
        if in_i_zone && dt > 0.0 {
            self.integral += error * dt;
            // Anti-windup: clamp integral
            self.integral = self.integral.clamp(-self.config.i_max, self.config.i_max);
        }
        let i_term = self.config.ki * self.integral;

        // Derivative term (on measurement to avoid derivative kick)
        let d_term = if let Some(prev) = self.prev_measurement {
            if dt > 0.0 {
                // Negative because we're using derivative on measurement, not error
                -self.config.kd * (measurement - prev) / dt
            } else {
                0.0
            }
        } else {
            0.0
        };
        self.prev_measurement = Some(measurement);

        // Feedforward term
        let f_term = self.config.kf * self.setpoint;

        // Sum and clamp output
        let output = p_term + i_term + d_term + f_term;
        output.clamp(self.config.output_min, self.config.output_max)
    }

    /// Get the current integral accumulator value
    pub fn integral(&self) -> f64 {
        self.integral
    }

    /// Get a reference to the configuration
    pub fn config(&self) -> &PidfConfig {
        &self.config
    }

    /// Update the configuration
    pub fn set_config(&mut self, config: PidfConfig) {
        self.config = config;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_p_only_proportional_output() {
        let config = PidfConfig::p(2.0);
        let mut ctrl = PidfController::new(config);
        ctrl.set_setpoint(10.0);
        
        // With measurement=4, error=6, P output should be 12
        let output = ctrl.update(4.0, 0.01);
        assert!((output - 12.0).abs() < 1e-9);
    }

    #[test]
    fn test_pi_eliminates_steady_state_error() {
        // PI controller with reasonable gains for the simple test plant
        let config = PidfConfig::pi(1.0, 5.0);
        let mut ctrl = PidfController::new(config);
        ctrl.set_setpoint(10.0);
        
        // Simulate several steps with a simple first-order plant
        // Plant model: measurement += output * 0.01 (integrating plant)
        let mut measurement = 0.0;
        let dt = 0.01;
        for _ in 0..500 {
            let output = ctrl.update(measurement, dt);
            measurement += output * dt; // Simple integrating plant
        }
        
        // After integrating, should be close to setpoint
        assert!((measurement - 10.0).abs() < 1.0, "Expected ~10.0, got {}", measurement);
    }

    #[test]
    fn test_anti_windup() {
        let config = PidfConfig::pi(1.0, 10.0).with_i_max(5.0);
        let mut ctrl = PidfController::new(config);
        ctrl.set_setpoint(100.0);
        
        // Run with large error to try to wind up
        for _ in 0..100 {
            ctrl.update(0.0, 0.1);
        }
        
        // Integral should be clamped
        assert!(ctrl.integral().abs() <= 5.0);
    }

    #[test]
    fn test_output_saturation() {
        let config = PidfConfig::p(100.0).with_limits(-1.0, 1.0);
        let mut ctrl = PidfController::new(config);
        ctrl.set_setpoint(10.0);
        
        let output = ctrl.update(0.0, 0.01);
        assert!((output - 1.0).abs() < 1e-9); // Should be clamped to max
        
        ctrl.set_setpoint(-10.0);
        let output = ctrl.update(0.0, 0.01);
        assert!((output - (-1.0)).abs() < 1e-9); // Should be clamped to min
    }

    #[test]
    fn test_derivative_on_measurement_no_kick() {
        let config = PidfConfig::pid(0.0, 0.0, 1.0);
        let mut ctrl = PidfController::new(config);
        
        // First update - no derivative yet
        ctrl.set_setpoint(0.0);
        let output1 = ctrl.update(5.0, 0.01);
        assert!((output1).abs() < 1e-9); // No previous measurement
        
        // Change setpoint - should NOT cause derivative spike
        ctrl.set_setpoint(100.0);
        let output2 = ctrl.update(5.0, 0.01);
        // Measurement didn't change, so derivative should be ~0
        assert!(output2.abs() < 1e-9);
    }

    #[test]
    fn test_reset_clears_state() {
        let config = PidfConfig::pi(1.0, 1.0);
        let mut ctrl = PidfController::new(config);
        ctrl.set_setpoint(10.0);
        
        // Accumulate some integral
        for _ in 0..10 {
            ctrl.update(0.0, 0.1);
        }
        assert!(ctrl.integral() > 0.0);
        
        ctrl.reset();
        assert!((ctrl.integral()).abs() < 1e-9);
    }

    #[test]
    fn test_i_zone() {
        let config = PidfConfig::pi(0.0, 1.0).with_i_zone(5.0);
        let mut ctrl = PidfController::new(config);
        ctrl.set_setpoint(10.0);
        
        // Error = 10, larger than i_zone=5, should not integrate
        ctrl.update(0.0, 0.1);
        assert!((ctrl.integral()).abs() < 1e-9);
        
        // Error = 3, smaller than i_zone=5, should integrate
        ctrl.update(7.0, 0.1);
        assert!(ctrl.integral() > 0.0);
    }
}
