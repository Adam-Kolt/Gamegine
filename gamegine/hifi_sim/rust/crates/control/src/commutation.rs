//! Motor Commutation Strategies
//!
//! Different strategies for how a motor controller applies voltage to motor phases.
//! These model the behavioral differences between commutation methods without
//! modifying the underlying d-q motor model.

use serde::{Deserialize, Serialize};
use std::f64::consts::PI;

/// Result of commutation computation
#[derive(Debug, Clone, Copy)]
pub struct CommutationOutput {
    /// Duty cycle for q-axis (torque-producing)
    pub duty_q: f64,
    /// Duty cycle for d-axis (flux)
    pub duty_d: f64,
    /// Instantaneous efficiency factor (0-1)
    pub efficiency: f64,
}

/// Trait for commutation strategies
///
/// Commutation determines how a commanded torque/duty maps to motor drive signals.
/// Different strategies have different efficiency and torque ripple characteristics.
pub trait CommutationStrategy: Send + Sync {
    /// Compute the duty cycles given desired normalized torque and rotor position
    ///
    /// # Arguments
    /// * `desired_duty` - Commanded duty cycle (-1 to 1)
    /// * `electrical_angle` - Rotor electrical angle in radians
    ///
    /// # Returns
    /// CommutationOutput with duty cycles and efficiency
    fn compute(&self, desired_duty: f64, electrical_angle: f64) -> CommutationOutput;

    /// Average efficiency factor (0-1) representing torque output vs ideal FOC
    fn average_efficiency(&self) -> f64;

    /// Approximate torque ripple amplitude as fraction of mean torque
    fn torque_ripple_amplitude(&self) -> f64;

    /// Clone this strategy into a boxed trait object
    fn box_clone(&self) -> Box<dyn CommutationStrategy>;
}

impl Clone for Box<dyn CommutationStrategy> {
    fn clone(&self) -> Self {
        self.box_clone()
    }
}

// ============================================================================
// Field-Oriented Control (FOC)
// ============================================================================

/// Field-Oriented Control commutation
///
/// The ideal commutation strategy that perfectly aligns stator field with rotor.
/// All torque-producing current goes to q-axis, d-axis is zero.
#[derive(Debug, Clone, Copy, Default, Serialize, Deserialize)]
pub struct FocCommutation;

impl CommutationStrategy for FocCommutation {
    fn compute(&self, desired_duty: f64, _electrical_angle: f64) -> CommutationOutput {
        CommutationOutput {
            duty_q: desired_duty,
            duty_d: 0.0,
            efficiency: 1.0,
        }
    }

    fn average_efficiency(&self) -> f64 {
        1.0
    }

    fn torque_ripple_amplitude(&self) -> f64 {
        0.0
    }

    fn box_clone(&self) -> Box<dyn CommutationStrategy> {
        Box::new(*self)
    }
}

// ============================================================================
// Trapezoidal (6-Step) Commutation
// ============================================================================

/// Trapezoidal (6-step) commutation
///
/// The simplest commutation method. Only two phases are energized at any time,
/// switching every 60 electrical degrees based on Hall sensor feedback.
///
/// Characteristics:
/// - Higher torque ripple (~10-15%)
/// - Lower efficiency due to 30° average misalignment
/// - Simpler to implement in hardware
#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
pub struct TrapezoidalCommutation {
    /// Base efficiency (typically 0.90-0.95)
    pub base_efficiency: f64,
    /// Torque ripple amplitude as fraction of mean (typically 0.10-0.15)
    pub ripple_amplitude: f64,
}

impl Default for TrapezoidalCommutation {
    fn default() -> Self {
        Self {
            base_efficiency: 0.95,
            ripple_amplitude: 0.12,
        }
    }
}

impl CommutationStrategy for TrapezoidalCommutation {
    fn compute(&self, desired_duty: f64, electrical_angle: f64) -> CommutationOutput {
        // Normalize angle to [0, 2π)
        let angle = electrical_angle.rem_euclid(2.0 * PI);
        
        // Compute which 60° sector we're in (0-5)
        let sector = (angle / (PI / 3.0)).floor() as i32 % 6;
        
        // Within each sector, the torque efficiency varies sinusoidally
        // Maximum efficiency at sector center, minimum at edges
        let sector_angle = angle - (sector as f64) * (PI / 3.0);
        let sector_center = PI / 6.0; // 30° into each sector
        
        // Distance from sector center (0 to π/6)
        let deviation = (sector_angle - sector_center).abs();
        
        // Efficiency varies from base_efficiency at edges to 1.0 at center
        // Using cosine to smooth the variation
        let efficiency_variation = (deviation * 3.0).cos(); // Maps [0, π/6] to [0, π/2]
        let instant_efficiency = self.base_efficiency + 
            (1.0 - self.base_efficiency) * efficiency_variation;
        
        // Torque ripple: sinusoidal at 6x electrical frequency
        let ripple = 1.0 + self.ripple_amplitude * (6.0 * angle).sin();
        
        CommutationOutput {
            duty_q: desired_duty * ripple * instant_efficiency,
            duty_d: 0.0, // Trapezoidal doesn't do field weakening
            efficiency: instant_efficiency,
        }
    }

    fn average_efficiency(&self) -> f64 {
        self.base_efficiency
    }

    fn torque_ripple_amplitude(&self) -> f64 {
        self.ripple_amplitude
    }

    fn box_clone(&self) -> Box<dyn CommutationStrategy> {
        Box::new(*self)
    }
}

// ============================================================================
// Sinusoidal Commutation
// ============================================================================

/// Sinusoidal commutation
///
/// All three phases are modulated with sinusoidal currents, 120° apart.
/// Better than trapezoidal but not quite as good as FOC.
///
/// Characteristics:
/// - Lower torque ripple than trapezoidal (~3-5%)
/// - Higher efficiency than trapezoidal
/// - Requires more precise position sensing (usually encoder)
#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
pub struct SinusoidalCommutation {
    /// Base efficiency (typically 0.97-0.99)
    pub base_efficiency: f64,
    /// Torque ripple amplitude as fraction of mean (typically 0.03-0.05)
    pub ripple_amplitude: f64,
}

impl Default for SinusoidalCommutation {
    fn default() -> Self {
        Self {
            base_efficiency: 0.98,
            ripple_amplitude: 0.04,
        }
    }
}

impl CommutationStrategy for SinusoidalCommutation {
    fn compute(&self, desired_duty: f64, electrical_angle: f64) -> CommutationOutput {
        // Sinusoidal commutation has smoother torque production
        // Small ripple at 6x electrical frequency (from PWM harmonics)
        let ripple = 1.0 + self.ripple_amplitude * (6.0 * electrical_angle).sin();
        
        CommutationOutput {
            duty_q: desired_duty * self.base_efficiency * ripple,
            duty_d: 0.0,
            efficiency: self.base_efficiency,
        }
    }

    fn average_efficiency(&self) -> f64 {
        self.base_efficiency
    }

    fn torque_ripple_amplitude(&self) -> f64 {
        self.ripple_amplitude
    }

    fn box_clone(&self) -> Box<dyn CommutationStrategy> {
        Box::new(*self)
    }
}

// ============================================================================
// Tests
// ============================================================================

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_foc_full_duty_to_q_axis() {
        let foc = FocCommutation;
        let output = foc.compute(1.0, 0.0);
        
        assert!((output.duty_q - 1.0).abs() < 1e-9);
        assert!((output.duty_d).abs() < 1e-9);
        assert!((output.efficiency - 1.0).abs() < 1e-9);
    }

    #[test]
    fn test_foc_angle_independent() {
        let foc = FocCommutation;
        
        for angle in [0.0, PI / 4.0, PI / 2.0, PI, 3.0 * PI / 2.0] {
            let output = foc.compute(0.5, angle);
            assert!((output.duty_q - 0.5).abs() < 1e-9);
        }
    }

    #[test]
    fn test_trapezoidal_has_ripple() {
        let trap = TrapezoidalCommutation::default();
        
        // Sample at multiple angles
        let mut outputs = Vec::new();
        for i in 0..60 {
            let angle = (i as f64) * PI / 30.0; // 0 to 2π
            outputs.push(trap.compute(1.0, angle).duty_q);
        }
        
        // Find min and max
        let min = outputs.iter().cloned().fold(f64::INFINITY, f64::min);
        let max = outputs.iter().cloned().fold(f64::NEG_INFINITY, f64::max);
        
        // Should have noticeable variation
        assert!(max - min > 0.1);
    }

    #[test]
    fn test_trapezoidal_efficiency_less_than_one() {
        let trap = TrapezoidalCommutation::default();
        assert!(trap.average_efficiency() < 1.0);
    }

    #[test]
    fn test_sinusoidal_less_ripple_than_trapezoidal() {
        let trap = TrapezoidalCommutation::default();
        let sine = SinusoidalCommutation::default();
        
        assert!(sine.torque_ripple_amplitude() < trap.torque_ripple_amplitude());
    }

    #[test]
    fn test_sinusoidal_higher_efficiency_than_trapezoidal() {
        let trap = TrapezoidalCommutation::default();
        let sine = SinusoidalCommutation::default();
        
        assert!(sine.average_efficiency() > trap.average_efficiency());
    }

    #[test]
    fn test_negative_duty_preserved() {
        let foc = FocCommutation;
        let trap = TrapezoidalCommutation::default();
        
        assert!(foc.compute(-0.5, 0.0).duty_q < 0.0);
        // Trapezoidal at angle 0 should also preserve sign (ripple might modify magnitude)
        assert!(trap.compute(-0.5, 0.0).duty_q < 0.0);
    }
}
