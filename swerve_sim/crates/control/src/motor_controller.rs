//! Motor Controller
//!
//! A configurable motor controller that supports multiple control modes
//! (duty cycle, current, velocity, position) with different commutation strategies.

use electrical::motor::MotorConstant;
use simcore::{ControlModel, Model, MotorInput, MotorState, SimContext, SimState};

use crate::commutation::{CommutationStrategy, FocCommutation};
use crate::pidf::{PidfConfig, PidfController};

/// Control mode for the motor controller
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum ControlMode {
    /// Direct duty cycle control (open-loop)
    #[default]
    DutyCycle,
    /// Closed-loop current (torque) control
    Current,
    /// Closed-loop velocity control with current inner loop
    Velocity,
    /// Closed-loop position control with velocity and current inner loops
    Position,
}

/// Configuration for a motor controller
#[derive(Debug, Clone)]
pub struct MotorControllerConfig {
    /// Control mode
    pub control_mode: ControlMode,
    /// Motor constants (for kt calculation)
    pub motor_constants: MotorConstant,
    /// Current controller configuration
    pub current_config: PidfConfig,
    /// Velocity controller configuration
    pub velocity_config: PidfConfig,
    /// Position controller configuration
    pub position_config: PidfConfig,
    /// Maximum allowed current (A)
    pub max_current: f64,
    /// Maximum motor velocity for velocity/position control output limiting (rad/s)
    pub max_velocity: f64,
}

impl MotorControllerConfig {
    /// Create a default configuration for a given motor
    pub fn new(motor: MotorConstant) -> Self {
        Self {
            control_mode: ControlMode::DutyCycle,
            motor_constants: motor,
            current_config: PidfConfig::pi(0.1, 1.0).with_limits(-1.0, 1.0),
            velocity_config: PidfConfig::pi(0.5, 0.1).with_limits(-100.0, 100.0),
            position_config: PidfConfig::p(5.0).with_limits(-100.0, 100.0),
            max_current: 60.0,
            max_velocity: 600.0, // ~6000 RPM
        }
    }

    /// Set the control mode
    pub fn with_mode(mut self, mode: ControlMode) -> Self {
        self.control_mode = mode;
        self
    }

    /// Set the current controller config
    pub fn with_current_controller(mut self, config: PidfConfig) -> Self {
        self.current_config = config;
        self
    }

    /// Set the velocity controller config
    pub fn with_velocity_controller(mut self, config: PidfConfig) -> Self {
        self.velocity_config = config;
        self
    }

    /// Set the position controller config
    pub fn with_position_controller(mut self, config: PidfConfig) -> Self {
        self.position_config = config;
        self
    }

    /// Set maximum current limit
    pub fn with_max_current(mut self, max_current: f64) -> Self {
        self.max_current = max_current;
        self
    }
}

/// Motor controller with state
pub struct MotorController {
    config: MotorControllerConfig,
    commutation: Box<dyn CommutationStrategy>,
    current_controller: PidfController,
    velocity_controller: PidfController,
    position_controller: PidfController,
    /// Current setpoint (units depend on control mode)
    setpoint: f64,
    /// Accumulated position estimate (for position control)
    position_estimate: f64,
    /// Whether position was set externally this frame (skip auto-integration)
    position_externally_set: bool,
    /// Torque constant derived from motor constants: kt = 1.5 * pole_pairs * flux_linkage
    kt: f64,
}

impl MotorController {
    /// Create a new motor controller with default FOC commutation
    pub fn new(config: MotorControllerConfig) -> Self {
        Self::with_commutation(config, Box::new(FocCommutation))
    }

    /// Create a new motor controller with a specific commutation strategy
    pub fn with_commutation(
        config: MotorControllerConfig,
        commutation: Box<dyn CommutationStrategy>,
    ) -> Self {
        let kt = 1.5 * (config.motor_constants.pole_pairs as f64) * config.motor_constants.flux_linkage;
        
        Self {
            current_controller: PidfController::new(config.current_config.clone()),
            velocity_controller: PidfController::new(config.velocity_config.clone()),
            position_controller: PidfController::new(config.position_config.clone()),
            config,
            commutation,
            setpoint: 0.0,
            position_estimate: 0.0,
            position_externally_set: false,
            kt,
        }
    }

    /// Set the setpoint (units depend on control mode)
    /// - DutyCycle: duty cycle (-1 to 1)
    /// - Current: amps
    /// - Velocity: rad/s
    /// - Position: radians
    pub fn set_setpoint(&mut self, setpoint: f64) {
        self.setpoint = setpoint;
    }

    /// Get the current setpoint
    pub fn setpoint(&self) -> f64 {
        self.setpoint
    }

    /// Get the torque constant
    pub fn kt(&self) -> f64 {
        self.kt
    }

    /// Set position estimate (for syncing with external encoder)
    pub fn set_position(&mut self, position: f64) {
        self.position_estimate = position;
        self.position_externally_set = true;
    }

    /// Get current position estimate
    pub fn position(&self) -> f64 {
        self.position_estimate
    }

    /// Update the controller and compute motor input
    pub fn update(&mut self, motor_state: &MotorState, dt: f64) -> MotorInput {
        // Update position estimate from velocity (unless set externally this frame)
        if self.position_externally_set {
            self.position_externally_set = false; // Reset for next frame
        } else {
            self.position_estimate += motor_state.mechanical_velocity * dt;
        }

        // Compute electrical angle for commutation
        let electrical_angle = self.position_estimate * (self.config.motor_constants.pole_pairs as f64);

        // Cascade through control loops based on mode
        let duty = match self.config.control_mode {
            ControlMode::DutyCycle => {
                self.setpoint.clamp(-1.0, 1.0)
            }
            ControlMode::Current => {
                let target_current = self.setpoint.clamp(-self.config.max_current, self.config.max_current);
                self.current_controller.set_setpoint(target_current);
                self.current_controller.update(motor_state.current_q, dt)
            }
            ControlMode::Velocity => {
                // Velocity loop outputs duty directly (bypasses current loop for stability)
                // The velocity controller should be tuned to output duty cycle values
                self.velocity_controller.set_setpoint(self.setpoint);
                self.velocity_controller.update(motor_state.mechanical_velocity, dt)
            }
            ControlMode::Position => {
                // Position loop outputs target velocity
                self.position_controller.set_setpoint(self.setpoint);
                let target_velocity = self.position_controller.update(self.position_estimate, dt);
                let target_velocity = target_velocity.clamp(-self.config.max_velocity, self.config.max_velocity);
                
                // Velocity loop outputs duty directly (bypasses current loop for stability)
                self.velocity_controller.set_setpoint(target_velocity);
                self.velocity_controller.update(motor_state.mechanical_velocity, dt)
            }
        };

        // Apply commutation
        let comm_output = self.commutation.compute(duty, electrical_angle);

        MotorInput {
            duty_cycle_q: comm_output.duty_q.clamp(-1.0, 1.0),
            duty_cycle_d: comm_output.duty_d.clamp(-1.0, 1.0),
        }
    }

    /// Reset all controller states
    pub fn reset(&mut self) {
        self.current_controller.reset();
        self.velocity_controller.reset();
        self.position_controller.reset();
        self.position_estimate = 0.0;
        self.setpoint = 0.0;
    }

    /// Get a reference to the current configuration
    pub fn config(&self) -> &MotorControllerConfig {
        &self.config
    }

    /// Get the commutation strategy's average efficiency
    pub fn commutation_efficiency(&self) -> f64 {
        self.commutation.average_efficiency()
    }
}

/// A bank of motor controllers implementing the ControlModel trait
pub struct MotorControllerBank {
    /// Individual motor controllers
    pub controllers: Vec<MotorController>,
    /// Setpoints for each motor
    pub setpoints: Vec<f64>,
}

impl MotorControllerBank {
    /// Create a new empty controller bank
    pub fn new() -> Self {
        Self {
            controllers: Vec::new(),
            setpoints: Vec::new(),
        }
    }

    /// Add a motor controller to the bank
    pub fn add_controller(&mut self, controller: MotorController) {
        self.controllers.push(controller);
        self.setpoints.push(0.0);
    }

    /// Set the setpoint for a specific motor
    pub fn set_setpoint(&mut self, index: usize, setpoint: f64) {
        if index < self.setpoints.len() {
            self.setpoints[index] = setpoint;
            self.controllers[index].set_setpoint(setpoint);
        }
    }

    /// Set setpoints for all motors
    pub fn set_all_setpoints(&mut self, setpoints: &[f64]) {
        for (i, &sp) in setpoints.iter().enumerate() {
            self.set_setpoint(i, sp);
        }
    }

    /// Get the number of controllers
    pub fn len(&self) -> usize {
        self.controllers.len()
    }

    /// Check if the bank is empty
    pub fn is_empty(&self) -> bool {
        self.controllers.is_empty()
    }
}

impl Default for MotorControllerBank {
    fn default() -> Self {
        Self::new()
    }
}

impl Model for MotorControllerBank {
    fn reset(&mut self) {
        for ctrl in &mut self.controllers {
            ctrl.reset();
        }
        for sp in &mut self.setpoints {
            *sp = 0.0;
        }
    }
}

impl ControlModel for MotorControllerBank {
    fn step_control(&mut self, ctx: SimContext, state: &mut SimState) {
        let dt = ctx.dt;
        
        // Ensure we have enough motor inputs
        while state.control_input.motor_inputs.len() < self.controllers.len() {
            state.control_input.motor_inputs.push(MotorInput {
                duty_cycle_q: 0.0,
                duty_cycle_d: 0.0,
            });
        }

        for (i, ctrl) in self.controllers.iter_mut().enumerate() {
            if i < state.true_state.motors.len() {
                let motor_input = ctrl.update(&state.true_state.motors[i], dt);
                state.control_input.motor_inputs[i] = motor_input;
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn test_motor() -> MotorConstant {
        MotorConstant::kraken_x60()
    }

    #[test]
    fn test_duty_cycle_mode_passthrough() {
        let config = MotorControllerConfig::new(test_motor())
            .with_mode(ControlMode::DutyCycle);
        let mut ctrl = MotorController::new(config);
        
        ctrl.set_setpoint(0.75);
        let motor_state = MotorState::default();
        let output = ctrl.update(&motor_state, 0.001);
        
        assert!((output.duty_cycle_q - 0.75).abs() < 1e-6);
        assert!((output.duty_cycle_d).abs() < 1e-6);
    }

    #[test]
    fn test_duty_cycle_clamped() {
        let config = MotorControllerConfig::new(test_motor())
            .with_mode(ControlMode::DutyCycle);
        let mut ctrl = MotorController::new(config);
        
        ctrl.set_setpoint(2.0);
        let motor_state = MotorState::default();
        let output = ctrl.update(&motor_state, 0.001);
        
        assert!((output.duty_cycle_q - 1.0).abs() < 1e-6);
    }

    #[test]
    fn test_kt_computed_from_motor_constants() {
        let motor = test_motor();
        let config = MotorControllerConfig::new(motor);
        let ctrl = MotorController::new(config);
        
        let expected_kt = 1.5 * (motor.pole_pairs as f64) * motor.flux_linkage;
        assert!((ctrl.kt() - expected_kt).abs() < 1e-9);
    }

    #[test]
    fn test_position_integrates() {
        let config = MotorControllerConfig::new(test_motor())
            .with_mode(ControlMode::DutyCycle);
        let mut ctrl = MotorController::new(config);
        
        let mut motor_state = MotorState::default();
        motor_state.mechanical_velocity = 10.0; // rad/s
        
        ctrl.update(&motor_state, 0.1);
        
        // Position should have integrated: 10.0 * 0.1 = 1.0 rad
        assert!((ctrl.position() - 1.0).abs() < 1e-9);
    }

    #[test]
    fn test_controller_bank_step() {
        let motor = test_motor();
        let config = MotorControllerConfig::new(motor)
            .with_mode(ControlMode::DutyCycle);
        
        let mut bank = MotorControllerBank::new();
        bank.add_controller(MotorController::new(config.clone()));
        bank.add_controller(MotorController::new(config));
        
        bank.set_setpoint(0, 0.5);
        bank.set_setpoint(1, -0.3);
        
        let mut state = SimState::default();
        state.true_state.motors = vec![MotorState::default(); 2];
        
        bank.step_control(SimContext { dt: 0.001, t: 0.0 }, &mut state);
        
        assert!((state.control_input.motor_inputs[0].duty_cycle_q - 0.5).abs() < 1e-6);
        assert!((state.control_input.motor_inputs[1].duty_cycle_q - (-0.3)).abs() < 1e-6);
    }
}
