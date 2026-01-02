//! Control systems for motor and robot control
//!
//! This crate provides:
//! - PIDF controllers for closed-loop control
//! - Commutation strategies (FOC, Trapezoidal, Sinusoidal)
//! - Motor controllers with multiple control modes

pub mod commutation;
pub mod motor_controller;
pub mod pidf;
pub mod swerve_ctrl;

pub use commutation::*;
pub use motor_controller::*;
pub use pidf::*;
