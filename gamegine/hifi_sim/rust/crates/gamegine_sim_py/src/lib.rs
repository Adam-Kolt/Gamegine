//! Gamegine High-Fidelity Simulation Python Bindings
//!
//! This module provides Python bindings for the Gamegine simulation library,
//! designed to maximize Rust performance by:
//! - Batching simulation steps to minimize Python↔Rust crossings
//! - Returning numpy arrays for vectorized analysis
//! - Using lazy state access (only copy to Python when requested)

use pyo3::prelude::*;
use pyo3::types::PyDict;
use numpy::{PyArray1, ToPyArray};

mod motor;
mod battery;
mod drivetrain;
mod mechanism;

pub use motor::*;
pub use battery::*;
pub use drivetrain::*;

/// Python module for Gamegine high-fidelity simulation
#[pymodule]
fn gamegine_sim_py(m: &Bound<'_, PyModule>) -> PyResult<()> {
    // Motor classes and analysis
    m.add_class::<motor::PyMotor>()?;
    
    // Battery classes and analysis
    m.add_class::<battery::PyBattery>()?;
    
    // Drivetrain simulation
    m.add_class::<drivetrain::PySwerveDrivetrain>()?;
    m.add_class::<drivetrain::PySimulator>()?;
    
    // General mechanism simulation
    m.add_class::<mechanism::PyLinkConfig>()?;
    m.add_class::<mechanism::PyMechanismSimulator>()?;
    m.add_class::<mechanism::MechanismResult>()?;
    
    Ok(())
}
