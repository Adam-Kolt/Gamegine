"""
High-Fidelity Simulation Module
===============================

This module provides high-performance physics simulation capabilities for robotics analysis.
It wraps the underlying Rust implementation (`gamegine_sim_py`) to offer:
- Motor torque-velocity curve generation
- Battery discharge simulation
- High-fidelity swerve drivetrain simulation

If the Rust extension is not compiled, this module provides placeholder functionality
and warns the user.
"""

import logging
import warnings
from typing import Optional, Dict, Tuple, List
import numpy as np

logger = logging.getLogger(__name__)

# Try to import the Rust extension
try:
    # Attempt absolute import first (if installed in integration)
    import gamegine_sim_py as _rust_sim
    _RUST_AVAILABLE = True
except ImportError:
    try:
        # Attempt relative import (if in the same package)
        from . import gamegine_sim_py as _rust_sim
        _RUST_AVAILABLE = True
    except ImportError:
        _RUST_AVAILABLE = False
        _rust_sim = None
        warnings.warn(
            "Gamegine high-fidelity simulation extension (gamegine_sim_py) not found. "
            "Simulation features will use simplified python fallbacks or raise errors."
        )

# Re-export key classes if available
if _RUST_AVAILABLE:
    PyMotor = _rust_sim.PyMotor
    PyBattery = _rust_sim.PyBattery
    PySwerveDrivetrain = _rust_sim.PySwerveDrivetrain
    PySimulator = _rust_sim.PySimulator
else:
    # Define placeholder classes for fallback
    class _Placeholder:
        def __init__(self, *args, **kwargs):
            raise RuntimeError(
                "Rust extension 'gamegine_sim_py' is required for this feature. "
                "Please compile the extension or install the full Gamegine package."
            )

    PyMotor = _Placeholder
    PyBattery = _Placeholder
    PySwerveDrivetrain = _Placeholder
    PySimulator = _Placeholder

def is_available() -> bool:
    """Check if the high-fidelity simulation engine is available."""
    return _RUST_AVAILABLE

class Motor:
    """
    Wrapper for high-fidelity motor models.
    """
    @staticmethod
    def kraken_x60():
        """Create a Kraken X60 motor model."""
        if not _RUST_AVAILABLE: 
            return None # TODO: Python fallback?
        return PyMotor.kraken_x60()

    @staticmethod
    def neo():
        """Create a NEO motor model."""
        if not _RUST_AVAILABLE: return None
        return PyMotor.neo()

    @staticmethod
    def kraken_x44():
        """Create a Kraken X44 motor model."""
        if not _RUST_AVAILABLE: return None
        return PyMotor.kraken_x44()

    @staticmethod
    def create(kv: float, kt: float, km: float):
        """Create a custom motor model."""
        if not _RUST_AVAILABLE: return None
        return PyMotor.from_recalc(kv, kt, km)

class Battery:
    """Wrapper for battery models."""
    @staticmethod
    def frc_standard():
        if not _RUST_AVAILABLE: return None
        return PyBattery.frc_standard()

class Drivetrain:
    """Wrapper for drivetrain simulation."""
    def __init__(self, mass: float, moi: float, module_positions: Optional[List[List[float]]] = None):
        self._inner = PySwerveDrivetrain(mass, moi, module_positions) if _RUST_AVAILABLE else None

    def simulate_construct(self):
        """Create a simulator instance."""
        if not self._inner: return None
        return PySimulator(self._inner)

if _RUST_AVAILABLE:
    PyLinkConfig = _rust_sim.PyLinkConfig
    PyMechanismSimulator = _rust_sim.PyMechanismSimulator
    MechanismResult = _rust_sim.MechanismResult
else:
    PyLinkConfig = _Placeholder
    PyMechanismSimulator = _Placeholder
    MechanismResult = _Placeholder

class LinkConfig:
    """Configuration for mechanical link (gearing, efficiency, friction)."""
    def __init__(self, gear_ratio: float = 1.0, radius: float = 0.0, 
                 efficiency: float = 1.0, friction_viscous: float = 0.0):
        if not _RUST_AVAILABLE:
            raise RuntimeError("Rust extension required")
        self._inner = PyLinkConfig(gear_ratio, radius, efficiency, friction_viscous)

class MechanismSimulator:
    """
    General-purpose mechanism simulator.
    
    Supports various mechanism types:
    - "vertical": Load against gravity (elevator)
    - "horizontal": Horizontal load (rollers, conveyors)
    - "flywheel": Pure rotational inertia (shooter)
    
    State persists between run() calls, allowing chained simulations
    with different inputs.
    """
    def __init__(self, motor, battery, link_config: LinkConfig, 
                 load_mass: float, load_type: str = "vertical"):
        if not _RUST_AVAILABLE:
            raise RuntimeError("Rust extension required for MechanismSimulator")
        self._inner = PyMechanismSimulator(
            motor, battery, link_config._inner, load_mass, load_type
        )
    
    def set_duty_cycle(self, duty: float):
        """Set motor duty cycle (-1.0 to 1.0)."""
        self._inner.set_duty_cycle(duty)
    
    def run(self, duration: float, dt: float = 0.001) -> dict:
        """
        Run simulation for specified duration.
        
        State persists between calls - can chain multiple runs with
        different inputs.
        
        Returns dict with numpy arrays: times, position, velocity, 
        current, torque, voltage, soc
        """
        result = self._inner.run(duration, dt)
        return result.to_dict()
    
    def reset(self):
        """Reset simulation to initial state."""
        self._inner.reset()
    
    @property
    def position(self) -> float:
        """Current position (m for linear, rad for rotational)."""
        return self._inner.position()
    
    @property
    def velocity(self) -> float:
        """Current velocity."""
        return self._inner.velocity()
    
    @property
    def time(self) -> float:
        """Current simulation time."""
        return self._inner.time()

