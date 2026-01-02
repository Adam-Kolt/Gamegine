"""
Integration tests for the Gamegine High-Fidelity Simulation module.
"""
import pytest
import numpy as np
import sys
import os

# Ensure we can import the module
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from gamegine.hifi_sim import Motor, Battery, Drivetrain, is_available

@pytest.mark.skipif(not is_available(), reason="Rust extension not available")
class TestHiFiSim:
    def test_motor_creation(self):
        """Test creating motor models."""
        motor = Motor.kraken_x60()
        assert motor is not None
        
        # Test basic properties (approximate values)
        assert 500.0 < motor.free_speed_rpm(12.0) < 12000.0
        assert motor.stall_torque(12.0) > 5.0
        
        # Test custom motor
        custom = Motor.create(kv=500, kt=0.019, km=0.1)
        assert custom is not None

    def test_motor_analysis(self):
        """Test vectorized motor analysis."""
        motor = Motor.kraken_x60()
        
        # Generate curve
        curve = motor.torque_velocity_curve(12.0, 50)
        
        assert "velocities" in curve
        assert "torques" in curve
        assert "efficiencies" in curve
        
        velocities = curve["velocities"]
        torques = curve["torques"]
        
        assert len(velocities) == 50
        assert len(torques) == 50
        
        # Torque should decrease as velocity increases
        assert torques[0] > torques[-1]
        
        # Efficiency should be 0 at 0 speed and max speed
        assert curve["efficiencies"][0] == 0.0

    def test_battery_simulation(self):
        """Test battery discharge simulation."""
        battery = Battery.frc_standard()
        assert battery is not None
        
        # Simulate 1 second at 100A
        result = battery.simulate_discharge(current=100.0, duration_s=1.0)
        
        assert "times" in result
        assert "voltages" in result
        assert "soc" in result
        
        times = result["times"]
        voltages = result["voltages"]
        soc = result["soc"]
        
        assert len(times) > 0
        assert times[-1] >= 0.99 # close to 1.0s
        
        # Voltage should sag under load
        ocv = battery.ocv_at_soc(1.0)
        assert voltages[0] < ocv
        
        # SoC should decrease
        assert soc[-1] < soc[0]

    def test_drivetrain_simulation(self):
        """Test full drivetrain batch simulation."""
        drivetrain = Drivetrain(mass=50.0, moi=5.0)
        sim = drivetrain.simulate_construct()
        assert sim is not None
        
        # Run simulation for 0.1s
        result_obj = sim.run(duration=0.1, dt=0.01)
        result = result_obj.to_dict()
        
        assert len(result["times"]) >= 10
        assert np.isclose(result["times"][-1], 0.1, atol=0.01)
        assert len(result["x"]) == len(result["times"])
        
        # Robot shouldn't move if no inputs
        assert np.allclose(result["x"], 0.0)
        assert np.allclose(result["y"], 0.0)
