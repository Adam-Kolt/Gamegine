"""
Tests for MechanismSimulator physics behavior.

These tests verify that the simulation produces sensible results:
- Motor produces torque in the expected direction
- Gravity affects vertical loads correctly
- Gear ratios scale forces appropriately
- Battery voltage sags under load
"""

import sys
import os
import pytest
import numpy as np

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from gamegine.hifi_sim import (
    Motor, Battery, LinkConfig, MechanismSimulator, is_available
)

pytestmark = pytest.mark.skipif(
    not is_available(),
    reason="Rust extension not available"
)


class TestMechanismSimulator:
    """Test general mechanism simulation behavior."""
    
    def test_vertical_load_falls_with_no_input(self):
        """A vertical load with no motor input should fall due to gravity."""
        motor = Motor.kraken_x60()
        battery = Battery.frc_standard()
        link = LinkConfig(gear_ratio=10.0, radius=0.05)
        
        sim = MechanismSimulator(motor, battery, link, load_mass=10.0, load_type="vertical")
        
        # Run with zero duty cycle
        sim.set_duty_cycle(0.0)
        result = sim.run(0.5, dt=0.001)
        
        # Position should decrease (falling)
        positions = result["position"]
        assert positions[-1] < positions[0], "Vertical load should fall with no motor input"
        
        # Velocity should be negative (downward)
        velocities = result["velocity"]
        assert velocities[-1] < 0, "Velocity should be negative (falling)"
    
    def test_positive_duty_lifts_elevator(self):
        """Positive duty cycle should lift the elevator against gravity."""
        motor = Motor.kraken_x60()
        battery = Battery.frc_standard()
        link = LinkConfig(gear_ratio=10.0, radius=0.05)
        
        sim = MechanismSimulator(motor, battery, link, load_mass=5.0, load_type="vertical")
        
        # Run with positive duty cycle
        sim.set_duty_cycle(0.5)
        result = sim.run(0.5, dt=0.001)
        
        # Position should increase (rising)
        positions = result["position"]
        assert positions[-1] > positions[0], "Positive duty should lift elevator"
    
    def test_horizontal_load_no_gravity(self):
        """Horizontal load should not be affected by gravity."""
        motor = Motor.kraken_x60()
        battery = Battery.frc_standard()
        link = LinkConfig(gear_ratio=10.0, radius=0.05)
        
        sim = MechanismSimulator(motor, battery, link, load_mass=10.0, load_type="horizontal")
        
        # Run with zero duty cycle
        sim.set_duty_cycle(0.0)
        result = sim.run(0.5, dt=0.001)
        
        # Position should stay near zero
        positions = result["position"]
        assert abs(positions[-1]) < 0.01, "Horizontal load should not move without input"
    
    def test_flywheel_spins_up(self):
        """Flywheel should spin up under positive duty."""
        motor = Motor.kraken_x60()
        battery = Battery.frc_standard()
        # No radius = rotational output
        link = LinkConfig(gear_ratio=5.0, radius=0.0)
        
        sim = MechanismSimulator(motor, battery, link, load_mass=0.1, load_type="flywheel")
        
        sim.set_duty_cycle(0.8)
        result = sim.run(1.0, dt=0.001)
        
        # Velocity should be positive and significant
        velocities = result["velocity"]
        assert velocities[-1] > 10.0, "Flywheel should spin up significantly"
    
    def test_motor_current_flows(self):
        """Motor should draw current under load."""
        motor = Motor.kraken_x60()
        battery = Battery.frc_standard()
        link = LinkConfig(gear_ratio=10.0, radius=0.05)
        
        sim = MechanismSimulator(motor, battery, link, load_mass=10.0, load_type="vertical")
        
        sim.set_duty_cycle(0.8)
        result = sim.run(0.1, dt=0.001)
        
        currents = result["current"]
        # Should have non-zero current
        max_current = max(abs(c) for c in currents)
        assert max_current > 1.0, f"Motor should draw significant current, got {max_current}"
    
    def test_battery_voltage_sag(self):
        """Battery voltage should sag under high current draw."""
        motor = Motor.kraken_x60()
        battery = Battery.frc_standard()
        link = LinkConfig(gear_ratio=10.0, radius=0.05)
        
        sim = MechanismSimulator(motor, battery, link, load_mass=20.0, load_type="vertical")
        
        # High duty cycle = high current
        sim.set_duty_cycle(1.0)
        result = sim.run(0.5, dt=0.001)
        
        voltages = result["voltage"]
        initial_voltage = voltages[0]
        min_voltage = min(voltages)
        
        # Voltage should drop from nominal ~12.7V
        assert min_voltage < initial_voltage, "Battery voltage should sag under load"
    
    def test_state_persists_between_runs(self):
        """State should persist between run() calls."""
        motor = Motor.kraken_x60()
        battery = Battery.frc_standard()
        link = LinkConfig(gear_ratio=10.0, radius=0.05)
        
        sim = MechanismSimulator(motor, battery, link, load_mass=5.0, load_type="vertical")
        
        # First run
        sim.set_duty_cycle(0.5)
        result1 = sim.run(0.3, dt=0.001)
        pos_after_first = sim.position
        time_after_first = sim.time
        
        # Second run with different input
        sim.set_duty_cycle(-0.2)
        result2 = sim.run(0.3, dt=0.001)
        
        # Time should continue
        assert sim.time > time_after_first, "Time should continue across runs"
        
        # Position should have changed
        assert sim.position != pos_after_first, "Position should change in second run"
        
        # First result's last position should be close to second result's first position
        # Note: There's one step of physics between them, so use relative tolerance
        assert np.isclose(result1["position"][-1], result2["position"][0], rtol=0.05)
    
    def test_reset_clears_state(self):
        """reset() should return simulation to initial state."""
        motor = Motor.kraken_x60()
        battery = Battery.frc_standard()
        link = LinkConfig(gear_ratio=10.0, radius=0.05)
        
        sim = MechanismSimulator(motor, battery, link, load_mass=5.0, load_type="vertical")
        
        # Run simulation
        sim.set_duty_cycle(0.5)
        sim.run(0.5, dt=0.001)
        
        assert sim.time > 0
        assert sim.position != 0
        
        # Reset
        sim.reset()
        
        assert sim.time == 0.0
        assert sim.position == 0.0
        assert sim.velocity == 0.0
    
    def test_gear_ratio_affects_speed(self):
        """Higher gear ratio should result in slower output speed."""
        motor = Motor.kraken_x60()
        battery = Battery.frc_standard()
        
        # Low gear ratio
        link_low = LinkConfig(gear_ratio=5.0, radius=0.05)
        sim_low = MechanismSimulator(motor, battery, link_low, load_mass=5.0, load_type="horizontal")
        sim_low.set_duty_cycle(0.5)
        result_low = sim_low.run(0.5, dt=0.001)
        
        # High gear ratio
        link_high = LinkConfig(gear_ratio=20.0, radius=0.05)
        sim_high = MechanismSimulator(motor, battery, link_high, load_mass=5.0, load_type="horizontal")
        sim_high.set_duty_cycle(0.5)
        result_high = sim_high.run(0.5, dt=0.001)
        
        # Lower gear ratio = higher output speed
        speed_low = abs(result_low["velocity"][-1])
        speed_high = abs(result_high["velocity"][-1])
        
        assert speed_low > speed_high, "Lower gear ratio should give higher output speed"


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
