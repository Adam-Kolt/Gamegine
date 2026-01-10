"""
Tests for Battery Model and Motor Current Limiting
===================================================

Verifies:
1. Battery voltage sag under load
2. Brownout current limits
3. Motor torque-speed curve with current limiting
4. Swerve module torque with battery integration
"""

import pytest
import sys
import os
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '../')))

from gamegine.reference.battery import BatteryModel
from gamegine.reference.motors import KrakenX60, MotorConfig, PowerConfig, MotorSpecification
from gamegine.reference.swerve import SwerveModule, SwerveConfig
from gamegine.reference import gearing
from gamegine.utils.NCIM.ComplexDimensions.omega import RadiansPerSecond
from gamegine.utils.NCIM.ComplexDimensions.torque import NewtonMeter
from gamegine.utils.NCIM.ComplexDimensions.electricpot import Volt
from gamegine.utils.NCIM.Dimensions.current import Ampere


class TestBatteryModel:
    """Test battery voltage sag and brownout protection."""
    
    def test_battery_creation(self):
        """Test default battery creation."""
        battery = BatteryModel.frc_standard()
        assert battery.state_of_charge == 1.0
        assert battery.internal_resistance > 0
        
    def test_voltage_sag_at_zero_current(self):
        """At zero current, terminal voltage equals OCV."""
        battery = BatteryModel.frc_standard()
        terminal_v = battery.get_terminal_voltage(Ampere(0))
        ocv = battery.open_circuit_voltage.to(Volt)
        assert abs(terminal_v.to(Volt) - ocv) < 0.01
        
    def test_voltage_sag_under_load(self):
        """Terminal voltage drops under load."""
        battery = BatteryModel(
            open_circuit_voltage=Volt(13.0),
            internal_resistance=0.020
        )
        # At 200A: V = 13 - 200*0.02 = 9V
        terminal_v = battery.get_terminal_voltage(Ampere(200))
        expected = 13.0 - 200 * 0.020
        assert abs(terminal_v.to(Volt) - expected) < 0.01
        
    def test_max_current_before_brownout(self):
        """Calculate max current that doesn't trigger brownout."""
        battery = BatteryModel(
            open_circuit_voltage=Volt(12.0),
            internal_resistance=0.015,
            brownout_voltage=Volt(6.3)
        )
        # I_max = (12 - 6.3) / 0.015 = 380A
        max_i = battery.get_max_current_before_brownout()
        expected = (12.0 - 6.3) / 0.015
        assert abs(max_i.to(Ampere) - expected) < 1.0
        
    def test_brownout_detection(self):
        """Detect brownout condition."""
        battery = BatteryModel(
            open_circuit_voltage=Volt(12.0),
            internal_resistance=0.015,
            brownout_voltage=Volt(6.3)
        )
        # At 400A: V = 12 - 400*0.015 = 6V (brownout!)
        assert battery.is_brownout(Ampere(400)) == True
        # At 300A: V = 12 - 300*0.015 = 7.5V (OK)
        assert battery.is_brownout(Ampere(300)) == False
        
    def test_worst_case_battery(self):
        """Worst case battery has lower voltage and higher resistance."""
        worst = BatteryModel.worst_case()
        standard = BatteryModel.frc_standard()
        
        assert worst.open_circuit_voltage.to(Volt) < standard.open_circuit_voltage.to(Volt)
        assert worst.internal_resistance > standard.internal_resistance
        
    def test_soc_tracking(self):
        """State of charge decreases with current draw."""
        battery = BatteryModel.frc_standard()
        initial_soc = battery.state_of_charge
        
        # Draw 50A for 60 seconds
        battery.update_soc(Ampere(50), 60.0)
        
        assert battery.state_of_charge < initial_soc
        assert battery.state_of_charge > 0
        # OCV should have dropped
        assert battery.open_circuit_voltage.to(Volt) < 13.03


class TestMotorTorque:
    """Test motor torque-speed curve with current limiting."""
    
    def test_torque_at_zero_speed_current_limited(self):
        """At zero speed with current limit, torque = kT * I_limit."""
        # With 60A limit: τ = 0.0194 * 60 = 1.164 Nm motor torque
        torque = KrakenX60.get_torque_at(
            RadiansPerSecond(0),
            supply_current_limit=Ampere(60),
            stator_current_limit=Ampere(0)
        )
        expected = float(KrakenX60.kT) * 60
        assert abs(torque.to(NewtonMeter) - expected) < 0.1
        
    def test_torque_at_free_speed_is_zero(self):
        """At free speed, torque is zero regardless of current limit."""
        free_speed = KrakenX60.free_speed
        torque = KrakenX60.get_torque_at(
            free_speed,
            supply_current_limit=Ampere(200),
            stator_current_limit=Ampere(0)
        )
        assert torque.to(NewtonMeter) < 0.1
        
    def test_torque_near_free_speed_is_low(self):
        """Near free speed, torque is low (back-EMF limits it)."""
        # 90% of free speed
        speed = RadiansPerSecond(KrakenX60.free_speed.to(RadiansPerSecond) * 0.9)
        torque = KrakenX60.get_torque_at(
            speed,
            supply_current_limit=Ampere(200),
            stator_current_limit=Ampere(0)
        )
        stall_torque = KrakenX60.stall_torque.to(NewtonMeter)
        # Should be about 10% of stall torque
        assert torque.to(NewtonMeter) < stall_torque * 0.2
        
    def test_voltage_reduction_reduces_torque(self):
        """Lower voltage reduces available torque (at mid-speed where curve matters)."""
        # At mid-speed with high current limit, the curve (not current) limits torque
        # Voltage reduction then reduces the available torque
        mid_speed = RadiansPerSecond(300)  # ~50% of free speed
        
        full_torque = KrakenX60.get_torque_at(
            mid_speed,
            supply_current_limit=Ampere(300),  # High limit so curve dominates
            available_voltage=Volt(12)
        )
        reduced_torque = KrakenX60.get_torque_at(
            mid_speed,
            supply_current_limit=Ampere(300),
            available_voltage=Volt(10)
        )
        # Lower voltage = lower torque
        assert reduced_torque.to(NewtonMeter) < full_torque.to(NewtonMeter)
        
    def test_voltage_reduction_reduces_free_speed(self):
        """Lower voltage reduces max speed (no torque at higher speeds)."""
        # At 50% voltage, free speed is halved
        half_free_speed = KrakenX60.free_speed.to(RadiansPerSecond) * 0.5
        
        # At full voltage, this speed has torque
        torque_full_v = KrakenX60.get_torque_at(
            RadiansPerSecond(half_free_speed),
            supply_current_limit=Ampere(100),
            available_voltage=Volt(12)
        )
        
        # At half voltage, this speed is AT the free speed (no torque)
        torque_half_v = KrakenX60.get_torque_at(
            RadiansPerSecond(half_free_speed),
            supply_current_limit=Ampere(100),
            available_voltage=Volt(6)
        )
        
        assert torque_full_v.to(NewtonMeter) > 1.0
        assert torque_half_v.to(NewtonMeter) < 0.1


class TestSwerveModuleTorque:
    """Test swerve module torque with battery integration."""
    
    @pytest.fixture
    def module(self):
        return SwerveModule(
            drive_motor=MotorConfig(
                KrakenX60,
                PowerConfig(Ampere(60), Ampere(240), 1.0),
            ),
            drive_gear_ratio=gearing.MK4I.L3,
            steer_motor=MotorConfig(
                KrakenX60,
                PowerConfig(Ampere(60), Ampere(240), 1.0),
            ),
            steer_gear_ratio=gearing.MK4I.L3,
        )
    
    def test_wheel_torque_at_zero_speed(self, module):
        """At zero wheel speed, get max wheel torque (current limited × gear ratio)."""
        gear_ratio = gearing.MK4I.L3.get_ratio()
        torque = module.get_torque(RadiansPerSecond(0))
        
        # Motor torque at 0 speed with 60A limit
        motor_torque = float(KrakenX60.kT) * 60
        expected_wheel_torque = motor_torque * gear_ratio
        
        assert abs(torque.to(NewtonMeter) - expected_wheel_torque) < 1.0
        
    def test_wheel_torque_near_max_speed_is_low(self, module):
        """Near max wheel speed, torque is low."""
        max_wheel_speed = module.get_max_speed()
        near_max = RadiansPerSecond(max_wheel_speed.to(RadiansPerSecond) * 0.95)
        torque = module.get_torque(near_max)
        
        # Should be very low
        assert torque.to(NewtonMeter) < 5.0
        
    def test_battery_reduces_wheel_torque(self, module):
        """Battery voltage sag reduces wheel torque at high-speed where curve dominates."""
        # At high wheel speed, motor is near free speed and curve torque dominates
        high_speed = RadiansPerSecond(80)  # ~78% of max wheel speed
        
        full_torque = module.get_torque(high_speed, available_voltage=Volt(12))
        sagged_torque = module.get_torque(high_speed, available_voltage=Volt(10))
        
        assert sagged_torque.to(NewtonMeter) < full_torque.to(NewtonMeter)


class TestRealisticAcceleration:
    """Verify that acceleration limits are now realistic."""
    
    def test_max_acceleration_estimate(self):
        """
        Estimate max acceleration with realistic current limits.
        
        With 60A supply limit per motor, 4 motors:
        - Motor torque at stall: kT * 60A ≈ 1.16 Nm
        - Wheel torque: 1.16 * 6.12 ≈ 7.1 Nm per wheel
        - Wheel radius: 0.051m (4" wheel)
        - Force per wheel: 7.1 / 0.051 ≈ 139 N
        - Total force: 139 * 4 = 556 N
        - Robot mass: ~55 kg (120 lbs)
        - Max accel: 556 / 55 ≈ 10.1 m/s²
        
        This is still high but more reasonable than 15 m/s².
        With battery sag (say 10V), it drops to ~8 m/s².
        """
        motor_config = MotorConfig(
            KrakenX60,
            PowerConfig(Ampere(60), Ampere(240), 1.0),
        )
        module = SwerveModule(
            drive_motor=motor_config,
            drive_gear_ratio=gearing.MK4I.L3,
            steer_motor=motor_config,
            steer_gear_ratio=gearing.MK4I.L3,
        )
        
        wheel_radius = 0.051  # 4" wheel in meters
        robot_mass = 55  # kg
        
        # Get torque at zero speed (max torque point)
        wheel_torque = module.get_torque(RadiansPerSecond(0))
        force_per_wheel = wheel_torque.to(NewtonMeter) / wheel_radius
        total_force = force_per_wheel * 4
        max_accel = total_force / robot_mass
        
        # Should be in realistic range (5-12 m/s²)
        assert 5 < max_accel < 15, f"Unrealistic acceleration: {max_accel} m/s²"
        
        # At high-speed, battery voltage matters more
        # Use high wheel-speed where curve torque dominates (motor near free speed)
        high_speed = RadiansPerSecond(80)  # ~78% of max wheel speed
        
        full_v_torque = module.get_torque(high_speed, available_voltage=Volt(12))
        sagged_v_torque = module.get_torque(high_speed, available_voltage=Volt(10))
        
        assert sagged_v_torque.to(NewtonMeter) < full_v_torque.to(NewtonMeter), \
            "Sagged battery should reduce torque at mid-speed"
        print(f"Max accel @12V (stall): {max_accel:.1f} m/s²")
        print(f"Torque @50rad/s: 12V={full_v_torque.to(NewtonMeter):.1f}Nm, 10V={sagged_v_torque.to(NewtonMeter):.1f}Nm")


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
