"""
Elevator PID Simulation Example

Demonstrates the general-purpose MechanismSimulator with:
- Kraken X60 motor
- 10:1 gear ratio, 5cm spool radius
- 10kg vertical load (elevator)
- Python PID controller with chained simulation runs

The simulation runs in batched Rust execution, but Python can
change inputs between runs.
"""

import sys
import os
import matplotlib.pyplot as plt
import numpy as np

# Ensure we can import gamegine
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from gamegine.hifi_sim import (
    Motor, Battery, LinkConfig, MechanismSimulator, is_available
)

if not is_available():
    print("Rust extension not available. Please compile gamegine_sim_py.")
    sys.exit(1)


class PIDController:
    """Simple PID controller."""
    def __init__(self, kp: float, ki: float, kd: float):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.integral = 0.0
        self.prev_error = 0.0
    
    def calculate(self, setpoint: float, measurement: float, dt: float) -> float:
        error = setpoint - measurement
        self.integral += error * dt
        derivative = (error - self.prev_error) / dt if dt > 0 else 0.0
        self.prev_error = error
        return self.kp * error + self.ki * self.integral + self.kd * derivative


def run_simulation():
    # ==========================================
    # Setup Components
    # ==========================================
    motor = Motor.kraken_x60()
    battery = Battery.frc_standard()
    
    # Link configuration: 10:1 gear ratio, 5cm spool radius, 85% efficient
    link = LinkConfig(
        gear_ratio=10.0,
        radius=0.05,  # 5cm spool for linear output
        efficiency=0.85,
        friction_viscous=0.1,  # Small viscous friction
    )
    
    # Create simulator: 10kg vertical load (elevator)
    sim = MechanismSimulator(
        motor=motor,
        battery=battery,
        link_config=link,
        load_mass=10.0,
        load_type="vertical",
    )
    
    # ==========================================
    # Controller Setup
    # ==========================================
    pid = PIDController(kp=20.0, ki=1.0, kd=5.0)
    target_position = 1.0  # Target: 1 meter
    
    # Control loop parameters
    control_dt = 0.02  # 50Hz control loop (20ms)
    sim_dt = 0.001     # 1kHz physics simulation
    total_time = 3.0   # 3 seconds
    
    # ==========================================
    # Run Simulation with Python Control Loop
    # ==========================================
    all_times = []
    all_positions = []
    all_velocities = []
    all_currents = []
    all_voltages = []
    all_duties = []
    
    time_offset = 0.0
    
    while sim.time < total_time:
        # Get current state
        position = sim.position
        
        # Calculate control output
        duty = pid.calculate(target_position, position, control_dt)
        
        # Clamp duty cycle
        duty = max(-1.0, min(1.0, duty))
        
        # Set input and run batch simulation for control period
        sim.set_duty_cycle(duty)
        result = sim.run(control_dt, sim_dt)
        
        # Accumulate results
        times = result["times"]
        all_times.extend(times)
        all_positions.extend(result["position"])
        all_velocities.extend(result["velocity"])
        all_currents.extend(result["current"])
        all_voltages.extend(result["voltage"])
        all_duties.extend([duty] * len(times))
    
    # ==========================================
    # Plotting
    # ==========================================
    fig, axes = plt.subplots(4, 1, figsize=(12, 10), sharex=True)
    
    # Position
    axes[0].plot(all_times, all_positions, 'b-', linewidth=1.5, label='Position')
    axes[0].axhline(target_position, color='r', linestyle='--', label='Target')
    axes[0].set_ylabel('Position (m)')
    axes[0].set_title('Elevator Position Response')
    axes[0].legend()
    axes[0].grid(True, alpha=0.3)
    
    # Velocity
    axes[1].plot(all_times, all_velocities, 'g-', linewidth=1)
    axes[1].set_ylabel('Velocity (m/s)')
    axes[1].set_title('Elevator Velocity')
    axes[1].grid(True, alpha=0.3)
    
    # Current
    axes[2].plot(all_times, all_currents, 'orange', linewidth=1)
    axes[2].set_ylabel('Current (A)')
    axes[2].set_title('Motor Current (q-axis)')
    axes[2].grid(True, alpha=0.3)
    
    # Battery Voltage
    axes[3].plot(all_times, all_voltages, 'purple', linewidth=1)
    axes[3].set_ylabel('Voltage (V)')
    axes[3].set_xlabel('Time (s)')
    axes[3].set_title('Battery Voltage')
    axes[3].grid(True, alpha=0.3)
    
    plt.tight_layout()
    plt.savefig('elevator_response.png', dpi=150)
    plt.show()
    
    # Print summary
    print(f"\nSimulation Summary:")
    print(f"  Final position: {all_positions[-1]:.4f} m (target: {target_position} m)")
    print(f"  Final velocity: {all_velocities[-1]:.4f} m/s")
    print(f"  Peak current: {max(abs(c) for c in all_currents):.1f} A")
    print(f"  Min voltage: {min(all_voltages):.2f} V")


if __name__ == "__main__":
    run_simulation()
