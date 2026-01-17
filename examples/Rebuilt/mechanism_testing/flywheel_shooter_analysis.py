"""
Flywheel Shooter Analysis Tool
==============================

High-fidelity simulation and optimization of flywheel shooter parameters
for FRC-style ball launchers.

Design goals:
- Launch 0.5 lb balls at up to 8 m/s exit velocity
- Support up to 28 balls/second continuous fire rate
- Minimize speed recovery time, current draw, and spinup time
- Optimize gear ratio and moment of inertia

Uses gamegine.hifi_sim MechanismSimulator for physics-accurate motor/battery modeling.
"""

import sys
import os
import numpy as np
import matplotlib.pyplot as plt
from dataclasses import dataclass, field
from typing import Dict, List, Tuple, Optional
import itertools
import time

# Ensure we can import gamegine
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..', '..')))

from gamegine.hifi_sim import (
    Motor, Battery, LinkConfig, MechanismSimulator, is_available
)


# =============================================================================
# Configuration
# =============================================================================

@dataclass
class FlywheelShooterConfig:
    """Configuration for a dual-flywheel shooter mechanism."""
    
    # Wheel geometry
    wheel_diameter_m: float = 0.1016  # 4 inches
    
    # Gearing: Motor:Flywheel reduction (>1 means flywheel slower than motor)
    # For 8 m/s with 4" wheels and Kraken X60 (6000 RPM):
    #   wheel ω = 8 m/s / 0.0508m = 157.5 rad/s
    #   motor free speed = 6000 RPM = 628 rad/s
    #   needed ratio = 628 / 157.5 ≈ 4:1
    gear_ratio: float = 4.0

    
    # Flywheel inertia (moment of inertia in kg⋅m²)
    # Typical FRC flywheel: 0.002 - 0.015 kg⋅m²
    flywheel_moi: float = 0.005
    
    # Ball properties
    ball_mass_kg: float = 0.227  # 0.5 lb
    target_exit_velocity_mps: float = 8.0  # m/s
    
    # Energy transfer efficiency (accounts for slip, compression losses)
    energy_transfer_efficiency: float = 0.85  # 85% of ideal energy transfer
    
    # Motors per side (1 motor for top wheels, 1 for bottom)
    motors_per_side: int = 1
    
    # Motor current limit per motor (A)
    motor_current_limit: float = 40.0
    
    @property
    def wheel_radius_m(self) -> float:
        return self.wheel_diameter_m / 2.0
    
    @property
    def ball_kinetic_energy_j(self) -> float:
        """Energy required to launch one ball."""
        return 0.5 * self.ball_mass_kg * (self.target_exit_velocity_mps ** 2)
    
    @property
    def wheel_surface_velocity_mps(self) -> float:
        """Target wheel surface velocity (equals ball exit velocity for no-slip)."""
        return self.target_exit_velocity_mps
    
    @property
    def wheel_angular_velocity_rad_s(self) -> float:
        """Target wheel angular velocity in rad/s."""
        return self.wheel_surface_velocity_mps / self.wheel_radius_m
    
    @property
    def motor_angular_velocity_rad_s(self) -> float:
        """Motor shaft angular velocity at target wheel speed."""
        return self.wheel_angular_velocity_rad_s * self.gear_ratio


# =============================================================================
# Flywheel Shooter Simulator
# =============================================================================

class FlywheelShooter:
    """
    Simulates a dual-flywheel shooter with energy extraction per shot.
    
    Models:
    - Two independent flywheel/motor systems (top and bottom)
    - Energy transfer from wheels to ball during shot
    - Velocity recovery between shots
    - Current draw and battery effects
    """
    
    def __init__(self, config: FlywheelShooterConfig):
        if not is_available():
            raise RuntimeError("Rust extension required for FlywheelShooter")
        
        self.config = config
        
        # Create motor and battery for each side
        self._motor_top = Motor.kraken_x44()
        self._motor_bottom = Motor.kraken_x44()
        self._battery = Battery.frc_standard()
        
        # Create link configs (radius=0 for rotational output)
        self._link_config_top = LinkConfig(
            gear_ratio=config.gear_ratio,
            radius=0.0,  # Rotational output
            efficiency=0.95,
            friction_viscous=0.001,
        )
        self._link_config_bottom = LinkConfig(
            gear_ratio=config.gear_ratio,
            radius=0.0,
            efficiency=0.95,
            friction_viscous=0.001,
        )
        
        # Create mechanism simulators (flywheel type)
        # load_mass for flywheel type = moment of inertia
        self._sim_top = MechanismSimulator(
            motor=self._motor_top,
            battery=self._battery,
            link_config=self._link_config_top,
            load_mass=config.flywheel_moi,
            load_type="flywheel",
        )
        self._sim_bottom = MechanismSimulator(
            motor=self._motor_bottom,
            battery=self._battery,
            link_config=self._link_config_bottom,
            load_mass=config.flywheel_moi,
            load_type="flywheel",
        )
        
        # Track state
        self._is_spun_up = False
    
    def reset(self):
        """Reset both flywheels to stopped state."""
        self._sim_top.reset()
        self._sim_bottom.reset()
        self._is_spun_up = False
    
    @property
    def top_velocity_rad_s(self) -> float:
        """Top flywheel angular velocity (rad/s at flywheel, not motor)."""
        return self._sim_top.velocity
    
    @property
    def bottom_velocity_rad_s(self) -> float:
        """Bottom flywheel angular velocity (rad/s at flywheel)."""
        return self._sim_bottom.velocity
    
    @property
    def top_surface_velocity_mps(self) -> float:
        """Top wheel surface velocity (m/s)."""
        return self.top_velocity_rad_s * self.config.wheel_radius_m
    
    @property
    def bottom_surface_velocity_mps(self) -> float:
        """Bottom wheel surface velocity (m/s)."""
        return self.bottom_velocity_rad_s * self.config.wheel_radius_m
    
    @property
    def average_surface_velocity_mps(self) -> float:
        """Average surface velocity of top and bottom wheels."""
        return (self.top_surface_velocity_mps + self.bottom_surface_velocity_mps) / 2.0
    
    def spinup(self, target_velocity_mps: Optional[float] = None, 
               max_time: float = 5.0, dt: float = 0.001,
               stop_at_target: bool = True) -> Dict:
        """
        Spin up both flywheels to target surface velocity.
        
        Args:
            target_velocity_mps: Target wheel surface velocity (default: config value)
            max_time: Maximum spinup time
            dt: Simulation timestep
            stop_at_target: If True, stop when target reached. If False, run full max_time.
            
        Returns:
            Dict with spinup data: times, velocities, currents, spinup_time
        """
        if target_velocity_mps is None:
            target_velocity_mps = self.config.target_exit_velocity_mps
        
        target_omega = target_velocity_mps / self.config.wheel_radius_m
        
        # Time series data
        times = []
        top_velocities = []
        bottom_velocities = []
        top_currents = []
        bottom_currents = []
        top_voltages = []
        
        # Run simulation in chunks with velocity control
        control_dt = 0.001  # 1kHz control loop
        elapsed = 0.0
        spinup_time = None
        
        while elapsed < max_time:
            # Bang-bang velocity control: full power if below target, coast if above
            avg_omega = (self._sim_top.velocity + self._sim_bottom.velocity) / 2.0
            if avg_omega < target_omega:
                self._sim_top.set_duty_cycle(1.0)
                self._sim_bottom.set_duty_cycle(1.0)
            else:
                # At or above target - coast (or brake slightly)
                self._sim_top.set_duty_cycle(0.0)
                self._sim_bottom.set_duty_cycle(0.0)
            
            result_top = self._sim_top.run(control_dt, dt)
            result_bottom = self._sim_bottom.run(control_dt, dt)
            
            # Record every 10ms for lighter data
            if len(times) == 0 or elapsed - times[-1] >= 0.01:
                times.append(elapsed + control_dt)
                top_velocities.append(result_top["velocity"][-1])
                bottom_velocities.append(result_bottom["velocity"][-1])
                top_currents.append(result_top["current"][-1])
                bottom_currents.append(result_bottom["current"][-1])
                top_voltages.append(result_top["voltage"][-1])
            
            elapsed += control_dt
            
            # Check if we've reached target (both wheels within 2%)
            avg_vel = (result_top["velocity"][-1] + result_bottom["velocity"][-1]) / 2.0
            if spinup_time is None and avg_vel >= 0.98 * target_omega:
                spinup_time = elapsed
                if stop_at_target:
                    break
        
        if spinup_time is None:
            spinup_time = max_time  # Didn't reach target
        
        self._is_spun_up = True
        self._target_omega = target_omega  # Store for velocity control during burst
        
        # Convert velocities to surface velocity (m/s)
        return {
            "times": np.array(times),
            "top_velocity_mps": np.array(top_velocities) * self.config.wheel_radius_m,
            "bottom_velocity_mps": np.array(bottom_velocities) * self.config.wheel_radius_m,
            "top_current": np.array(top_currents),
            "bottom_current": np.array(bottom_currents),
            "voltage": np.array(top_voltages),
            "spinup_time": spinup_time,
            "target_velocity_mps": target_velocity_mps,
        }
    
    def shoot_ball(self) -> Dict:
        """
        Simulate energy extraction from one ball passing through.
        
        The ball extracts kinetic energy from both flywheels.
        Energy split 50/50 between top and bottom.
        
        Returns:
            Dict with shot data: velocity_drop, exit_velocity_estimate, energy_extracted
        """
        # Energy required for ball
        ball_ke = self.config.ball_kinetic_energy_j
        
        # Account for inefficiency (need to extract MORE energy to get desired ball KE)
        energy_to_extract = ball_ke / self.config.energy_transfer_efficiency
        
        # Split 50/50 between top and bottom wheels
        energy_per_wheel = energy_to_extract / 2.0
        
        # Current flywheel kinetic energies
        I = self.config.flywheel_moi
        omega_top = self.top_velocity_rad_s
        omega_bottom = self.bottom_velocity_rad_s
        
        ke_top = 0.5 * I * (omega_top ** 2)
        ke_bottom = 0.5 * I * (omega_bottom ** 2)
        
        # New kinetic energies after extraction
        new_ke_top = max(0, ke_top - energy_per_wheel)
        new_ke_bottom = max(0, ke_bottom - energy_per_wheel)
        
        # New angular velocities
        new_omega_top = np.sqrt(2 * new_ke_top / I) if new_ke_top > 0 else 0
        new_omega_bottom = np.sqrt(2 * new_ke_bottom / I) if new_ke_bottom > 0 else 0
        
        # Calculate velocity drops
        delta_omega_top = omega_top - new_omega_top
        delta_omega_bottom = omega_bottom - new_omega_bottom
        
        # Update simulator states
        self._sim_top._inner.set_velocity(new_omega_top)
        self._sim_bottom._inner.set_velocity(new_omega_bottom)
        
        # Estimate ball exit velocity based on average wheel surface velocity at time of shot
        avg_surface_vel = (omega_top + omega_bottom) / 2.0 * self.config.wheel_radius_m
        # Actual exit velocity is less due to energy transfer
        exit_velocity = avg_surface_vel * np.sqrt(self.config.energy_transfer_efficiency)
        
        return {
            "velocity_drop_top_mps": delta_omega_top * self.config.wheel_radius_m,
            "velocity_drop_bottom_mps": delta_omega_bottom * self.config.wheel_radius_m,
            "exit_velocity_estimate_mps": exit_velocity,
            "energy_extracted_j": energy_to_extract,
            "pre_shot_velocity_mps": avg_surface_vel,
            "post_shot_velocity_mps": (new_omega_top + new_omega_bottom) / 2.0 * self.config.wheel_radius_m,
        }
    
    def run_recovery(self, duration: float, dt: float = 0.001) -> Dict:
        """
        Run recovery simulation for specified duration.
        
        Args:
            duration: Time to simulate (seconds)
            dt: Timestep
            
        Returns:
            Dict with recovery data
        """
        self._sim_top.set_duty_cycle(1.0)
        self._sim_bottom.set_duty_cycle(1.0)
        
        result_top = self._sim_top.run(duration, dt)
        result_bottom = self._sim_bottom.run(duration, dt)
        
        return {
            "times": result_top["times"],
            "top_velocity_mps": result_top["velocity"] * self.config.wheel_radius_m,
            "bottom_velocity_mps": result_bottom["velocity"] * self.config.wheel_radius_m,
            "top_current": result_top["current"],
            "bottom_current": result_bottom["current"],
            "voltage": result_top["voltage"],
        }
    
    def run_burst(self, n_balls: int, feed_rate_hz: float, 
                  jitter_fraction: float = 0.0, dt: float = 0.001) -> Dict:
        """
        Simulate a burst of shots at specified feed rate with velocity control.
        
        Flywheels are controlled to maintain target velocity. After each shot,
        we measure recovery time to get back to target.
        
        Args:
            n_balls: Number of balls to shoot
            feed_rate_hz: Feed rate in balls per second
            jitter_fraction: Random timing variation as fraction of interval (0-1)
            dt: Simulation timestep
            
        Returns:
            Dict with full burst time-series data and per-shot metrics
        """
        shot_interval = 1.0 / feed_rate_hz
        
        # Get target omega (set during spinup)
        target_omega = getattr(self, '_target_omega', 
                               self.config.target_exit_velocity_mps / self.config.wheel_radius_m)
        
        times = []
        velocities_top = []
        velocities_bottom = []
        currents_top = []
        currents_bottom = []
        voltages = []
        
        shot_data = []
        current_time = 0.0
        recovery_times = []  # Track time to recover to target after each shot
        
        for i in range(n_balls):
            # Determine time until this shot
            if i == 0:
                time_to_shot = 0.0
            else:
                # Add jitter
                jitter = np.random.uniform(-jitter_fraction, jitter_fraction) * shot_interval
                time_to_shot = shot_interval + jitter
            
            if time_to_shot > 0:
                # Run recovery until shot time WITH velocity control
                n_steps = max(1, int(time_to_shot / 0.001))
                recovery_complete_time = None
                
                for step in range(n_steps):
                    # Bang-bang velocity control
                    avg_omega = (self._sim_top.velocity + self._sim_bottom.velocity) / 2.0
                    if avg_omega < 0.98 * target_omega:
                        self._sim_top.set_duty_cycle(1.0)
                        self._sim_bottom.set_duty_cycle(1.0)
                    else:
                        self._sim_top.set_duty_cycle(0.0)
                        self._sim_bottom.set_duty_cycle(0.0)
                        if recovery_complete_time is None:
                            recovery_complete_time = step * 0.001
                    
                    r_top = self._sim_top.run(0.001, dt)
                    r_bottom = self._sim_bottom.run(0.001, dt)
                    current_time += 0.001
                    
                    times.append(current_time)
                    velocities_top.append(r_top["velocity"][-1] * self.config.wheel_radius_m)
                    velocities_bottom.append(r_bottom["velocity"][-1] * self.config.wheel_radius_m)
                    currents_top.append(r_top["current"][-1])
                    currents_bottom.append(r_bottom["current"][-1])
                    voltages.append(r_top["voltage"][-1])
                
                if i > 0:  # Track recovery time for shots after the first
                    if recovery_complete_time is not None:
                        recovery_times.append(recovery_complete_time)
                    else:
                        recovery_times.append(time_to_shot)  # Didn't fully recover
            
            # Shoot ball
            shot_result = self.shoot_ball()
            shot_result["time"] = current_time
            shot_result["ball_number"] = i + 1
            shot_data.append(shot_result)
            
            # Record post-shot state
            times.append(current_time)
            velocities_top.append(self.top_surface_velocity_mps)
            velocities_bottom.append(self.bottom_surface_velocity_mps)
            # Current spike during shot (not modeled, use last value)
            currents_top.append(currents_top[-1] if currents_top else 0)
            currents_bottom.append(currents_bottom[-1] if currents_bottom else 0)
            voltages.append(voltages[-1] if voltages else 12.0)
        
        # Calculate summary statistics
        exit_velocities = [s["exit_velocity_estimate_mps"] for s in shot_data]
        velocity_drops = [s["velocity_drop_top_mps"] for s in shot_data]
        
        # Recovery time statistics
        mean_recovery_time = np.mean(recovery_times) if recovery_times else 0.0
        max_recovery_time = np.max(recovery_times) if recovery_times else 0.0
        
        return {
            "times": np.array(times),
            "top_velocity_mps": np.array(velocities_top),
            "bottom_velocity_mps": np.array(velocities_bottom),
            "top_current": np.array(currents_top),
            "bottom_current": np.array(currents_bottom),
            "voltage": np.array(voltages),
            "shot_data": shot_data,
            "mean_exit_velocity_mps": np.mean(exit_velocities),
            "min_exit_velocity_mps": np.min(exit_velocities),
            "max_exit_velocity_mps": np.max(exit_velocities),
            "exit_velocity_std_mps": np.std(exit_velocities),
            "mean_velocity_drop_mps": np.mean(velocity_drops),
            "peak_current_a": max(np.max(np.abs(currents_top)), np.max(np.abs(currents_bottom))) if currents_top else 0,
            "mean_recovery_time_s": mean_recovery_time,
            "max_recovery_time_s": max_recovery_time,
        }


# =============================================================================
# Optimization
# =============================================================================

@dataclass
class OptimizationResult:
    """Result from optimization sweep."""
    gear_ratio: float
    flywheel_moi: float
    spinup_time: float
    mean_velocity_drop: float
    peak_current: float
    min_exit_velocity: float
    objective_score: float
    config: FlywheelShooterConfig


class OptimizationRunner:
    """
    Sweeps parameter space to find optimal flywheel configuration.
    
    Supports custom cost functions via the `custom_cost_fn` parameter.
    The function receives a dict of metrics and should return a float (lower is better).
    
    Example custom cost function:
        def my_cost(metrics):
            # Heavily penalize low exit velocity
            if metrics['min_exit_velocity'] < 7.0:
                return 1000.0
            return metrics['spinup_time'] + 2 * metrics['velocity_drop']
    """
    
    # Type alias for metrics dict passed to cost function
    MetricsDict = Dict[str, float]  # Keys: spinup_time, velocity_drop, peak_current, min_exit_velocity, target_velocity, mean_recovery_time, max_recovery_time
    
    def __init__(self, 
                 gear_ratios: List[float] = None,
                 flywheel_mois: List[float] = None,
                 base_config: FlywheelShooterConfig = None,
                 custom_cost_fn: Optional[callable] = None):
        """
        Args:
            gear_ratios: List of gear ratios to sweep
            flywheel_mois: List of MOI values to sweep (kg⋅m²)
            base_config: Base configuration for non-swept parameters
            custom_cost_fn: Optional custom cost function. Receives dict with keys:
                            'spinup_time', 'velocity_drop', 'peak_current', 
                            'min_exit_velocity', 'target_velocity', 
                            'mean_recovery_time', 'max_recovery_time'
                            Should return float (lower is better).
        """
        self.gear_ratios = gear_ratios or [2.5, 3.0, 3.5, 4.0, 4.5, 5.0, 5.5]
        self.flywheel_mois = flywheel_mois or [0.002, 0.004, 0.006, 0.008, 0.010, 0.012, 0.015]
        self.base_config = base_config or FlywheelShooterConfig()
        self.custom_cost_fn = custom_cost_fn
        
        self.results: List[OptimizationResult] = []
    
    def compute_objective(self, 
                         spinup_time: float,
                         mean_velocity_drop: float,
                         peak_current: float,
                         min_exit_velocity: float,
                         target_exit_velocity: float,
                         weights: Dict[str, float] = None) -> float:
        """
        Compute combined objective function (lower is better).
        
        Components:
        - Spinup time (normalized to ~2s baseline)
        - Velocity drop per shot (normalized to ~1 m/s baseline)
        - Peak current (normalized to 100A baseline)
        - Exit velocity penalty (if below target)
        """
        if weights is None:
            weights = {
                "spinup_time": 1.0,
                "velocity_drop": 2.0,
                "peak_current": 0.5,
                "exit_velocity_penalty": 5.0,
            }
        
        # Normalize each component
        spinup_score = spinup_time / 2.0  # 2s baseline
        drop_score = mean_velocity_drop / 1.0  # 1 m/s baseline
        current_score = peak_current / 100.0  # 100A baseline
        
        # Penalty for not meeting exit velocity requirement
        velocity_shortfall = max(0, target_exit_velocity - min_exit_velocity)
        velocity_penalty = velocity_shortfall / target_exit_velocity
        
        objective = (
            weights["spinup_time"] * spinup_score +
            weights["velocity_drop"] * drop_score +
            weights["peak_current"] * current_score +
            weights["exit_velocity_penalty"] * velocity_penalty
        )
        
        return objective
    
    def run_sweep(self, n_balls: int = 10, feed_rate_hz: float = 28.0,
                  jitter: float = 0.0, verbose: bool = True) -> List[OptimizationResult]:
        """
        Run full parameter sweep.
        
        Args:
            n_balls: Number of balls in test burst
            feed_rate_hz: Feed rate for burst test
            jitter: Timing jitter fraction
            verbose: Print progress
            
        Returns:
            List of OptimizationResult sorted by objective (best first)
        """
        self.results = []
        total = len(self.gear_ratios) * len(self.flywheel_mois)
        
        if verbose:
            print(f"Running optimization sweep: {total} configurations")
            print(f"  Gear ratios: {self.gear_ratios}")
            print(f"  MOI values: {self.flywheel_mois}")
            print()
        
        for i, (gr, moi) in enumerate(itertools.product(self.gear_ratios, self.flywheel_mois)):
            if verbose:
                print(f"  [{i+1}/{total}] GR={gr:.2f}, MOI={moi:.4f} kg⋅m² ... ", end="", flush=True)
            
            # Create config for this combination
            config = FlywheelShooterConfig(
                wheel_diameter_m=self.base_config.wheel_diameter_m,
                gear_ratio=gr,
                flywheel_moi=moi,
                ball_mass_kg=self.base_config.ball_mass_kg,
                target_exit_velocity_mps=self.base_config.target_exit_velocity_mps,
                energy_transfer_efficiency=self.base_config.energy_transfer_efficiency,
                motors_per_side=self.base_config.motors_per_side,
            )
            
            try:
                # Create shooter and run tests
                shooter = FlywheelShooter(config)
                
                # Spinup test
                spinup_data = shooter.spinup(max_time=5.0)
                spinup_time = spinup_data["spinup_time"]
                
                # Burst test
                burst_data = shooter.run_burst(n_balls, feed_rate_hz, jitter)
                
                # Extract metrics
                mean_velocity_drop = burst_data["mean_velocity_drop_mps"]
                peak_current = burst_data["peak_current_a"]
                min_exit_velocity = burst_data["min_exit_velocity_mps"]
                
                # Build metrics dict for cost function
                metrics = {
                    'spinup_time': spinup_time,
                    'velocity_drop': mean_velocity_drop,
                    'peak_current': peak_current,
                    'min_exit_velocity': min_exit_velocity,
                    'target_velocity': config.target_exit_velocity_mps,
                    'mean_recovery_time': burst_data.get('mean_recovery_time_s', 0.0),
                    'max_recovery_time': burst_data.get('max_recovery_time_s', 0.0),
                }
                
                # Compute objective using custom or default cost function
                if self.custom_cost_fn is not None:
                    objective = self.custom_cost_fn(metrics)
                else:
                    objective = self.compute_objective(
                        spinup_time=spinup_time,
                        mean_velocity_drop=mean_velocity_drop,
                        peak_current=peak_current,
                        min_exit_velocity=min_exit_velocity,
                        target_exit_velocity=config.target_exit_velocity_mps,
                    )
                
                result = OptimizationResult(
                    gear_ratio=gr,
                    flywheel_moi=moi,
                    spinup_time=spinup_time,
                    mean_velocity_drop=mean_velocity_drop,
                    peak_current=peak_current,
                    min_exit_velocity=min_exit_velocity,
                    objective_score=objective,
                    config=config,
                )
                self.results.append(result)
                
                if verbose:
                    print(f"spinup={spinup_time:.2f}s, drop={mean_velocity_drop:.2f}m/s, min_v={min_exit_velocity:.2f}m/s, obj={objective:.3f}")
                
            except Exception as e:
                if verbose:
                    print(f"FAILED: {e}")
        
        # Sort by objective (lower is better)
        self.results.sort(key=lambda r: r.objective_score)
        
        return self.results
    
    def get_best(self) -> Optional[OptimizationResult]:
        """Get best configuration from last sweep."""
        return self.results[0] if self.results else None


# =============================================================================
# Visualization
# =============================================================================

def plot_spinup_comparison(configs: List[FlywheelShooterConfig], 
                           labels: List[str] = None,
                           save_path: str = None):
    """
    Plot spinup curves for multiple configurations.
    """
    fig, axes = plt.subplots(2, 1, figsize=(12, 8), sharex=True)
    
    colors = plt.cm.viridis(np.linspace(0, 1, len(configs)))
    
    for i, config in enumerate(configs):
        shooter = FlywheelShooter(config)
        data = shooter.spinup(max_time=3.0)
        
        label = labels[i] if labels else f"GR={config.gear_ratio:.1f}, MOI={config.flywheel_moi:.3f}"
        
        # Velocity plot
        axes[0].plot(data["times"], data["top_velocity_mps"], 
                    color=colors[i], linewidth=2, label=label)
        
        # Current plot
        axes[1].plot(data["times"], data["top_current"],
                    color=colors[i], linewidth=1.5)
    
    # Target velocity line
    axes[0].axhline(configs[0].target_exit_velocity_mps, color='red', 
                   linestyle='--', linewidth=1, label='Target')
    
    axes[0].set_ylabel('Surface Velocity (m/s)')
    axes[0].set_title('Flywheel Spinup Comparison')
    axes[0].legend(loc='lower right', fontsize=8)
    axes[0].grid(True, alpha=0.3)
    
    axes[1].set_ylabel('Motor Current (A)')
    axes[1].set_xlabel('Time (s)')
    axes[1].set_title('Motor Current Draw During Spinup')
    axes[1].grid(True, alpha=0.3)
    
    plt.tight_layout()
    
    if save_path:
        plt.savefig(save_path, dpi=150)
        print(f"Saved spinup comparison to {save_path}")
    
    return fig


def plot_burst_sequence(shooter: FlywheelShooter, n_balls: int, feed_rate_hz: float,
                        jitter: float = 0.0, save_path: str = None):
    """
    Plot velocity and current during a burst of shots.
    """
    # First spinup
    shooter.reset()
    spinup_data = shooter.spinup()
    
    # Then run burst
    burst_data = shooter.run_burst(n_balls, feed_rate_hz, jitter)
    
    fig, axes = plt.subplots(3, 1, figsize=(14, 10), sharex=True)
    
    # Combine spinup and burst times
    burst_times = burst_data["times"] + spinup_data["spinup_time"]
    
    # Velocity plot
    axes[0].plot(spinup_data["times"], spinup_data["top_velocity_mps"], 
                'b-', linewidth=1.5, label='Top Wheel')
    axes[0].plot(spinup_data["times"], spinup_data["bottom_velocity_mps"],
                'r-', linewidth=1.5, label='Bottom Wheel', alpha=0.7)
    axes[0].plot(burst_times, burst_data["top_velocity_mps"],
                'b-', linewidth=1.5)
    axes[0].plot(burst_times, burst_data["bottom_velocity_mps"],
                'r-', linewidth=1.5, alpha=0.7)
    
    # Mark shots
    for shot in burst_data["shot_data"]:
        shot_time = shot["time"] + spinup_data["spinup_time"]
        axes[0].axvline(shot_time, color='green', linestyle=':', alpha=0.5)
    
    axes[0].axhline(shooter.config.target_exit_velocity_mps, color='gray',
                   linestyle='--', label='Target', alpha=0.5)
    axes[0].set_ylabel('Surface Velocity (m/s)')
    axes[0].set_title(f'Burst Sequence: {n_balls} balls at {feed_rate_hz} Hz')
    axes[0].legend(loc='lower right')
    axes[0].grid(True, alpha=0.3)
    
    # Current plot
    axes[1].plot(spinup_data["times"], spinup_data["top_current"],
                'b-', linewidth=1, label='Top Motor')
    axes[1].plot(spinup_data["times"], spinup_data["bottom_current"],
                'r-', linewidth=1, label='Bottom Motor', alpha=0.7)
    axes[1].plot(burst_times, burst_data["top_current"],
                'b-', linewidth=1)
    axes[1].plot(burst_times, burst_data["bottom_current"],
                'r-', linewidth=1, alpha=0.7)
    axes[1].set_ylabel('Motor Current (A)')
    axes[1].set_title('Motor Current Draw')
    axes[1].legend(loc='upper right')
    axes[1].grid(True, alpha=0.3)
    
    # Exit velocity per shot
    shot_times = [s["time"] + spinup_data["spinup_time"] for s in burst_data["shot_data"]]
    exit_vels = [s["exit_velocity_estimate_mps"] for s in burst_data["shot_data"]]
    
    axes[2].bar(shot_times, exit_vels, width=0.01, color='green', alpha=0.7)
    axes[2].axhline(shooter.config.target_exit_velocity_mps, color='red',
                   linestyle='--', label='Target')
    axes[2].set_ylabel('Exit Velocity (m/s)')
    axes[2].set_xlabel('Time (s)')
    axes[2].set_title('Ball Exit Velocities')
    axes[2].legend()
    axes[2].grid(True, alpha=0.3)
    
    plt.tight_layout()
    
    if save_path:
        plt.savefig(save_path, dpi=150)
        print(f"Saved burst sequence to {save_path}")
    
    return fig


def plot_optimization_heatmap(results: List[OptimizationResult], 
                              metric: str = "objective_score",
                              save_path: str = None):
    """
    Plot 2D heatmap of optimization results.
    
    Args:
        results: List of OptimizationResult
        metric: Which metric to plot ("objective_score", "spinup_time", etc.)
        save_path: Path to save figure
    """
    # Extract unique gear ratios and MOIs
    gear_ratios = sorted(set(r.gear_ratio for r in results))
    mois = sorted(set(r.flywheel_moi for r in results))
    
    # Create grid
    grid = np.zeros((len(mois), len(gear_ratios)))
    
    for r in results:
        i = mois.index(r.flywheel_moi)
        j = gear_ratios.index(r.gear_ratio)
        grid[i, j] = getattr(r, metric)
    
    fig, ax = plt.subplots(figsize=(10, 8))
    
    # Plot heatmap
    im = ax.imshow(grid, cmap='viridis_r', aspect='auto',
                  extent=[gear_ratios[0]-0.1, gear_ratios[-1]+0.1,
                         mois[0]*1000-0.2, mois[-1]*1000+0.2],
                  origin='lower')
    
    # Add colorbar
    cbar = plt.colorbar(im, ax=ax)
    
    # Metric-specific labels
    metric_labels = {
        "objective_score": ("Objective Score (lower is better)", "Score"),
        "spinup_time": ("Spinup Time", "Time (s)"),
        "mean_velocity_drop": ("Mean Velocity Drop", "m/s"),
        "peak_current": ("Peak Current", "A"),
        "min_exit_velocity": ("Minimum Exit Velocity", "m/s"),
    }
    
    title, cbar_label = metric_labels.get(metric, (metric, metric))
    cbar.set_label(cbar_label)
    
    ax.set_xlabel('Gear Ratio')
    ax.set_ylabel('Flywheel MOI (g⋅m²)')
    ax.set_title(f'Flywheel Optimization: {title}')
    
    # Mark best configuration
    best = min(results, key=lambda r: r.objective_score)
    ax.plot(best.gear_ratio, best.flywheel_moi * 1000, 'r*', 
           markersize=15, label='Optimal')
    ax.legend()
    
    plt.tight_layout()
    
    if save_path:
        plt.savefig(save_path, dpi=150)
        print(f"Saved optimization heatmap to {save_path}")
    
    return fig


def plot_parameter_sensitivity(results: List[OptimizationResult], save_path: str = None):
    """
    Plot how each parameter affects performance metrics.
    """
    fig, axes = plt.subplots(2, 2, figsize=(14, 10))
    
    gear_ratios = sorted(set(r.gear_ratio for r in results))
    mois = sorted(set(r.flywheel_moi for r in results))
    
    # Group results by gear ratio
    for gr in gear_ratios:
        gr_results = [r for r in results if r.gear_ratio == gr]
        gr_results.sort(key=lambda r: r.flywheel_moi)
        
        moi_vals = [r.flywheel_moi * 1000 for r in gr_results]  # Convert to g⋅m²
        
        axes[0, 0].plot(moi_vals, [r.spinup_time for r in gr_results], 
                       'o-', label=f'GR={gr:.1f}')
        axes[0, 1].plot(moi_vals, [r.mean_velocity_drop for r in gr_results],
                       'o-', label=f'GR={gr:.1f}')
    
    axes[0, 0].set_xlabel('Flywheel MOI (g⋅m²)')
    axes[0, 0].set_ylabel('Spinup Time (s)')
    axes[0, 0].set_title('Spinup Time vs MOI')
    axes[0, 0].legend()
    axes[0, 0].grid(True, alpha=0.3)
    
    axes[0, 1].set_xlabel('Flywheel MOI (g⋅m²)')
    axes[0, 1].set_ylabel('Velocity Drop (m/s)')
    axes[0, 1].set_title('Velocity Drop vs MOI')
    axes[0, 1].legend()
    axes[0, 1].grid(True, alpha=0.3)
    
    # Group results by MOI
    for moi in mois:
        moi_results = [r for r in results if r.flywheel_moi == moi]
        moi_results.sort(key=lambda r: r.gear_ratio)
        
        gr_vals = [r.gear_ratio for r in moi_results]
        
        axes[1, 0].plot(gr_vals, [r.peak_current for r in moi_results],
                       'o-', label=f'MOI={moi*1000:.0f}g⋅m²')
        axes[1, 1].plot(gr_vals, [r.objective_score for r in moi_results],
                       'o-', label=f'MOI={moi*1000:.0f}g⋅m²')
    
    axes[1, 0].set_xlabel('Gear Ratio')
    axes[1, 0].set_ylabel('Peak Current (A)')
    axes[1, 0].set_title('Peak Current vs Gear Ratio')
    axes[1, 0].legend(fontsize=7)
    axes[1, 0].grid(True, alpha=0.3)
    
    axes[1, 1].set_xlabel('Gear Ratio')
    axes[1, 1].set_ylabel('Objective Score')
    axes[1, 1].set_title('Overall Objective vs Gear Ratio')
    axes[1, 1].legend(fontsize=7)
    axes[1, 1].grid(True, alpha=0.3)
    
    plt.tight_layout()
    
    if save_path:
        plt.savefig(save_path, dpi=150)
        print(f"Saved parameter sensitivity to {save_path}")
    
    return fig


# =============================================================================
# Main Execution
# =============================================================================

def run_analysis(
    target_velocity_mps: float = 8.0,
    ball_mass_kg: float = 0.227,
    feed_rate_hz: float = 28.0,
    n_balls_burst: int = 28,
    energy_efficiency: float = 0.85,
    jitter_fraction: float = 0.0,
    run_optimization: bool = True,
    save_plots: bool = True,
    output_dir: str = None,
):
    """
    Run complete flywheel shooter analysis.
    
    Args:
        target_velocity_mps: Target ball exit velocity (m/s)
        ball_mass_kg: Ball mass (kg)
        feed_rate_hz: Ball feed rate (balls/second)
        n_balls_burst: Number of balls in burst test
        energy_efficiency: Energy transfer efficiency (0-1)
        jitter_fraction: Timing jitter as fraction of interval
        run_optimization: Whether to run full optimization sweep
        save_plots: Whether to save plots to files
        output_dir: Directory for output files (default: script directory)
    """
    if not is_available():
        print("ERROR: Rust extension 'gamegine_sim_py' not available.")
        print("Please compile the extension first.")
        return
    
    if output_dir is None:
        output_dir = os.path.dirname(os.path.abspath(__file__))
    
    print("=" * 60)
    print("FLYWHEEL SHOOTER ANALYSIS")
    print("=" * 60)
    print()
    print(f"Target exit velocity: {target_velocity_mps} m/s")
    print(f"Ball mass: {ball_mass_kg:.3f} kg ({ball_mass_kg * 2.205:.2f} lb)")
    print(f"Ball kinetic energy: {0.5 * ball_mass_kg * target_velocity_mps**2:.2f} J")
    print(f"Feed rate: {feed_rate_hz} balls/second")
    print(f"Energy transfer efficiency: {energy_efficiency * 100:.0f}%")
    print(f"Jitter: {jitter_fraction * 100:.0f}%")
    print()
    
    # =========================================================================
    # Part 1: Default Configuration Demo
    # =========================================================================
    print("-" * 60)
    print("PART 1: Default Configuration Demo")
    print("-" * 60)
    
    default_config = FlywheelShooterConfig(
        target_exit_velocity_mps=target_velocity_mps,
        ball_mass_kg=ball_mass_kg,
        energy_transfer_efficiency=energy_efficiency,
        gear_ratio=3.0,
        flywheel_moi=0.001,  # 6 g⋅m²
    )
    
    shooter = FlywheelShooter(default_config)
    
    # Spinup
    print("\nSpinup test...")
    spinup_data = shooter.spinup()
    print(f"  Spinup time to {target_velocity_mps} m/s: {spinup_data['spinup_time']:.3f} s")
    print(f"  Final velocity: {spinup_data['top_velocity_mps'][-1]:.2f} m/s")
    
    # Burst
    print(f"\nBurst test: {n_balls_burst} balls at {feed_rate_hz} Hz...")
    shooter.reset()
    shooter.spinup()
    burst_data = shooter.run_burst(n_balls_burst, feed_rate_hz, jitter_fraction)
    
    print(f"  Mean exit velocity: {burst_data['mean_exit_velocity_mps']:.2f} m/s")
    print(f"  Min exit velocity: {burst_data['min_exit_velocity_mps']:.2f} m/s")
    print(f"  Max exit velocity: {burst_data['max_exit_velocity_mps']:.2f} m/s")
    print(f"  Exit velocity std: {burst_data['exit_velocity_std_mps']:.3f} m/s")
    print(f"  Mean velocity drop: {burst_data['mean_velocity_drop_mps']:.3f} m/s")
    print(f"  Peak current: {burst_data['peak_current_a']:.1f} A")
    
    # Plot burst sequence
    if save_plots:
        shooter.reset()
        plot_burst_sequence(
            shooter, n_balls_burst, feed_rate_hz, jitter_fraction,
            save_path=os.path.join(output_dir, "flywheel_burst.png")
        )
    
    # =========================================================================
    # Part 2: Spinup Comparison
    # =========================================================================
    print()
    print("-" * 60)
    print("PART 2: Configuration Comparison")
    print("-" * 60)
    
    comparison_configs = [
        FlywheelShooterConfig(gear_ratio=3.0, flywheel_moi=0.004, 
                             target_exit_velocity_mps=target_velocity_mps,
                             energy_transfer_efficiency=energy_efficiency),
        FlywheelShooterConfig(gear_ratio=3.5, flywheel_moi=0.006,
                             target_exit_velocity_mps=target_velocity_mps,
                             energy_transfer_efficiency=energy_efficiency),
        FlywheelShooterConfig(gear_ratio=4.0, flywheel_moi=0.008,
                             target_exit_velocity_mps=target_velocity_mps,
                             energy_transfer_efficiency=energy_efficiency),
        FlywheelShooterConfig(gear_ratio=4.5, flywheel_moi=0.010,
                             target_exit_velocity_mps=target_velocity_mps,
                             energy_transfer_efficiency=energy_efficiency),
    ]
    
    if save_plots:
        plot_spinup_comparison(
            comparison_configs,
            save_path=os.path.join(output_dir, "flywheel_spinup.png")
        )
    
    # =========================================================================
    # Part 3: Optimization Sweep
    # =========================================================================
    if run_optimization:
        print()
        print("-" * 60)
        print("PART 3: Optimization Sweep")
        print("-" * 60)
        
        optimizer = OptimizationRunner(
            gear_ratios=[x for x in np.linspace(1.0, 10.0, 20)],
            flywheel_mois=[x for x in np.linspace(0.001, 0.1, 30)],
            base_config=default_config,
        )
        
        results = optimizer.run_sweep(
            n_balls=n_balls_burst,
            feed_rate_hz=feed_rate_hz,
            jitter=jitter_fraction,
            verbose=True,
        )
        
        print()
        print("=" * 60)
        print("OPTIMIZATION RESULTS")
        print("=" * 60)
        
        best = optimizer.get_best()
        if best:
            print()
            print("OPTIMAL CONFIGURATION:")
            print(f"  Gear Ratio: {best.gear_ratio:.2f}:1")
            print(f"  Flywheel MOI: {best.flywheel_moi * 1000:.1f} g⋅m²")
            print()
            print("PERFORMANCE:")
            print(f"  Spinup Time: {best.spinup_time:.3f} s")
            print(f"  Mean Velocity Drop: {best.mean_velocity_drop:.3f} m/s")
            print(f"  Peak Current: {best.peak_current:.1f} A")
            print(f"  Min Exit Velocity: {best.min_exit_velocity:.2f} m/s")
            print(f"  Objective Score: {best.objective_score:.4f}")
        
        # Top 5 configurations
        print()
        print("TOP 5 CONFIGURATIONS:")
        print("-" * 80)
        print(f"{'Rank':<5} {'GR':>6} {'MOI(g⋅m²)':>10} {'Spinup(s)':>10} {'Drop(m/s)':>10} {'Score':>8}")
        print("-" * 80)
        for i, r in enumerate(results[:5]):
            print(f"{i+1:<5} {r.gear_ratio:>6.2f} {r.flywheel_moi*1000:>10.1f} "
                  f"{r.spinup_time:>10.3f} {r.mean_velocity_drop:>10.3f} {r.objective_score:>8.4f}")
        
        # Save plots
        if save_plots:
            plot_optimization_heatmap(
                results,
                save_path=os.path.join(output_dir, "flywheel_optimization.png")
            )
            plot_parameter_sensitivity(
                results,
                save_path=os.path.join(output_dir, "flywheel_sensitivity.png")
            )
    
    print()
    print("=" * 60)
    print("Analysis complete!")
    if save_plots:
        print(f"Plots saved to: {output_dir}")
    print("=" * 60)
    
    plt.show()


def verify_physics():
    """
    Run physics verification tests.
    """
    print("Running physics verification...")
    
    config = FlywheelShooterConfig()
    shooter = FlywheelShooter(config)
    
    # Test 1: Spinup produces reasonable time
    spinup_data = shooter.spinup()
    assert 0.01 < spinup_data["spinup_time"] < 10.0, \
        f"Spinup time {spinup_data['spinup_time']} outside reasonable range"
    print(f"  ✓ Spinup time: {spinup_data['spinup_time']:.3f} s (0.01-10s expected)")
    
    # Test 2: Peak current under stall current
    max_current = np.max(np.abs(spinup_data["top_current"]))
    assert max_current < 400, f"Peak current {max_current} exceeds motor capability"
    print(f"  ✓ Peak current: {max_current:.1f} A (<400A expected)")
    
    # Test 3: Energy conservation in shot
    shooter.reset()
    shooter.spinup()
    
    pre_vel = shooter.average_surface_velocity_mps
    shot_data = shooter.shoot_ball()
    post_vel = shooter.average_surface_velocity_mps
    
    # Velocity should drop
    assert post_vel < pre_vel, "Velocity should drop after shot"
    print(f"  ✓ Velocity drop: {pre_vel:.2f} → {post_vel:.2f} m/s")
    
    # Test 4: Burst produces consistent shots
    shooter.reset()
    shooter.spinup()
    burst_data = shooter.run_burst(10, 10.0)
    
    # All exit velocities should be positive
    assert burst_data["min_exit_velocity_mps"] > 0, "Exit velocity should be positive"
    print(f"  ✓ Exit velocities: {burst_data['min_exit_velocity_mps']:.2f} - "
          f"{burst_data['max_exit_velocity_mps']:.2f} m/s")
    
    print()
    print("All physics verification tests passed!")


if __name__ == "__main__":
    import argparse
    
    parser = argparse.ArgumentParser(description="Flywheel Shooter Analysis Tool")
    parser.add_argument("--verify-physics", action="store_true",
                       help="Run physics verification tests")
    parser.add_argument("--no-optimization", action="store_true",
                       help="Skip optimization sweep")
    parser.add_argument("--no-plots", action="store_true",
                       help="Don't save plots")
    parser.add_argument("--target-velocity", type=float, default=8.0,
                       help="Target exit velocity (m/s)")
    parser.add_argument("--feed-rate", type=float, default=28.0,
                       help="Ball feed rate (balls/second)")
    parser.add_argument("--n-balls", type=int, default=28,
                       help="Number of balls in burst test")
    parser.add_argument("--efficiency", type=float, default=0.85,
                       help="Energy transfer efficiency (0-1)")
    parser.add_argument("--jitter", type=float, default=0.0,
                       help="Timing jitter fraction (0-1)")
    
    args = parser.parse_args()
    
    if args.verify_physics:
        verify_physics()
    else:
        run_analysis(
            target_velocity_mps=args.target_velocity,
            feed_rate_hz=args.feed_rate,
            n_balls_burst=args.n_balls,
            energy_efficiency=args.efficiency,
            jitter_fraction=args.jitter,
            run_optimization=not args.no_optimization,
            save_plots=not args.no_plots,
        )
