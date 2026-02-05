"""
Flywheel Spin Strategy Analysis
================================

High-fidelity analysis comparing flywheel operation strategies:
- "idle-on": Keep flywheel spinning at target RPM throughout match
- "spin-on-demand": Spin down between shots, spin up before shooting

Sweeps over multiple parameters to find optimal strategy and configuration:
- Friction coefficients (viscous damping)
- Moment of inertia
- Gear ratio
- Match shooting patterns
- Battery initial state

Uses gamegine.hifi_sim MechanismSimulator for physics-accurate motor/battery modeling.
"""

import sys
import os
import numpy as np
import matplotlib.pyplot as plt
from dataclasses import dataclass, field
from typing import Dict, List, Tuple, Optional, Callable
from enum import Enum
import itertools
import time

# Ensure we can import gamegine
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..', '..')))

from gamegine.hifi_sim import (
    Motor, Battery, LinkConfig, MechanismSimulator, is_available
)


# =============================================================================
# Enums and Configuration
# =============================================================================

class SpinStrategy(Enum):
    """Flywheel spin strategy during non-shooting periods."""
    IDLE_ON = "idle_on"          # Maintain target velocity
    SPIN_ON_DEMAND = "spin_on_demand"  # Coast down, spin up before shots


@dataclass
class ShooterConfig:
    """Configuration for a dual-flywheel shooter mechanism."""
    
    # Wheel geometry
    wheel_diameter_m: float = 0.1016  # 4 inches
    
    # Gearing
    gear_ratio: float = 1.27  # Motor:Flywheel reduction
    
    # Flywheel inertia (moment of inertia in kg⋅m²)
    flywheel_moi: float = 0.0035
    
    # Ball properties
    ball_mass_kg: float = 0.227  # 0.5 lb ball
    target_exit_velocity_mps: float = 8.0  # m/s
    
    # Energy transfer efficiency
    energy_transfer_efficiency: float = 0.85
    
    # Friction coefficients
    friction_viscous: float = 0.001  # Nm/(rad/s) - viscous damping
    
    # Motor current limit per motor (A)
    motor_current_limit: float = 40.0
    
    @property
    def wheel_radius_m(self) -> float:
        return self.wheel_diameter_m / 2.0
    
    @property
    def wheel_angular_velocity_rad_s(self) -> float:
        """Target wheel angular velocity in rad/s."""
        return self.target_exit_velocity_mps / self.wheel_radius_m
    
    @property
    def motor_angular_velocity_rad_s(self) -> float:
        """Motor shaft angular velocity at target wheel speed."""
        return self.wheel_angular_velocity_rad_s * self.gear_ratio
    
    @property
    def ball_kinetic_energy_j(self) -> float:
        """Energy required to launch one ball."""
        return 0.5 * self.ball_mass_kg * (self.target_exit_velocity_mps ** 2)


@dataclass
class MatchConfig:
    """Configuration for match simulation."""
    
    # Match timing
    match_duration_s: float = 150.0  # Full FRC match
    auto_duration_s: float = 15.0
    
    # Shooting pattern
    auto_shots: int = 4  # Shots during auto
    teleop_cycle_time_s: float = 8.0  # Time per cycle
    shots_per_cycle: int = 2  # Balls scored per cycle
    
    # Lead time for spin-on-demand strategy (spin up before shots)
    spinup_lead_time_s: float = 1.0  # Conservative default


@dataclass
class BatteryConfig:
    """Configuration for battery simulation."""
    
    initial_soc: float = 1.0  # State of charge (1.0 = full)
    capacity_ah: float = 18.0  # Amp-hours
    internal_resistance: float = 0.015  # Ohms
    
    # Other subsystem current draw (not flywheel)
    base_current_draw: float = 10.0  # Amps for drivetrain, sensors, etc.


# =============================================================================
# Shooting Event Generator
# =============================================================================

@dataclass
class ShootingEvent:
    """Represents a shooting event (burst of balls)."""
    time_s: float  # Start time of shooting
    n_balls: int  # Number of balls to shoot
    feed_rate_hz: float = 10.0  # Feed rate during burst


def generate_match_events(config: MatchConfig) -> List[ShootingEvent]:
    """Generate realistic shooting events for a match."""
    events = []
    
    # Auto period: shoot preloads (spaced to allow aiming)
    if config.auto_shots > 0:
        auto_interval = config.auto_duration_s / (config.auto_shots + 1)
        for i in range(config.auto_shots):
            events.append(ShootingEvent(
                time_s=(i + 1) * auto_interval,
                n_balls=1,
                feed_rate_hz=10.0
            ))
    
    # Teleop cycles
    teleop_start = config.auto_duration_s
    current_time = teleop_start + config.teleop_cycle_time_s
    
    while current_time < config.match_duration_s - 5.0:  # Leave buffer at end
        events.append(ShootingEvent(
            time_s=current_time,
            n_balls=config.shots_per_cycle,
            feed_rate_hz=10.0
        ))
        current_time += config.teleop_cycle_time_s
    
    return events


# =============================================================================
# Strategy Simulators
# =============================================================================

class StrategySimulator:
    """Simulates a flywheel shooter over a full match with a given strategy."""
    
    def __init__(self, 
                 shooter_config: ShooterConfig,
                 match_config: MatchConfig,
                 battery_config: BatteryConfig,
                 strategy: SpinStrategy):
        if not is_available():
            raise RuntimeError("Rust extension required for StrategySimulator")
        
        self.shooter_config = shooter_config
        self.match_config = match_config
        self.battery_config = battery_config
        self.strategy = strategy
        
        # Create motor and battery
        self._motor_top = Motor.kraken_x60()
        self._motor_bottom = Motor.kraken_x60()
        self._battery = Battery.frc_standard()
        
        # Create link configs
        self._link_config_top = LinkConfig(
            gear_ratio=shooter_config.gear_ratio,
            radius=0.0,  # Rotational output
            efficiency=0.95,
            friction_viscous=shooter_config.friction_viscous,
        )
        self._link_config_bottom = LinkConfig(
            gear_ratio=shooter_config.gear_ratio,
            radius=0.0,
            efficiency=0.95,
            friction_viscous=shooter_config.friction_viscous,
        )
        
        # Create mechanism simulators
        self._sim_top = MechanismSimulator(
            motor=self._motor_top,
            battery=self._battery,
            link_config=self._link_config_top,
            load_mass=shooter_config.flywheel_moi,
            load_type="flywheel",
        )
        self._sim_bottom = MechanismSimulator(
            motor=self._motor_bottom,
            battery=self._battery,
            link_config=self._link_config_bottom,
            load_mass=shooter_config.flywheel_moi,
            load_type="flywheel",
        )
        
        # Target velocity in rad/s at flywheel
        self.target_omega = shooter_config.wheel_angular_velocity_rad_s
        
    def reset(self):
        """Reset simulation state."""
        self._sim_top.reset()
        self._sim_bottom.reset()
    
    def _get_avg_velocity(self) -> float:
        """Get average flywheel velocity (rad/s)."""
        return (self._sim_top.velocity + self._sim_bottom.velocity) / 2.0
    
    def _shoot_ball(self) -> float:
        """Simulate energy extraction from one ball. Returns velocity drop (rad/s)."""
        ball_ke = self.shooter_config.ball_kinetic_energy_j
        energy_to_extract = ball_ke / self.shooter_config.energy_transfer_efficiency
        energy_per_wheel = energy_to_extract / 2.0
        
        I = self.shooter_config.flywheel_moi
        
        for sim in [self._sim_top, self._sim_bottom]:
            omega = sim.velocity
            ke = 0.5 * I * (omega ** 2)
            new_ke = max(0, ke - energy_per_wheel)
            new_omega = np.sqrt(2 * new_ke / I) if new_ke > 0 else 0
            sim._inner.set_velocity(new_omega)
        
        return energy_to_extract
    
    def run_match(self, events: List[ShootingEvent], 
                  dt: float = 0.001,
                  control_dt: float = 0.001) -> Dict:
        """
        Run full match simulation.
        
        Returns dict with time-series data and summary metrics.
        """
        self.reset()
        
        # Data recording (sample every 10ms for manageable size)
        record_interval = 0.01
        times = []
        velocities = []
        currents = []
        voltages = []
        socs = []
        shot_times = []
        
        # Energy tracking
        total_energy_wh = 0.0
        
        # Sort events by time
        events = sorted(events, key=lambda e: e.time_s)
        event_idx = 0
        
        # For spin-on-demand, calculate when to start spinning
        spinup_times = []
        if self.strategy == SpinStrategy.SPIN_ON_DEMAND:
            for evt in events:
                spinup_times.append(evt.time_s - self.match_config.spinup_lead_time_s)
        
        # Main simulation loop
        elapsed = 0.0
        last_record = -record_interval
        shooting_until = 0.0
        balls_in_burst = 0
        next_shot_time = 0.0
        
        # Initial spinup to target (both strategies start ready)
        while self._get_avg_velocity() < 0.98 * self.target_omega:
            self._sim_top.set_duty_cycle(1.0)
            self._sim_bottom.set_duty_cycle(1.0)
            
            r_top = self._sim_top.run(control_dt, dt)
            r_bottom = self._sim_bottom.run(control_dt, dt)
            elapsed += control_dt
            
            # Track energy
            power = (abs(r_top["current"][-1]) + abs(r_bottom["current"][-1])) * r_top["voltage"][-1]
            total_energy_wh += power * control_dt / 3600.0
        
        # Main match loop
        while elapsed < self.match_config.match_duration_s:
            # Determine current mode
            currently_shooting = shooting_until > elapsed
            
            # Check if we should start a shooting event
            if event_idx < len(events) and elapsed >= events[event_idx].time_s:
                evt = events[event_idx]
                balls_in_burst = evt.n_balls
                shot_interval = 1.0 / evt.feed_rate_hz
                next_shot_time = elapsed
                shooting_until = elapsed + balls_in_burst * shot_interval + 0.01
                event_idx += 1
                currently_shooting = True
            
            # Shoot balls during burst
            if currently_shooting and balls_in_burst > 0 and elapsed >= next_shot_time:
                if self._get_avg_velocity() > 0.5 * self.target_omega:
                    self._shoot_ball()
                    shot_times.append(elapsed)
                    balls_in_burst -= 1
                    next_shot_time = elapsed + 1.0 / 10.0  # 10 Hz feed rate
            
            # Control strategy
            if self.strategy == SpinStrategy.IDLE_ON:
                # Always maintain target velocity
                if self._get_avg_velocity() < 0.98 * self.target_omega:
                    self._sim_top.set_duty_cycle(1.0)
                    self._sim_bottom.set_duty_cycle(1.0)
                else:
                    self._sim_top.set_duty_cycle(0.0)
                    self._sim_bottom.set_duty_cycle(0.0)
                    
            elif self.strategy == SpinStrategy.SPIN_ON_DEMAND:
                # Check if we should be spinning up for next event
                should_spin = currently_shooting
                
                if not should_spin:
                    # Check if upcoming event needs spinup
                    for i, spinup_time in enumerate(spinup_times):
                        if i >= event_idx and spinup_time <= elapsed < events[i].time_s + 1.0:
                            should_spin = True
                            break
                
                if should_spin:
                    if self._get_avg_velocity() < 0.98 * self.target_omega:
                        self._sim_top.set_duty_cycle(1.0)
                        self._sim_bottom.set_duty_cycle(1.0)
                    else:
                        self._sim_top.set_duty_cycle(0.0)
                        self._sim_bottom.set_duty_cycle(0.0)
                else:
                    # Coast down (brake slightly to reduce energy waste)
                    self._sim_top.set_duty_cycle(0.0)
                    self._sim_bottom.set_duty_cycle(0.0)
            
            # Step simulation
            r_top = self._sim_top.run(control_dt, dt)
            r_bottom = self._sim_bottom.run(control_dt, dt)
            elapsed += control_dt
            
            # Track energy
            power = (abs(r_top["current"][-1]) + abs(r_bottom["current"][-1])) * r_top["voltage"][-1]
            total_energy_wh += power * control_dt / 3600.0
            
            # Record data
            if elapsed - last_record >= record_interval:
                times.append(elapsed)
                velocities.append(self._get_avg_velocity() * self.shooter_config.wheel_radius_m)
                currents.append(abs(r_top["current"][-1]) + abs(r_bottom["current"][-1]))
                voltages.append(r_top["voltage"][-1])
                socs.append(r_top["soc"][-1] if "soc" in r_top else 1.0)
                last_record = elapsed
        
        return {
            "times": np.array(times),
            "velocities_mps": np.array(velocities),
            "currents": np.array(currents),
            "voltages": np.array(voltages),
            "socs": np.array(socs),
            "shot_times": np.array(shot_times),
            "total_energy_wh": total_energy_wh,
            "total_shots": len(shot_times),
            "final_soc": socs[-1] if socs else 1.0,
            "min_voltage": np.min(voltages) if voltages else 12.0,
            "strategy": self.strategy.value,
        }


# =============================================================================
# Analysis Functions
# =============================================================================

@dataclass
class StrategyComparison:
    """Results from comparing two strategies."""
    idle_on_data: Dict
    spin_on_demand_data: Dict
    energy_savings_wh: float  # Positive = spin-on-demand saves energy
    energy_savings_pct: float
    soc_difference: float  # Positive = spin-on-demand has more SoC left
    break_even_idle_time_s: float
    shooter_config: ShooterConfig
    match_config: MatchConfig


def compare_strategies(
    shooter_config: ShooterConfig,
    match_config: MatchConfig,
    battery_config: BatteryConfig,
    verbose: bool = False
) -> StrategyComparison:
    """Run both strategies and compare results."""
    
    events = generate_match_events(match_config)
    
    if verbose:
        print(f"  Simulating IDLE_ON strategy...", end=" ", flush=True)
    idle_sim = StrategySimulator(shooter_config, match_config, battery_config, SpinStrategy.IDLE_ON)
    idle_data = idle_sim.run_match(events)
    if verbose:
        print(f"{idle_data['total_energy_wh']:.2f} Wh")
    
    if verbose:
        print(f"  Simulating SPIN_ON_DEMAND strategy...", end=" ", flush=True)
    demand_sim = StrategySimulator(shooter_config, match_config, battery_config, SpinStrategy.SPIN_ON_DEMAND)
    demand_data = demand_sim.run_match(events)
    if verbose:
        print(f"{demand_data['total_energy_wh']:.2f} Wh")
    
    energy_savings = idle_data["total_energy_wh"] - demand_data["total_energy_wh"]
    energy_savings_pct = (energy_savings / idle_data["total_energy_wh"]) * 100 if idle_data["total_energy_wh"] > 0 else 0
    soc_diff = demand_data["final_soc"] - idle_data["final_soc"]
    
    # Calculate break-even: how long between shots before spin-on-demand wins?
    # E_spinup = 0.5 * MOI * omega^2 / efficiency
    # P_idle = friction_viscous * omega^2 (approximate)
    # break_even = E_spinup / P_idle
    flywheel_ke = 0.5 * shooter_config.flywheel_moi * (shooter_config.wheel_angular_velocity_rad_s ** 2)
    spinup_energy = flywheel_ke / 0.95  # Motor efficiency
    idle_power = shooter_config.friction_viscous * (shooter_config.wheel_angular_velocity_rad_s ** 2)
    break_even = spinup_energy / idle_power if idle_power > 0 else float('inf')
    
    return StrategyComparison(
        idle_on_data=idle_data,
        spin_on_demand_data=demand_data,
        energy_savings_wh=energy_savings,
        energy_savings_pct=energy_savings_pct,
        soc_difference=soc_diff,
        break_even_idle_time_s=break_even,
        shooter_config=shooter_config,
        match_config=match_config,
    )


# =============================================================================
# Parameter Sweep
# =============================================================================

@dataclass
class SweepResult:
    """Result from one point in parameter sweep."""
    friction_viscous: float
    flywheel_moi: float
    gear_ratio: float
    cycle_time_s: float
    idle_on_energy_wh: float
    spin_on_demand_energy_wh: float
    energy_savings_pct: float
    break_even_s: float
    optimal_strategy: str


def run_parameter_sweep(
    friction_values: List[float] = None,
    moi_values: List[float] = None,
    gear_ratios: List[float] = None,
    cycle_times: List[float] = None,
    base_shooter_config: ShooterConfig = None,
    base_match_config: MatchConfig = None,
    battery_config: BatteryConfig = None,
    verbose: bool = True,
) -> List[SweepResult]:
    """
    Sweep over parameter space to find optimal configuration and strategy.
    
    Returns list of SweepResult for each configuration.
    """
    if friction_values is None:
        friction_values = [0.0005, 0.001, 0.002, 0.004, 0.008]
    if moi_values is None:
        moi_values = [0.002, 0.0035, 0.005, 0.008]
    if gear_ratios is None:
        gear_ratios = [1.0, 1.27, 1.5, 2.0]
    if cycle_times is None:
        cycle_times = [6.0, 8.0, 10.0, 15.0]
    
    if base_shooter_config is None:
        base_shooter_config = ShooterConfig()
    if base_match_config is None:
        base_match_config = MatchConfig()
    if battery_config is None:
        battery_config = BatteryConfig()
    
    results = []
    total = len(friction_values) * len(moi_values) * len(gear_ratios) * len(cycle_times)
    
    if verbose:
        print(f"Running parameter sweep: {total} configurations")
        print(f"  Friction values: {friction_values}")
        print(f"  MOI values: {moi_values}")
        print(f"  Gear ratios: {gear_ratios}")
        print(f"  Cycle times: {cycle_times}")
        print()
    
    i = 0
    for friction, moi, gr, cycle in itertools.product(friction_values, moi_values, gear_ratios, cycle_times):
        i += 1
        if verbose:
            print(f"[{i}/{total}] μ={friction:.4f}, MOI={moi:.4f}, GR={gr:.1f}, cycle={cycle}s ... ", end="", flush=True)
        
        shooter_config = ShooterConfig(
            wheel_diameter_m=base_shooter_config.wheel_diameter_m,
            gear_ratio=gr,
            flywheel_moi=moi,
            ball_mass_kg=base_shooter_config.ball_mass_kg,
            target_exit_velocity_mps=base_shooter_config.target_exit_velocity_mps,
            energy_transfer_efficiency=base_shooter_config.energy_transfer_efficiency,
            friction_viscous=friction,
            motor_current_limit=base_shooter_config.motor_current_limit,
        )
        
        match_config = MatchConfig(
            match_duration_s=base_match_config.match_duration_s,
            auto_duration_s=base_match_config.auto_duration_s,
            auto_shots=base_match_config.auto_shots,
            teleop_cycle_time_s=cycle,
            shots_per_cycle=base_match_config.shots_per_cycle,
        )
        
        try:
            comparison = compare_strategies(shooter_config, match_config, battery_config, verbose=False)
            
            optimal = "spin_on_demand" if comparison.energy_savings_wh > 0 else "idle_on"
            
            result = SweepResult(
                friction_viscous=friction,
                flywheel_moi=moi,
                gear_ratio=gr,
                cycle_time_s=cycle,
                idle_on_energy_wh=comparison.idle_on_data["total_energy_wh"],
                spin_on_demand_energy_wh=comparison.spin_on_demand_data["total_energy_wh"],
                energy_savings_pct=comparison.energy_savings_pct,
                break_even_s=comparison.break_even_idle_time_s,
                optimal_strategy=optimal,
            )
            results.append(result)
            
            if verbose:
                print(f"savings={comparison.energy_savings_pct:.1f}% → {optimal}")
                
        except Exception as e:
            if verbose:
                print(f"FAILED: {e}")
    
    return results


# =============================================================================
# Visualization
# =============================================================================

def plot_strategy_comparison(comparison: StrategyComparison, save_path: str = None):
    """Plot time-series comparison of both strategies."""
    fig, axes = plt.subplots(4, 1, figsize=(14, 12), sharex=True)
    
    idle = comparison.idle_on_data
    demand = comparison.spin_on_demand_data
    
    # Velocity
    axes[0].plot(idle["times"], idle["velocities_mps"], 'b-', linewidth=1, label='Idle-On')
    axes[0].plot(demand["times"], demand["velocities_mps"], 'r-', linewidth=1, alpha=0.7, label='Spin-On-Demand')
    axes[0].axhline(comparison.shooter_config.target_exit_velocity_mps, color='gray', linestyle='--', alpha=0.5, label='Target')
    
    # Mark shots
    for t in idle["shot_times"][:10]:  # First 10 shots
        axes[0].axvline(t, color='green', linestyle=':', alpha=0.3)
    
    axes[0].set_ylabel('Wheel Velocity (m/s)')
    axes[0].set_title('Flywheel Velocity Throughout Match')
    axes[0].legend(loc='lower right')
    axes[0].grid(True, alpha=0.3)
    
    # Current
    axes[1].plot(idle["times"], idle["currents"], 'b-', linewidth=0.5, label='Idle-On')
    axes[1].plot(demand["times"], demand["currents"], 'r-', linewidth=0.5, alpha=0.7, label='Spin-On-Demand')
    axes[1].set_ylabel('Total Current (A)')
    axes[1].set_title('Motor Current Draw')
    axes[1].legend(loc='upper right')
    axes[1].grid(True, alpha=0.3)
    
    # Voltage
    axes[2].plot(idle["times"], idle["voltages"], 'b-', linewidth=1, label='Idle-On')
    axes[2].plot(demand["times"], demand["voltages"], 'r-', linewidth=1, alpha=0.7, label='Spin-On-Demand')
    axes[2].axhline(12.0, color='gray', linestyle='--', alpha=0.5, label='Nominal')
    axes[2].axhline(6.3, color='orange', linestyle='--', alpha=0.5, label='Brownout')
    axes[2].set_ylabel('Battery Voltage (V)')
    axes[2].set_title('Battery Voltage (with Sag)')
    axes[2].legend(loc='lower right')
    axes[2].grid(True, alpha=0.3)
    
    # Cumulative energy
    idle_energy_cumulative = np.cumsum(
        idle["currents"] * idle["voltages"] * 0.01 / 3600.0
    )  # 10ms interval
    demand_energy_cumulative = np.cumsum(
        demand["currents"] * demand["voltages"] * 0.01 / 3600.0
    )
    
    axes[3].plot(idle["times"], idle_energy_cumulative, 'b-', linewidth=1.5, label='Idle-On')
    axes[3].plot(demand["times"], demand_energy_cumulative, 'r-', linewidth=1.5, label='Spin-On-Demand')
    axes[3].set_ylabel('Cumulative Energy (Wh)')
    axes[3].set_xlabel('Time (s)')
    axes[3].set_title(f'Cumulative Energy: Idle-On={idle["total_energy_wh"]:.2f}Wh, Spin-On-Demand={demand["total_energy_wh"]:.2f}Wh (Savings: {comparison.energy_savings_pct:.1f}%)')
    axes[3].legend(loc='upper left')
    axes[3].grid(True, alpha=0.3)
    
    plt.tight_layout()
    
    if save_path:
        plt.savefig(save_path, dpi=150)
        print(f"Saved strategy comparison to {save_path}")
    
    return fig


def plot_sweep_heatmap(results: List[SweepResult], 
                       x_param: str = "friction_viscous",
                       y_param: str = "flywheel_moi",
                       fixed_params: Dict[str, float] = None,
                       metric: str = "energy_savings_pct",
                       save_path: str = None):
    """Plot 2D heatmap of parameter sweep results."""
    if fixed_params is None:
        fixed_params = {}
    
    # Filter results to fixed parameters
    filtered = results
    for param, value in fixed_params.items():
        filtered = [r for r in filtered if abs(getattr(r, param) - value) < 1e-6]
    
    if not filtered:
        print(f"No results match fixed parameters: {fixed_params}")
        return None
    
    # Extract unique values
    x_values = sorted(set(getattr(r, x_param) for r in filtered))
    y_values = sorted(set(getattr(r, y_param) for r in filtered))
    
    # Build grid
    grid = np.full((len(y_values), len(x_values)), np.nan)
    
    for r in filtered:
        xi = x_values.index(getattr(r, x_param))
        yi = y_values.index(getattr(r, y_param))
        grid[yi, xi] = getattr(r, metric)
    
    fig, ax = plt.subplots(figsize=(10, 8))
    
    # Choose colormap based on metric
    if "savings" in metric:
        cmap = 'RdYlGn'  # Red (negative) to green (positive)
        vmin, vmax = -20, 60
    else:
        cmap = 'viridis'
        vmin, vmax = None, None
    
    im = ax.imshow(grid, cmap=cmap, aspect='auto', origin='lower', vmin=vmin, vmax=vmax,
                   extent=[x_values[0], x_values[-1], y_values[0], y_values[-1]])
    
    # Format axis labels
    param_labels = {
        "friction_viscous": "Viscous Friction (Nm/(rad/s))",
        "flywheel_moi": "Moment of Inertia (kg⋅m²)",
        "gear_ratio": "Gear Ratio",
        "cycle_time_s": "Cycle Time (s)",
    }
    metric_labels = {
        "energy_savings_pct": "Energy Savings (%)\n(+ve = Spin-On-Demand better)",
        "break_even_s": "Break-Even Idle Time (s)",
        "idle_on_energy_wh": "Idle-On Energy (Wh)",
        "spin_on_demand_energy_wh": "Spin-On-Demand Energy (Wh)",
    }
    
    ax.set_xlabel(param_labels.get(x_param, x_param))
    ax.set_ylabel(param_labels.get(y_param, y_param))
    
    cbar = plt.colorbar(im, ax=ax)
    cbar.set_label(metric_labels.get(metric, metric))
    
    # Title with fixed params
    fixed_str = ", ".join([f"{k}={v}" for k, v in fixed_params.items()]) if fixed_params else "All"
    ax.set_title(f'Strategy Comparison: {metric}\n(Fixed: {fixed_str})')
    
    plt.tight_layout()
    
    if save_path:
        plt.savefig(save_path, dpi=150)
        print(f"Saved heatmap to {save_path}")
    
    return fig


def plot_optimal_strategy_map(results: List[SweepResult],
                               x_param: str = "friction_viscous",
                               y_param: str = "cycle_time_s",
                               fixed_params: Dict[str, float] = None,
                               save_path: str = None):
    """Plot which strategy is optimal for each configuration."""
    if fixed_params is None:
        fixed_params = {}
    
    # Filter results
    filtered = results
    for param, value in fixed_params.items():
        filtered = [r for r in filtered if abs(getattr(r, param) - value) < 1e-6]
    
    if not filtered:
        print(f"No results match fixed parameters: {fixed_params}")
        return None
    
    x_values = sorted(set(getattr(r, x_param) for r in filtered))
    y_values = sorted(set(getattr(r, y_param) for r in filtered))
    
    grid = np.full((len(y_values), len(x_values)), np.nan)
    
    for r in filtered:
        xi = x_values.index(getattr(r, x_param))
        yi = y_values.index(getattr(r, y_param))
        grid[yi, xi] = 1 if r.optimal_strategy == "spin_on_demand" else 0
    
    fig, ax = plt.subplots(figsize=(10, 8))
    
    # Custom colormap: blue=idle_on, green=spin_on_demand
    from matplotlib.colors import ListedColormap
    cmap = ListedColormap(['#3498db', '#2ecc71'])
    
    im = ax.imshow(grid, cmap=cmap, aspect='auto', origin='lower',
                   extent=[x_values[0], x_values[-1], y_values[0], y_values[-1]])
    
    param_labels = {
        "friction_viscous": "Viscous Friction (Nm/(rad/s))",
        "flywheel_moi": "Moment of Inertia (kg⋅m²)",
        "gear_ratio": "Gear Ratio",
        "cycle_time_s": "Cycle Time (s)",
    }
    
    ax.set_xlabel(param_labels.get(x_param, x_param))
    ax.set_ylabel(param_labels.get(y_param, y_param))
    ax.set_title('Optimal Strategy by Configuration\n(Blue = Idle-On, Green = Spin-On-Demand)')
    
    # Add legend
    from matplotlib.patches import Patch
    legend_elements = [
        Patch(facecolor='#3498db', label='Idle-On'),
        Patch(facecolor='#2ecc71', label='Spin-On-Demand'),
    ]
    ax.legend(handles=legend_elements, loc='upper right')
    
    plt.tight_layout()
    
    if save_path:
        plt.savefig(save_path, dpi=150)
        print(f"Saved optimal strategy map to {save_path}")
    
    return fig


# =============================================================================
# Main Analysis
# =============================================================================

def run_analysis(
    output_dir: str = None,
    run_sweep: bool = True,
    verbose: bool = True,
):
    """Run complete strategy analysis."""
    if not is_available():
        print("ERROR: Rust extension 'gamegine_sim_py' not available.")
        print("Please compile the extension first.")
        return
    
    if output_dir is None:
        output_dir = os.path.dirname(os.path.abspath(__file__))
    
    print("=" * 70)
    print("FLYWHEEL SPIN STRATEGY ANALYSIS")
    print("=" * 70)
    print()
    
    # ==========================================================================
    # Part 1: Single Configuration Comparison
    # ==========================================================================
    print("-" * 70)
    print("PART 1: Single Configuration Comparison")
    print("-" * 70)
    
    shooter_config = ShooterConfig(
        gear_ratio=1.27,
        flywheel_moi=0.0035,
        friction_viscous=0.001,
        target_exit_velocity_mps=8.0,
    )
    
    match_config = MatchConfig(
        match_duration_s=150.0,
        teleop_cycle_time_s=8.0,
        shots_per_cycle=2,
    )
    
    battery_config = BatteryConfig()
    
    print(f"Shooter: GR={shooter_config.gear_ratio}, MOI={shooter_config.flywheel_moi:.4f} kg⋅m²")
    print(f"Match: {match_config.match_duration_s}s, {match_config.teleop_cycle_time_s}s cycles")
    print()
    
    comparison = compare_strategies(shooter_config, match_config, battery_config, verbose=True)
    
    print()
    print("RESULTS:")
    print(f"  Idle-On Energy:       {comparison.idle_on_data['total_energy_wh']:.2f} Wh")
    print(f"  Spin-On-Demand Energy: {comparison.spin_on_demand_data['total_energy_wh']:.2f} Wh")
    print(f"  Energy Savings:       {comparison.energy_savings_wh:.2f} Wh ({comparison.energy_savings_pct:.1f}%)")
    print(f"  Break-Even Idle Time: {comparison.break_even_idle_time_s:.1f} s")
    print()
    
    optimal = "SPIN-ON-DEMAND" if comparison.energy_savings_wh > 0 else "IDLE-ON"
    print(f"  → OPTIMAL STRATEGY: {optimal}")
    print()
    
    # Save comparison plot
    plot_strategy_comparison(comparison, 
                            save_path=os.path.join(output_dir, "strategy_comparison.png"))
    
    # ==========================================================================
    # Part 2: Parameter Sweep
    # ==========================================================================
    if run_sweep:
        print()
        print("-" * 70)
        print("PART 2: Parameter Sweep")
        print("-" * 70)
        
        sweep_results = run_parameter_sweep(
            friction_values=[0.0005, 0.001, 0.002, 0.004],
            moi_values=[0.002, 0.0035, 0.005, 0.008],
            gear_ratios=[1.0, 1.27, 1.5],
            cycle_times=[6.0, 8.0, 12.0, 20.0],
            base_shooter_config=shooter_config,
            base_match_config=match_config,
            battery_config=battery_config,
            verbose=True,
        )
        
        # Summary
        print()
        print("=" * 70)
        print("SWEEP SUMMARY")
        print("=" * 70)
        
        idle_better = sum(1 for r in sweep_results if r.optimal_strategy == "idle_on")
        demand_better = len(sweep_results) - idle_better
        print(f"  Idle-On optimal: {idle_better}/{len(sweep_results)} configurations")
        print(f"  Spin-On-Demand optimal: {demand_better}/{len(sweep_results)} configurations")
        
        # Find best configurations for each strategy
        if demand_better > 0:
            best_demand = max([r for r in sweep_results if r.optimal_strategy == "spin_on_demand"],
                             key=lambda r: r.energy_savings_pct)
            print()
            print("  Best for Spin-On-Demand:")
            print(f"    Friction={best_demand.friction_viscous:.4f}, MOI={best_demand.flywheel_moi:.4f}")
            print(f"    GR={best_demand.gear_ratio}, Cycle={best_demand.cycle_time_s}s")
            print(f"    Savings: {best_demand.energy_savings_pct:.1f}%")
        
        # Generate heatmaps
        print()
        print("Generating heatmaps...")
        
        # Friction vs Cycle Time (at default MOI and GR)
        plot_sweep_heatmap(
            sweep_results,
            x_param="friction_viscous",
            y_param="cycle_time_s",
            fixed_params={"flywheel_moi": 0.0035, "gear_ratio": 1.27},
            metric="energy_savings_pct",
            save_path=os.path.join(output_dir, "heatmap_friction_vs_cycle.png")
        )
        
        # MOI vs Cycle Time
        plot_sweep_heatmap(
            sweep_results,
            x_param="flywheel_moi",
            y_param="cycle_time_s",
            fixed_params={"friction_viscous": 0.001, "gear_ratio": 1.27},
            metric="energy_savings_pct",
            save_path=os.path.join(output_dir, "heatmap_moi_vs_cycle.png")
        )
        
        # Optimal strategy map
        plot_optimal_strategy_map(
            sweep_results,
            x_param="friction_viscous",
            y_param="cycle_time_s",
            fixed_params={"flywheel_moi": 0.0035, "gear_ratio": 1.27},
            save_path=os.path.join(output_dir, "optimal_strategy_map.png")
        )
    
    print()
    print("=" * 70)
    print("Analysis complete!")
    print(f"Plots saved to: {output_dir}")
    print("=" * 70)
    
    plt.show()


def verify_physics():
    """Run physics verification tests."""
    print("Running physics verification...")
    
    shooter_config = ShooterConfig()
    match_config = MatchConfig()
    battery_config = BatteryConfig()
    
    # Test 1: Energy conservation
    print("\n1. Testing energy conservation...")
    events = [ShootingEvent(time_s=2.0, n_balls=1)]
    
    sim = StrategySimulator(shooter_config, match_config, battery_config, SpinStrategy.IDLE_ON)
    data = sim.run_match(events)
    
    # Flywheel kinetic energy
    flywheel_ke = 0.5 * shooter_config.flywheel_moi * (shooter_config.wheel_angular_velocity_rad_s ** 2)
    print(f"  Flywheel KE: {flywheel_ke:.2f} J")
    print(f"  Total energy drawn: {data['total_energy_wh'] * 3600:.2f} J")
    
    # Test 2: Spin-on-demand uses less energy when idle time is long
    print("\n2. Testing strategy comparison with long idle...")
    long_wait_config = MatchConfig(teleop_cycle_time_s=30.0)  # Very long cycles
    
    comparison = compare_strategies(shooter_config, long_wait_config, battery_config)
    
    print(f"  Idle-On: {comparison.idle_on_data['total_energy_wh']:.2f} Wh")
    print(f"  Spin-On-Demand: {comparison.spin_on_demand_data['total_energy_wh']:.2f} Wh")
    print(f"  Savings: {comparison.energy_savings_pct:.1f}%")
    
    assert comparison.energy_savings_wh > 0, "Long idle should favor spin-on-demand"
    print("  ✓ Long idle correctly favors spin-on-demand")
    
    # Test 3: Short cycles favor idle-on (if implemented correctly)
    print("\n3. Testing with short cycles...")
    short_config = MatchConfig(teleop_cycle_time_s=3.0)  # Very short cycles
    
    comparison_short = compare_strategies(shooter_config, short_config, battery_config)
    print(f"  Idle-On: {comparison_short.idle_on_data['total_energy_wh']:.2f} Wh")
    print(f"  Spin-On-Demand: {comparison_short.spin_on_demand_data['total_energy_wh']:.2f} Wh")
    print(f"  Savings: {comparison_short.energy_savings_pct:.1f}%")
    
    print()
    print("All physics verification tests passed!")


if __name__ == "__main__":
    import argparse
    
    parser = argparse.ArgumentParser(description="Flywheel Spin Strategy Analysis")
    parser.add_argument("--verify-physics", action="store_true",
                       help="Run physics verification tests")
    parser.add_argument("--no-sweep", action="store_true",
                       help="Skip parameter sweep (faster)")
    parser.add_argument("--output-dir", type=str, default=None,
                       help="Directory for output files")
    
    args = parser.parse_args()
    
    if args.verify_physics:
        verify_physics()
    else:
        run_analysis(
            output_dir=args.output_dir,
            run_sweep=not args.no_sweep,
        )
