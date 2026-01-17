"""Robot Speed Analysis - Cycle time analysis across gear ratios and weights

Analyzes cycle speeds for different Thrifty Swerve gear ratios and robot weights
across two strategic cycle routes on the REBUILT field.

Routes tested:
1. Start (trench line) → Neutral Zone (center) → HUB
2. Start → Depot → HUB → Tower
"""

import sys
sys.path.insert(0, '.')

from typing import List, Tuple
from dataclasses import dataclass

from examples.Rebuilt.Rebuilt import (
    create_rebuilt_game, TRENCH_LINE, HALF_LENGTH, HALF_WIDTH,
    TOWER_OFFSET_FROM_WALL, BUMP_WIDTH, FIELD_LENGTH
)

from gamegine.representation.robot import SwerveRobot, PhysicalParameters
from gamegine.reference import gearing, motors
from gamegine.reference.gearing import ThriftySwerve
from gamegine.reference.swerve import SwerveConfig, SwerveModule
from gamegine.simulation.GameServer import DiscreteGameServer, ServerConfig
from gamegine.simulation.robot import RobotState
from gamegine.representation.bounds import Rectangle
from gamegine.representation.zone import TraversalZone
from gamegine.utils.NCIM.Dimensions.spatial import Meter, Feet, Inch
from gamegine.utils.NCIM.Dimensions.angular import Degree
from gamegine.utils.NCIM.Dimensions.mass import Pound
from gamegine.utils.NCIM.Dimensions.temporal import Second
from gamegine.utils.NCIM.ComplexDimensions.acceleration import MeterPerSecondSquared
from gamegine.utils.NCIM.ComplexDimensions.electricpot import Volt
from gamegine.utils.NCIM.ncim import Ampere

from gamegine.render import Renderer, DisplayLevel
from gamegine.reference.battery import BatteryModel
import arcade


# =============================================================================
# CONFIGURATION
# =============================================================================

ROBOT_WIDTH = Inch(30)
ROBOT_HEIGHT = Inch(20)  # Short enough to pass under bars

# Weight range to test (100lb to 150lb in 10lb increments)
WEIGHTS = [Pound(w) for w in range(100+20, 115+20, 3)]

# All Thrifty Swerve gear ratio combinations
GEAR_CONFIGS = [
    (ThriftySwerve.Pinions.P12, ThriftySwerve.Stage2.S18, "P12-S18 (6.75:1)"),
    (ThriftySwerve.Pinions.P12, ThriftySwerve.Stage2.S16, "P12-S16 (6.00:1)"),
    (ThriftySwerve.Pinions.P13, ThriftySwerve.Stage2.S18, "P13-S18 (6.23:1)"),
    (ThriftySwerve.Pinions.P13, ThriftySwerve.Stage2.S16, "P13-S16 (5.54:1)"),
    (ThriftySwerve.Pinions.P14, ThriftySwerve.Stage2.S18, "P14-S18 (5.79:1)"),
    (ThriftySwerve.Pinions.P14, ThriftySwerve.Stage2.S16, "P14-S16 (5.14:1)"),
]


@dataclass
class AnalysisResult:
    """Result of a single analysis run."""
    gear_name: str
    weight_lb: float
    route_name: str
    total_time: float  # seconds
    distance: float    # meters
    avg_speed: float   # m/s


# =============================================================================
# ROUTE DEFINITIONS
# =============================================================================

# Route waypoints as (x, y, heading)
START_POS = (TRENCH_LINE - BUMP_WIDTH, Feet(3), Degree(0))

ROUTE_1_WAYPOINTS = [
    (HALF_LENGTH, HALF_WIDTH, Degree(0)),           # Neutral zone
    (TRENCH_LINE - BUMP_WIDTH - Feet(3), HALF_WIDTH, Degree(0)), # Blue HUB
]

ROUTE_2_WAYPOINTS = [
    (Feet(4), HALF_WIDTH, Degree(180)),             # Blue Depot
    (TRENCH_LINE - BUMP_WIDTH - Feet(3), HALF_WIDTH, Degree(0)), # Blue HUB
    (TOWER_OFFSET_FROM_WALL, HALF_WIDTH, Degree(180)), # Blue Tower
]

# Red HUB is on opposite side of field
RED_HUB_POS = (FIELD_LENGTH - TRENCH_LINE + BUMP_WIDTH + Feet(3), HALF_WIDTH, Degree(180))
BLUE_HUB_POS = (TRENCH_LINE - BUMP_WIDTH - Feet(3), HALF_WIDTH, Degree(0))

ROUTE_3_WAYPOINTS = [
    RED_HUB_POS,  # Red HUB (other side)
    BLUE_HUB_POS,  # Blue HUB (back to left side)
]


# =============================================================================
# ROBOT FACTORY
# =============================================================================

def create_swerve_config(pinion, stage2):
    """Create swerve config with specified Thrifty gear ratio."""
    gear_ratio = ThriftySwerve.get_gear_ratio(pinion, stage2)
    
    return SwerveConfig(
        SwerveModule(
            motors.MotorConfig(
                motors.KrakenX60,
                motors.PowerConfig(Ampere(40), Ampere(240), 1.0),
            ),
            gear_ratio,
            motors.MotorConfig(
                motors.KrakenX60,
                motors.PowerConfig(Ampere(40), Ampere(240), 1.0),
            ),
            gearing.MK4I.L3,  # Steer uses standard ratio
        )
    )


def create_robot(name: str, weight: Pound, pinion, stage2) -> SwerveRobot:
    """Create a swerve robot with specified weight and gearing."""
    structure = [
        Rectangle.from_center((Inch(0), Inch(0)), ROBOT_WIDTH, ROBOT_WIDTH).get_3d(
            Inch(0), ROBOT_HEIGHT
        )
    ]
    
    robot = SwerveRobot(
        name=name,
        drivetrain=create_swerve_config(pinion, stage2),
        structure=structure,
        physics=PhysicalParameters(
            mass=weight,
            moi=weight * Inch(15) ** 2,
            max_acceleration=MeterPerSecondSquared(7.0),
        ),
    )
    robot.override_bounding_radius(Inch(16))
    return robot


# =============================================================================
# CYCLE TIME CALCULATION
# =============================================================================

def calculate_route_time(game, robot, start_pos, waypoints, battery_model=None):
    """Calculate total time to traverse a route through waypoints.
    
    :param battery_model: Optional BatteryModel for voltage-aware acceleration limiting.
                          If None, creates a fresh FRC standard battery for each run.
    """
    server = DiscreteGameServer(ServerConfig())
    server.load_from_game(game)
    server.add_robot(robot)
    server.init_robot(robot.name, RobotState(*start_pos))
    
    # Use provided battery or create fresh standard FRC battery
    battery = battery_model if battery_model else BatteryModel.frc_standard()
    
    total_time = 0.0
    total_distance = 0.0
    current_pos = start_pos
    
    for target in waypoints:
        # Prepare traversal space
        traversal_space = server.physics_engine.prepare_traversal_space(
            robot.name,
            robot,
            list(game.get_obstacles()),
            game.get_field_size(),
        )
        
        # Find path
        path = server.physics_engine.pathfind(
            robot.name,
            current_pos[0], current_pos[1],
            target[0], target[1],
            traversal_space,
        )
        
        # Generate trajectory with speed zones and battery simulation
        trajectory = server.physics_engine.generate_trajectory(
            robot.name,
            robot,
            current_pos,
            target,
            path,
            traversal_space,
            speed_zones=game.get_zones(),
            battery_model=battery,
        )
        
        leg_time = float(trajectory.get_travel_time().to(Second))
        total_time += leg_time
        
        # Approximate distance from path
        points = path.get_points() if path else []
        for i in range(len(points) - 1):
            dx = float(points[i+1][0].to(Meter) - points[i][0].to(Meter))
            dy = float(points[i+1][1].to(Meter) - points[i][1].to(Meter))
            total_distance += (dx**2 + dy**2)**0.5
        
        current_pos = target
    
    return total_time, total_distance


# =============================================================================
# ANALYSIS RUNNER
# =============================================================================

def run_analysis():
    """Run full analysis across all gear ratios and weights."""
    print("=" * 70)
    print("ROBOT SPEED ANALYSIS - ThriftySwerve Gear Ratios vs Weight")
    print("=" * 70)
    
    game = create_rebuilt_game()
    results: List[AnalysisResult] = []
    
    print(f"\nField: {game.name}")
    print(f"Testing {len(GEAR_CONFIGS)} gear configurations × {len(WEIGHTS)} weights = {len(GEAR_CONFIGS) * len(WEIGHTS)} combinations")
    print(f"Routes: 3 (Start→Center→HUB, Start→Depot→HUB→Tower, Start→RedHUB→BlueHUB)")
    
    # Print theoretical top speed and acceleration table
    print("\n" + "=" * 70)
    print("THEORETICAL PERFORMANCE (Top Speed & Max Acceleration)")
    print("=" * 70)
    
    from gamegine.utils.NCIM.ComplexDimensions.velocity import MetersPerSecond
    from gamegine.utils.NCIM.Dimensions.mass import Kilogram
    
    for pinion, stage2, gear_name in GEAR_CONFIGS:
        robot = create_robot("TestBot", WEIGHTS[0], pinion, stage2)
        module = robot.drivetrain.module
        
        # Top speed = wheel angular velocity * wheel radius
        max_wheel_omega = module.get_max_speed()
        wheel_radius = module.wheel.diameter / 2
        top_speed_mps = float(max_wheel_omega) * float(wheel_radius.to(Meter))
        
        print(f"\n{gear_name}:")
        print(f"  Top Speed: {top_speed_mps:.2f} m/s ({top_speed_mps * 3.28084:.1f} ft/s)")
        
        # Max acceleration = (4 modules * max torque / wheel radius) / mass
        max_torque = module.get_max_torque()
        
        for weight in WEIGHTS:
            mass_kg = float(weight.to(Kilogram))
            # Force = Torque / radius, 4 modules
            force_n = 4 * float(max_torque) / float(wheel_radius.to(Meter))
            max_accel = force_n / mass_kg
            print(f"    @ {float(weight.to(Pound)):>3.0f}lb: Max Accel = {max_accel:.2f} m/s²")
    
    print()
    
    # Run analysis for each configuration
    for pinion, stage2, gear_name in GEAR_CONFIGS:
        gear_ratio = ThriftySwerve.get_gear_ratio(pinion, stage2)
        print(f"\n--- {gear_name} ---")
        
        for weight in WEIGHTS:
            robot = create_robot("TestBot", weight, pinion, stage2)
            
            # Route 1: Start → Neutral Zone → HUB
            time1, dist1 = calculate_route_time(game, robot, START_POS, ROUTE_1_WAYPOINTS)
            results.append(AnalysisResult(
                gear_name=gear_name,
                weight_lb=float(weight.to(Pound)),
                route_name="Route 1",
                total_time=time1,
                distance=dist1,
                avg_speed=dist1/time1 if time1 > 0 else 0
            ))
            
            # Route 2: Start → Depot → HUB → Tower
            time2, dist2 = calculate_route_time(game, robot, START_POS, ROUTE_2_WAYPOINTS)
            results.append(AnalysisResult(
                gear_name=gear_name,
                weight_lb=float(weight.to(Pound)),
                route_name="Route 2",
                total_time=time2,
                distance=dist2,
                avg_speed=dist2/time2 if time2 > 0 else 0
            ))
            
            # Route 3: Start → Red HUB → Blue HUB
            time3, dist3 = calculate_route_time(game, robot, START_POS, ROUTE_3_WAYPOINTS)
            results.append(AnalysisResult(
                gear_name=gear_name,
                weight_lb=float(weight.to(Pound)),
                route_name="Route 3",
                total_time=time3,
                distance=dist3,
                avg_speed=dist3/time3 if time3 > 0 else 0
            ))
            
            print(f"  {weight}: R1={time1:.2f}s, R2={time2:.2f}s, R3={time3:.2f}s")
    
    return results, game


def print_results_table(results: List[AnalysisResult]):
    """Print formatted results table."""
    print("\n" + "=" * 70)
    print("RESULTS SUMMARY - Route 1 (Start → Center → HUB)")
    print("=" * 70)
    
    # Header
    print(f"{'Weight':<10}", end="")
    for _, _, gear_name in GEAR_CONFIGS:
        print(f"{gear_name:>10}", end="")
    print()
    print("-" * 70)
    
    # Route 1 data
    for weight in WEIGHTS:
        print(f"{float(weight.to(Pound)):>6.0f}lb  ", end="")
        for _, _, gear_name in GEAR_CONFIGS:
            r = next((r for r in results 
                     if r.gear_name == gear_name 
                     and r.weight_lb == float(weight.to(Pound))
                     and r.route_name == "Route 1"), None)
            if r:
                print(f"{r.total_time:>9.2f}s", end="")
        print()
    
    print("\n" + "=" * 70)
    print("RESULTS SUMMARY - Route 2 (Start → Depot → HUB → Tower)")
    print("=" * 70)
    
    # Header
    print(f"{'Weight':<10}", end="")
    for _, _, gear_name in GEAR_CONFIGS:
        print(f"{gear_name:>10}", end="")
    print()
    print("-" * 70)
    
    # Route 2 data
    for weight in WEIGHTS:
        print(f"{float(weight.to(Pound)):>6.0f}lb  ", end="")
        for _, _, gear_name in GEAR_CONFIGS:
            r = next((r for r in results 
                     if r.gear_name == gear_name 
                     and r.weight_lb == float(weight.to(Pound))
                     and r.route_name == "Route 2"), None)
            if r:
                print(f"{r.total_time:>9.2f}s", end="")
        print()
    
    print("\n" + "=" * 70)
    print("RESULTS SUMMARY - Route 3 (Start → Red HUB → Blue HUB)")
    print("=" * 70)
    
    # Header
    print(f"{'Weight':<10}", end="")
    for _, _, gear_name in GEAR_CONFIGS:
        print(f"{gear_name:>10}", end="")
    print()
    print("-" * 70)
    
    # Route 3 data
    for weight in WEIGHTS:
        print(f"{float(weight.to(Pound)):>6.0f}lb  ", end="")
        for _, _, gear_name in GEAR_CONFIGS:
            r = next((r for r in results 
                     if r.gear_name == gear_name 
                     and r.weight_lb == float(weight.to(Pound))
                     and r.route_name == "Route 3"), None)
            if r:
                print(f"{r.total_time:>9.2f}s", end="")
        print()
    
    # Find optimal
    route1_results = [r for r in results if r.route_name == "Route 1"]
    route2_results = [r for r in results if r.route_name == "Route 2"]
    route3_results = [r for r in results if r.route_name == "Route 3"]
    
    best_r1 = min(route1_results, key=lambda r: r.total_time)
    best_r2 = min(route2_results, key=lambda r: r.total_time)
    best_r3 = min(route3_results, key=lambda r: r.total_time)
    
    print("\n" + "=" * 70)
    print("OPTIMAL CONFIGURATIONS")
    print("=" * 70)
    print(f"Route 1 Best: {best_r1.gear_name} @ {best_r1.weight_lb:.0f}lb → {best_r1.total_time:.2f}s")
    print(f"Route 2 Best: {best_r2.gear_name} @ {best_r2.weight_lb:.0f}lb → {best_r2.total_time:.2f}s")
    print(f"Route 3 Best: {best_r3.gear_name} @ {best_r3.weight_lb:.0f}lb → {best_r3.total_time:.2f}s")
    
    return best_r1, best_r2, best_r3


# =============================================================================
# VISUALIZATION
# =============================================================================

def draw_zone(zone, canvas, theme, display_level, renderer=None):
    """Draw a traversal zone as a semi-transparent colored region."""
    bounds = zone.boundary
    if isinstance(bounds, Rectangle):
        x = canvas.to_pixels(bounds.x)
        y = canvas.to_pixels(bounds.y)
        w = canvas.to_pixels(bounds.width)
        h = canvas.to_pixels(bounds.height)
        
        if zone.speed_multiplier < 0.6:
            color = (255, 165, 0, 80)  # Orange - slow
        elif zone.speed_multiplier < 1.0:
            color = (255, 255, 0, 60)  # Yellow - medium
        else:
            color = (100, 255, 100, 60)  # Green - fast
        
        points = [(x, y), (x + w, y), (x + w, y + h), (x, y + h)]
        arcade.draw_polygon_filled(points, color)
        arcade.draw_text(
            f"{zone.name}\n{int(zone.speed_multiplier*100)}% speed",
            x + w/2, y + h/2, (255, 255, 255, 200), 12,
            anchor_x="center", anchor_y="center",
        )


class VisualDemo:
    """Visual demonstration of optimal route using Renderer."""
    
    def __init__(self, game, best_result: AnalysisResult, route_waypoints):
        self.game = game
        self.best_result = best_result
        self.route_waypoints = route_waypoints
        
        # Find matching gear config
        for pinion, stage2, name in GEAR_CONFIGS:
            if name == best_result.gear_name:
                self.pinion = pinion
                self.stage2 = stage2
                break
        
        # Create robot with optimal config
        self.robot = create_robot("OptimalBot", Pound(best_result.weight_lb), 
                                  self.pinion, self.stage2)
        
        # Game server
        self.server = DiscreteGameServer(ServerConfig())
        self.server.load_from_game(game)
        self.server.add_robot(self.robot)
        self.server.init_robot(self.robot.name, RobotState(*START_POS))
        
        # Generate all trajectories for the route
        self.trajectories = []
        current_pos = START_POS
        for target in self.route_waypoints:
            traversal_space = self.server.physics_engine.prepare_traversal_space(
                self.robot.name, self.robot, 
                list(game.get_obstacles()), game.get_field_size()
            )
            path = self.server.physics_engine.pathfind(
                self.robot.name, current_pos[0], current_pos[1],
                target[0], target[1], traversal_space
            )
            traj = self.server.physics_engine.generate_trajectory(
                self.robot.name, self.robot, current_pos, target,
                path, traversal_space, speed_zones=game.get_zones()
            )
            self.trajectories.append(traj)
            current_pos = target
        
        # Animation state
        self.current_traj_idx = 0
        self.anim_time = 0.0
        self.current_state = RobotState(*START_POS)
        self.is_animating = False
        
        # Setup renderer
        self.renderer = Renderer.create(game=game)
        self.renderer.display_level = DisplayLevel.SHOWCASE
        
        # Add obstacles
        for obs in game.get_obstacles():
            self.renderer.add(obs)
        
        # Add zones
        from gamegine.render.renderer import ObjectRendererRegistry
        ObjectRendererRegistry.register_handler(TraversalZone, draw_zone)
        for zone in game.get_zones():
            self.renderer.add(zone)
        
        # Add trajectories
        for traj in self.trajectories:
            self.renderer.add(traj)
        
        # Track robot
        self.renderer.track_robot(lambda: self.current_state)
        self.renderer.on_update_callback(self.update)
        self.renderer.on_key_press_callback(self.on_key)
        
        from gamegine.render import AlertType
        self.renderer.show_alert(
            f"Optimal: {best_result.gear_name} @ {best_result.weight_lb:.0f}lb | SPACE to animate",
            AlertType.SUCCESS, 8.0
        )
    
    def on_key(self, key, modifiers):
        if key == arcade.key.SPACE:
            self.is_animating = True
            self.current_traj_idx = 0
            self.anim_time = 0.0
            from gamegine.render import AlertType
            self.renderer.show_alert("Animating...", AlertType.INFO, 2.0)
    
    def update(self, dt):
        if not self.is_animating:
            return
        
        if self.current_traj_idx >= len(self.trajectories):
            self.is_animating = False
            from gamegine.render import AlertType
            self.renderer.show_alert(
                f"Complete! Total: {self.best_result.total_time:.2f}s | SPACE to replay",
                AlertType.SUCCESS, 5.0
            )
            return
        
        traj = self.trajectories[self.current_traj_idx]
        travel_time = float(traj.get_travel_time().to(Second))
        
        self.anim_time += dt
        
        if self.anim_time >= travel_time:
            # Move to next trajectory
            state = traj.get_at_time(traj.get_travel_time())
            self.current_state = RobotState(state.x, state.y, state.theta)
            self.current_traj_idx += 1
            self.anim_time = 0.0
        else:
            state = traj.get_at_time(Second(self.anim_time))
            self.current_state = RobotState(state.x, state.y, state.theta)
    
    def run(self):
        arcade.run()


# =============================================================================
# MAIN
# =============================================================================

def plot_results(results: List[AnalysisResult]):
    """Generate matplotlib graphs of the analysis results."""
    import matplotlib.pyplot as plt
    import numpy as np
    
    # Setup figure with 3 subplots (one per route)
    fig, axes = plt.subplots(1, 3, figsize=(16, 5))
    fig.suptitle('Robot Cycle Time vs Weight by Gear Ratio', fontsize=14, fontweight='bold')
    
    route_names = ["Route 1", "Route 2", "Route 3"]
    route_descriptions = [
        "Start \u2192 Center \u2192 HUB",
        "Start \u2192 Depot \u2192 HUB \u2192 Tower",
        "Start \u2192 Red HUB \u2192 Blue HUB"
    ]
    
    # Colors for each gear ratio
    colors = ['#2ecc71', '#3498db', '#9b59b6', '#e74c3c', '#f39c12', '#1abc9c']
    
    weights = [float(w.to(Pound)) for w in WEIGHTS]
    
    for ax_idx, (route_name, route_desc) in enumerate(zip(route_names, route_descriptions)):
        ax = axes[ax_idx]
        all_times = []
        
        for color_idx, (_, _, gear_name) in enumerate(GEAR_CONFIGS):
            times = []
            for weight in WEIGHTS:
                r = next((r for r in results 
                         if r.gear_name == gear_name 
                         and r.weight_lb == float(weight.to(Pound))
                         and r.route_name == route_name), None)
                times.append(r.total_time if r else 0)
            all_times.extend(times)
            
            ax.plot(weights, times, marker='o', linewidth=2, markersize=6,
                   color=colors[color_idx], label=gear_name)
        
        ax.set_xlabel('Robot Weight (lb)', fontsize=10)
        ax.set_ylabel('Cycle Time (s)', fontsize=10)
        ax.set_title(f'{route_name}\n{route_desc}', fontsize=11)
        ax.grid(True, alpha=0.3)
        ax.legend(loc='upper left', fontsize=8)
        
        # Auto-scale Y-axis to show actual data range with padding
        if all_times:
            min_time = min(all_times)
            max_time = max(all_times)
            padding = (max_time - min_time) * 0.15
            ax.set_ylim(bottom=max(0, min_time - padding), top=max_time + padding)
    
    plt.tight_layout()
    plt.savefig('robot_speed_analysis_results.png', dpi=150, bbox_inches='tight')
    print("\nGraph saved to: robot_speed_analysis_results.png")
    plt.show()


def plot_acceleration_curves(weight=Pound(120)):
    """Generate acceleration vs speed graph for all gear ratios.
    
    Shows simulated acceleration at each wheel speed, accounting for:
    - Motor torque-speed characteristics
    - Gear ratio effects
    - Battery voltage sag under load
    
    :param weight: Robot weight to use for calculations
    """
    import matplotlib.pyplot as plt
    import numpy as np
    from gamegine.utils.NCIM.ComplexDimensions.omega import RadiansPerSecond
    from gamegine.utils.NCIM.Dimensions.mass import Kilogram
    
    print("\nGenerating Acceleration vs Speed curves...")
    
    # Setup plot
    fig, ax = plt.subplots(figsize=(12, 7))
    fig.suptitle(f'Acceleration vs Wheel Speed by Gear Ratio\n(Robot Weight: {float(weight.to(Pound)):.0f}lb, with Battery Simulation)', 
                 fontsize=14, fontweight='bold')
    
    # Colors for each gear ratio
    colors = ['#2ecc71', '#3498db', '#9b59b6', '#e74c3c', '#f39c12', '#1abc9c']
    
    # Battery model for voltage sag
    battery = BatteryModel.frc_standard()
    
    mass_kg = float(weight.to(Kilogram))
    
    # Speed range (0 to max speed across all gearings)
    max_speed_overall = 0
    for pinion, stage2, gear_name in GEAR_CONFIGS:
        robot = create_robot("TestBot", weight, pinion, stage2)
        module = robot.drivetrain.module
        wheel_radius = module.wheel.diameter / 2
        max_wheel_omega = module.get_max_speed()
        max_linear_speed = float(max_wheel_omega) * float(wheel_radius.to(Meter))
        max_speed_overall = max(max_speed_overall, max_linear_speed)
    
    speeds = np.linspace(0, max_speed_overall * 1.05, 100)
    
    for color_idx, (pinion, stage2, gear_name) in enumerate(GEAR_CONFIGS):
        robot = create_robot("TestBot", weight, pinion, stage2)
        module = robot.drivetrain.module
        wheel_radius = module.wheel.diameter / 2
        wheel_radius_m = float(wheel_radius.to(Meter))
        
        accelerations = []
        
        for speed in speeds:
            # Convert linear speed to wheel angular velocity
            wheel_omega = speed / wheel_radius_m  # rad/s
            
            # Get torque at this speed from motor model
            torque = module.get_torque(RadiansPerSecond(wheel_omega))
            torque_nm = float(torque)
            
            # Account for battery voltage sag
            # Estimate current draw: I ≈ T * ω / V (simplified)
            motor_omega = wheel_omega * module.drive_gear_ratio.get_ratio()
            power_w = abs(torque_nm * motor_omega / module.drive_gear_ratio.get_ratio())
            
            # Current from 4 motors
            estimated_current = 4 * power_w / 12.0  # Rough estimate at nominal voltage
            terminal_voltage = battery.get_terminal_voltage(Ampere(estimated_current))
            voltage_factor = float(terminal_voltage.to(Volt)) / 12.0
            
            # Adjusted torque due to voltage sag
            adjusted_torque = torque_nm * max(0.5, voltage_factor)
            
            # Force = 4 modules * torque / wheel_radius
            force = 4 * adjusted_torque / wheel_radius_m
            
            # Acceleration = F / m
            accel = force / mass_kg
            accelerations.append(max(0, accel))
        
        # Find free speed (where torque = 0)
        max_wheel_omega = module.get_max_speed()
        free_speed = float(max_wheel_omega) * wheel_radius_m
        
        ax.plot(speeds, accelerations, linewidth=2.5, color=colors[color_idx], 
                label=f'{gear_name} (max {free_speed:.1f} m/s)')
        
        # Mark the free speed point
        ax.axvline(x=free_speed, color=colors[color_idx], linestyle='--', alpha=0.3, linewidth=1)
    
    ax.set_xlabel('Wheel Speed (m/s)', fontsize=12)
    ax.set_ylabel('Acceleration (m/s²)', fontsize=12)
    ax.set_xlim(left=0)
    ax.set_ylim(bottom=0)
    ax.grid(True, alpha=0.3)
    ax.legend(loc='upper right', fontsize=10)
    
    # Add annotation
    ax.annotate('Higher gear ratio = more torque, less speed', 
                xy=(0.02, 0.98), xycoords='axes fraction',
                fontsize=9, ha='left', va='top', style='italic',
                color='gray')
    
    plt.tight_layout()
    plt.savefig('acceleration_vs_speed.png', dpi=150, bbox_inches='tight')
    print("Graph saved to: acceleration_vs_speed.png")
    plt.show()


class MultiRobotRace:
    """Visual race comparing all gear ratios on Route 3 at 120lb."""
    
    def __init__(self, game, weight=Pound(150)):
        self.game = game
        self.weight = weight
        
        # Colors for each gear ratio (RGB)
        self.colors = [
            (46, 204, 113),   # Green - P12-S18
            (52, 152, 219),   # Blue - P12-S16
            (155, 89, 182),   # Purple - P13-S18
            (231, 76, 60),    # Red - P13-S16
            (243, 156, 18),   # Orange - P14-S18
            (26, 188, 156),   # Teal - P14-S16
        ]
        
        # Create robots and generate trajectories
        self.robots = []
        self.trajectories = []
        self.robot_states = []
        
        server = DiscreteGameServer(ServerConfig())
        server.load_from_game(game)
        
        for i, (pinion, stage2, gear_name) in enumerate(GEAR_CONFIGS):
            robot = create_robot(f"Robot_{gear_name}", weight, pinion, stage2)
            self.robots.append((robot, gear_name))
            
            # Generate trajectories for Route 3
            server.add_robot(robot)
            server.init_robot(robot.name, RobotState(*START_POS))
            
            trajs = []
            current_pos = START_POS
            for target in ROUTE_3_WAYPOINTS:
                traversal_space = server.physics_engine.prepare_traversal_space(
                    robot.name, robot,
                    list(game.get_obstacles()), game.get_field_size()
                )
                path = server.physics_engine.pathfind(
                    robot.name, current_pos[0], current_pos[1],
                    target[0], target[1], traversal_space
                )
                traj = server.physics_engine.generate_trajectory(
                    robot.name, robot, current_pos, target,
                    path, traversal_space, speed_zones=game.get_zones()
                )
                trajs.append(traj)
                current_pos = target
            
            self.trajectories.append(trajs)
            self.robot_states.append(RobotState(*START_POS))
        
        # Animation state
        self.anim_time = 0.0
        self.is_animating = False
        self.finished = [False] * len(self.robots)
        
        # Calculate total times for each robot
        self.total_times = []
        for trajs in self.trajectories:
            total = sum(float(t.get_travel_time().to(Second)) for t in trajs)
            self.total_times.append(total)
        
        # Current trajectory index for each robot
        self.traj_indices = [0] * len(self.robots)
        self.traj_times = [0.0] * len(self.robots)
        
        # Setup renderer
        self.renderer = Renderer.create(game=game)
        self.renderer.display_level = DisplayLevel.SHOWCASE
        
        # Add obstacles
        for obs in game.get_obstacles():
            self.renderer.add(obs)
        
        # Add zones
        from gamegine.render.renderer import ObjectRendererRegistry
        ObjectRendererRegistry.register_handler(TraversalZone, draw_zone)
        for zone in game.get_zones():
            self.renderer.add(zone)
        
        # Add trajectory for first robot only (for reference)
        for traj in self.trajectories[0]:
            self.renderer.add(traj)
        
        # Register custom multi-robot drawing handler
        def draw_race_robots(race_ref, canvas, theme, display_level, renderer=None):
            """Draw all racing robots."""
            import math
            race = race_ref()  # Dereference the weakref-like callable
            if race is None:
                return
            
            robot_size = canvas.to_pixels(Inch(30))
            
            for i, (robot, gear_name) in enumerate(race.robots):
                state = race.robot_states[i]
                x = canvas.to_pixels(state.x.get())
                y = canvas.to_pixels(state.y.get())
                
                color = race.colors[i]
                heading = float(state.heading.get().to(Degree))
                half_size = robot_size / 2
                
                # Calculate corners based on rotation
                rad = math.radians(heading)
                cos_h, sin_h = math.cos(rad), math.sin(rad)
                
                corners = []
                for dx, dy in [(-1, -1), (1, -1), (1, 1), (-1, 1)]:
                    rx = dx * half_size
                    ry = dy * half_size
                    corners.append((
                        x + rx * cos_h - ry * sin_h,
                        y + rx * sin_h + ry * cos_h
                    ))
                
                arcade.draw_polygon_filled(corners, (*color, 200))
                arcade.draw_polygon_outline(corners, (*color, 255), 2)
                
                # Draw gear ratio label
                arcade.draw_text(
                    gear_name, x, y + robot_size,
                    (*color, 255), 10, anchor_x="center"
                )
                
                # Draw finish indicator
                if race.finished[i]:
                    arcade.draw_text(
                        f"✓ {race.total_times[i]:.2f}s", x, y - robot_size,
                        (255, 255, 255), 10, anchor_x="center"
                    )
        
        # Create a simple class to hold reference and register handler
        class RaceDisplay:
            def __init__(self, race):
                self._race = race
            def __call__(self):
                return self._race
        
        self.race_display = RaceDisplay(self)
        ObjectRendererRegistry.register_handler(RaceDisplay, draw_race_robots)
        self.renderer.add(self.race_display)
        
        self.renderer.on_update_callback(self.update)
        self.renderer.on_key_press_callback(self.on_key)
        
        from gamegine.render import AlertType
        self.renderer.show_alert(
            f"GEAR RATIO RACE - Route 3 @ {float(weight.to(Pound)):.0f}lb | SPACE to start",
            AlertType.SUCCESS, 8.0
        )
    
    def on_key(self, key, modifiers):
        if key == arcade.key.SPACE:
            self.is_animating = True
            self.anim_time = 0.0
            self.traj_indices = [0] * len(self.robots)
            self.traj_times = [0.0] * len(self.robots)
            self.finished = [False] * len(self.robots)
            self.robot_states = [RobotState(*START_POS) for _ in self.robots]
            from gamegine.render import AlertType
            self.renderer.show_alert("RACE STARTED!", AlertType.INFO, 2.0)
    
    def update(self, dt):
        if not self.is_animating:
            return
        
        self.anim_time += dt
        all_finished = True
        
        for i, trajs in enumerate(self.trajectories):
            if self.finished[i]:
                continue
            
            all_finished = False
            self.traj_times[i] += dt
            
            # Get current trajectory
            if self.traj_indices[i] >= len(trajs):
                self.finished[i] = True
                continue
            
            traj = trajs[self.traj_indices[i]]
            travel_time = float(traj.get_travel_time().to(Second))
            
            if self.traj_times[i] >= travel_time:
                # Move to next trajectory
                state = traj.get_at_time(traj.get_travel_time())
                self.robot_states[i] = RobotState(state.x, state.y, state.theta)
                self.traj_indices[i] += 1
                self.traj_times[i] = 0.0
            else:
                state = traj.get_at_time(Second(self.traj_times[i]))
                self.robot_states[i] = RobotState(state.x, state.y, state.theta)
        
        if all_finished:
            self.is_animating = False
            # Find winner
            winner_idx = self.total_times.index(min(self.total_times))
            winner_name = self.robots[winner_idx][1]
            from gamegine.render import AlertType
            self.renderer.show_alert(
                f"WINNER: {winner_name} @ {self.total_times[winner_idx]:.2f}s | SPACE to replay",
                AlertType.SUCCESS, 10.0
            )
    
    def run(self):
        arcade.run()


def main():
    print("\n" + "=" * 70)
    print("THRIFTY SWERVE GEAR RATIO ANALYSIS")
    print("=" * 70 + "\n")
    
    # Run analysis
    results, game = run_analysis()
    
    # Print summary tables
    best_r1, best_r2, best_r3 = print_results_table(results)
    
    # Multi-robot race on Route 3 at 120lb
    print("\n\nLaunching GEAR RATIO RACE - Route 3 @ 120lb...")
    print("Press SPACE to start race, close window when done.\n")
    
    race = MultiRobotRace(game, Pound(120))
    race.run()
    
    # Show graphs after visualization closes
    print("\nGenerating analysis graphs...")
    plot_results(results)
    
    # Show acceleration vs speed curves
    plot_acceleration_curves(Pound(120))


if __name__ == "__main__":
    main()
