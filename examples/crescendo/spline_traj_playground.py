"""
Spline Trajectory Playground - A simplified trajectory generator using cubic splines.

This example demonstrates the SplineTrajectoryGenerator which bypasses the complex
optimization solver for faster, more reliable trajectory generation.

Usage:
- Click to set waypoints
- Trajectory is generated instantly using cubic splines
- Press 'C' to clear all trajectories
"""

from typing import List, Tuple
import pygame
import math
from examples.crescendo.crescendo import Crescendo
from gamegine.analysis import pathfinding
from gamegine.analysis.meshing import TriangulatedGraph
from gamegine.analysis.trajectory.generator import SplineTrajectoryGenerator
from gamegine.analysis.trajectory.lib.TrajGen import (
    SwerveTrajectory,
    SwerveRobotConstraints,
)
from gamegine.reference import gearing, motors
from gamegine.reference.swerve import SwerveConfig, SwerveModule
from gamegine.render.renderer import Renderer
from gamegine.representation.bounds import ExpandedObjectBounds
from gamegine.representation.robot import PhysicalParameters
from gamegine.utils.NCIM.ComplexDimensions.MOI import PoundsInchesSquared
from gamegine.utils.NCIM.ComplexDimensions.acceleration import MeterPerSecondSquared
from gamegine.utils.NCIM.ComplexDimensions.alpha import RadiansPerSecondSquared
from gamegine.utils.NCIM.ComplexDimensions.omega import RadiansPerSecond
from gamegine.utils.NCIM.ComplexDimensions.velocity import MetersPerSecond
from gamegine.utils.NCIM.Dimensions.angular import AngularMeasurement, Degree, Radian
from gamegine.utils.NCIM.Dimensions.current import Ampere
from gamegine.utils.NCIM.Dimensions.mass import Pound
from gamegine.utils.NCIM.Dimensions.spatial import (
    Centimeter,
    Feet,
    Inch,
    Meter,
    SpatialMeasurement,
)
from gamegine.utils.NCIM.Dimensions.temporal import Second
from gamegine.render import helpers
from gamegine.render.style import Palette


# Set up field and obstacles
expanded_obstacles = ExpandedObjectBounds(
    Crescendo.get_obstacles(),
    robot_radius=Inch(20),
    discretization_quality=16,
)
slightly_more_expanded_obstacles = ExpandedObjectBounds(
    Crescendo.get_obstacles(),
    robot_radius=Inch(20) + Inch(2),
    discretization_quality=16,
)

# Create navigation mesh
map = TriangulatedGraph(
    slightly_more_expanded_obstacles, Feet(2), Crescendo.get_field_size()
)

# Define robot constraints (needed for SwerveTrajectory drawing)
ROBOT_CONSTRAINTS = SwerveRobotConstraints(
    MeterPerSecondSquared(5),
    MetersPerSecond(6),
    RadiansPerSecondSquared(3.14),
    RadiansPerSecond(3.14),
    SwerveConfig(
        module=SwerveModule(
            motors.MotorConfig(
                motors.KrakenX60,
                motors.PowerConfig(Ampere(60), Ampere(360), 1.0),
            ),
            gearing.MK4I.L3,
            motors.MotorConfig(
                motors.KrakenX60,
                motors.PowerConfig(Ampere(60), Ampere(360), 1.0),
            ),
            gearing.MK4I.L3,
        )
    ),
    physical_parameters=PhysicalParameters(
        mass=Pound(110),
        moi=PoundsInchesSquared(21327.14),
    ),
)

# Create the spline trajectory generator
# Centripetal acceleration is now automatically computed from wheel friction (µ * g)
trajectory_generator = SplineTrajectoryGenerator(
    max_velocity=MetersPerSecond(5.0),
    max_acceleration=MeterPerSecondSquared(3.0),  # Tangential acceleration limit
    min_curvature_radius=Meter(0.3),
    resolution=Meter(0.15),
)


def CreatePath(
    start: Tuple[SpatialMeasurement, SpatialMeasurement],
    end: Tuple[SpatialMeasurement, SpatialMeasurement],
) -> pathfinding.Path:
    """Creates an A* path from start to end, avoiding obstacles."""
    path = pathfinding.findPath(
        map,
        start,
        end,
        pathfinding.AStar,
        pathfinding.InitialConnectionPolicy.ConnectToClosest,
    )
    path.shortcut(expanded_obstacles)
    return path


def CreateTrajectory(
    start: Tuple[SpatialMeasurement, SpatialMeasurement, AngularMeasurement],
    end: Tuple[SpatialMeasurement, SpatialMeasurement],
    target_heading: AngularMeasurement,
) -> SwerveTrajectory:
    """Creates a trajectory using the spline generator."""
    path = CreatePath(start[:2], end)
    
    # Store path for visualization
    global paths
    paths.append(path)
    
    # Generate trajectory using spline generator (no solver!)
    trajectory = trajectory_generator.generate(
        robot_name="TestRobot",
        robot=None,
        start_state=(start[0], start[1], start[2]),
        target_state=(end[0], end[1], target_heading),
        path=path,
        traversal_space=None,
        robot_constraints=ROBOT_CONSTRAINTS,  # Pass proper constraints for drawing
    )
    
    return trajectory


# Initialize renderer
renderer = Renderer()
renderer.set_game(Crescendo)
renderer.set_render_scale(Centimeter(1))
renderer.init_display()
print("Spline Trajectory Playground initialized")
print("Click to create trajectories. Press 'C' to clear.")

# Global state
Current = (Feet(6), Inch(64.081) + Inch(82.645) / 2, Degree(0))
trajectories: List[SwerveTrajectory] = []
paths = []
current_target_heading = Degree(0)
ROTATION_SPEED = Degree(180)  # Degrees per second

# Main loop
loop = True
clock = pygame.time.Clock()
current_time = Second(0)

while loop:
    events = renderer.loop()
    if events is False:
        loop = False
        break
    
    dt_ms = clock.tick(60)
    dt_sec = dt_ms / 1000.0
    current_time += Second(dt_sec)

    # Handle Continuous Key Presses for Rotation
    keys = pygame.key.get_pressed()
    if keys[pygame.K_q]:
        current_target_heading += ROTATION_SPEED * dt_sec
    if keys[pygame.K_e]:
        current_target_heading -= ROTATION_SPEED * dt_sec

    for event in events:
        if event.type == pygame.MOUSEBUTTONUP:
            pos = pygame.mouse.get_pos()
            x, y = (
                Renderer.render_scale * pos[0],
                Renderer.render_scale * pos[1],
            )
            
            print(f"Generating trajectory from {Current} to ({x}, {y}) with heading {current_target_heading}...")
            try:
                trajectory = CreateTrajectory(Current, (x, y), current_target_heading)
                trajectories.append(trajectory)
                Current = (x, y, current_target_heading)
                print(f"  Generated! Length: {trajectory.get_length()}, Time: {trajectory.get_travel_time()}")
            except Exception as e:
                print(f"  Failed: {e}")
                
        if event.type == pygame.KEYUP:
            if event.key == pygame.K_c:
                trajectories = []
                paths = []
                current_time = Second(0)
                print("Cleared all trajectories")

    # Draw the field
    renderer.draw_element(map)
    renderer.draw_elements(expanded_obstacles)
    
    # Draw A* paths (thin green lines)
    render_scale = Renderer.render_scale
    for path in paths:
        points = path.get_points()
        for i in range(len(points) - 1):
            p1 = points[i]
            p2 = points[i + 1]
            helpers.draw_line(
                p1[0], p1[1], p2[0], p2[1],
                Inch(0.5), Palette.LIGHT_GREEN, render_scale
            )
    
    # Draw trajectories
    renderer.draw_elements(trajectories)
    renderer.draw_elements(paths)
    renderer.draw_static_elements()

    # === ROBOT ANIMATION ===
    if trajectories:
        total_traj_time = Second(0)
        for t in trajectories:
            total_traj_time += t.get_travel_time()
            
        if total_traj_time > Second(0):
            anim_time = current_time % total_traj_time
            
            accum_time = Second(0)
            for t in trajectories:
                duration = t.get_travel_time()
                if accum_time + duration > anim_time:
                    # Distinct time within this trajectory
                    local_time = anim_time - accum_time
                    state = t.get_at_time(local_time)
                    
                    # Draw Robot
                    helpers.draw_point(
                        state.x, 
                        state.y, 
                        Inch(15), 
                        Palette.GREEN, 
                        Renderer.render_scale
                    )
                    
                    # Draw Heading
                    head_len = Inch(20)
                    end_x = state.x + head_len * math.cos(state.theta.to(Radian))
                    end_y = state.y + head_len * math.sin(state.theta.to(Radian))
                    
                    start_pix = (Renderer.to_pixels(state.x), Renderer.to_pixels(state.y))
                    end_pix = (Renderer.to_pixels(end_x), Renderer.to_pixels(end_y))
                    
                    pygame.draw.line(
                        pygame.display.get_surface(),
                        (0, 0, 0),
                        start_pix,
                        end_pix,
                        width=3
                    )
                    break
                accum_time += duration

    # Draw current position marker
    helpers.draw_point(Current[0], Current[1], Inch(3), Palette.BLUE, render_scale)

    # Draw Destination Overlay (Ghost Robot)
    mouse_pos = pygame.mouse.get_pos()
    mx, my = Renderer.render_scale * mouse_pos[0], Renderer.render_scale * mouse_pos[1]
    
    # Ghost Robot Body
    helpers.draw_point(mx, my, Inch(15), Palette.WHITE, render_scale) # Grey for 'ghost'
    
    # Ghost Robot Heading
    head_len = Inch(20)
    ghost_end_x = mx + head_len * math.cos(current_target_heading.to(Radian))
    ghost_end_y = my + head_len * math.sin(current_target_heading.to(Radian))
    
    start_pix = (Renderer.to_pixels(mx), Renderer.to_pixels(my))
    end_pix = (Renderer.to_pixels(ghost_end_x), Renderer.to_pixels(ghost_end_y))
    
    pygame.draw.line(
        pygame.display.get_surface(),
        (100, 100, 100), # Dark Grey
        start_pix,
        end_pix,
        width=2
    )
    
    # Draw Heading Text
    font = pygame.font.SysFont("Arial", 16) if pygame.font.get_init() else None
    if font:
        deg_val = int(current_target_heading.to(Degree) % 360)
        text_surf = font.render(f"{deg_val}°", True, (0, 0, 0))
        pygame.display.get_surface().blit(text_surf, (start_pix[0] + 15, start_pix[1] - 15))

    renderer.render_frame()

pygame.quit()
