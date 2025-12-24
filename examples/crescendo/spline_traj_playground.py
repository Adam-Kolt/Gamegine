"""
Spline Trajectory Playground - Complete Feature Demo.

Controls:
    Click     - Create trajectory to clicked position
    Q/E       - Rotate destination heading (visible as ghost at cursor)
    C         - Clear all trajectories
    D         - Toggle debug/showcase mode
    G         - Toggle grid
    
Debug mode shows:
    - Trajectory waypoints
    - Swerve module positions along path
"""

import math
import arcade
from examples.crescendo.crescendo import Crescendo
from gamegine.analysis import pathfinding
from gamegine.analysis.meshing import TriangulatedGraph
from gamegine.analysis.trajectory.generator import SplineTrajectoryGenerator
from gamegine.analysis.trajectory.lib.TrajGen import SwerveRobotConstraints
from gamegine.reference import gearing, motors
from gamegine.reference.swerve import SwerveConfig, SwerveModule
from gamegine.render import Renderer, DisplayLevel, AlertType, run
from gamegine.representation.bounds import ExpandedObjectBounds
from gamegine.representation.robot import PhysicalParameters, SwerveRobot
from gamegine.utils.NCIM.ComplexDimensions.MOI import PoundsInchesSquared
from gamegine.utils.NCIM.ComplexDimensions.acceleration import MeterPerSecondSquared
from gamegine.utils.NCIM.ComplexDimensions.alpha import RadiansPerSecondSquared
from gamegine.utils.NCIM.ComplexDimensions.omega import RadiansPerSecond
from gamegine.utils.NCIM.ComplexDimensions.velocity import MetersPerSecond
from gamegine.utils.NCIM.Dimensions.angular import Degree, Radian
from gamegine.utils.NCIM.Dimensions.current import Ampere
from gamegine.utils.NCIM.Dimensions.mass import Pound
from gamegine.utils.NCIM.Dimensions.spatial import Feet, Inch, Meter
from gamegine.utils.NCIM.Dimensions.temporal import Second

# =============================================================================
# Configuration
# =============================================================================

obstacles = Crescendo.get_obstacles()
expanded = ExpandedObjectBounds(obstacles, Inch(20), 16)
nav_mesh = TriangulatedGraph(expanded, Feet(2), Crescendo.get_field_size())

# Swerve robot configuration
swerve_config = SwerveConfig(SwerveModule(
    motors.MotorConfig(motors.KrakenX60, motors.PowerConfig(Ampere(60), Ampere(360), 1.0)),
    gearing.MK4I.L3,
    motors.MotorConfig(motors.KrakenX60, motors.PowerConfig(Ampere(60), Ampere(360), 1.0)),
    gearing.MK4I.L3,
))

constraints = SwerveRobotConstraints(
    MeterPerSecondSquared(5), MetersPerSecond(6),
    RadiansPerSecondSquared(3.14), RadiansPerSecond(3.14),
    swerve_config,
    PhysicalParameters(Pound(110), PoundsInchesSquared(21327.14)),
)

# Create SwerveRobot with physics for accurate trajectory generation
robot = SwerveRobot(
    "TestRobot",
    drivetrain=swerve_config,
    physics=PhysicalParameters(Pound(110), PoundsInchesSquared(21327.14)),
)

generator = SplineTrajectoryGenerator(
    MetersPerSecond(5.0), MeterPerSecondSquared(3.0), Meter(0.3), Meter(0.15),
)

# Robot dimensions
ROBOT_HALF_SIZE = 18  # pixels

# =============================================================================
# Renderer Setup
# =============================================================================

renderer = Renderer.create(game=Crescendo)
renderer.display_level = DisplayLevel.SHOWCASE

# Add static objects
renderer.add_obstacles(obstacles)
renderer.add_safety_padding(expanded)
renderer.add(nav_mesh)

# =============================================================================
# State
# =============================================================================

current_pos = (Feet(6), Inch(64.081 + 82.645/2), Degree(0))
destination_heading = Degree(0)
trajectories = []  # List of (trajectory, constraints) pairs
paths = []
elapsed_time = 0.0
mouse_world_x = 0.0
mouse_world_y = 0.0
hovered_robot_state = None  # For hover tooltip

def clear_all():
    """Clear all trajectories."""
    global trajectories, paths, current_pos
    trajectories = []
    paths = []
    renderer.clear_objects()
    renderer.add_obstacles(obstacles)
    renderer.add_safety_padding(expanded)
    renderer.add(nav_mesh)
    current_pos = (Feet(6), Inch(64.081 + 82.645/2), Degree(0))
    renderer.show_alert("Trajectories Cleared", AlertType.SUCCESS)

# =============================================================================
# Event Handlers
# =============================================================================

@renderer.on_click
def on_click(x, y):
    global current_pos, destination_heading, trajectories, paths
    target = (Meter(x), Meter(y))
    
    try:
        path = pathfinding.findPath(
            nav_mesh, current_pos[:2], target,
            pathfinding.AStar, pathfinding.InitialConnectionPolicy.ConnectToClosest,
        )
        path.shortcut(expanded)
        
        traj = generator.generate(
            "Robot", robot, current_pos, (target[0], target[1], destination_heading),
            path, expanded,  # Pass robot and expanded obstacles
        )
        
        renderer.add(path)
        renderer.add(traj)
        # Store trajectory with its constraints for debug drawing
        trajectories.append((traj, constraints))
        paths.append(path)
        current_pos = (target[0], target[1], destination_heading)
        print(f"Trajectory: {traj.get_length()}, {traj.get_travel_time()}")
    except Exception as e:
        renderer.show_alert(f"Path error: {str(e)[:30]}", AlertType.ERROR, 3.0)

def on_update(dt):
    global destination_heading, elapsed_time
    elapsed_time += dt
    
    # Q/E to rotate destination heading
    if renderer.is_key_pressed(arcade.key.Q):
        destination_heading += Degree(180 * dt)
    if renderer.is_key_pressed(arcade.key.E):
        destination_heading -= Degree(180 * dt)

def on_key_press(key, mods):
    """Handle key press events."""
    if key == arcade.key.C:
        clear_all()

renderer.on_update_callback(on_update)
renderer.on_key_press_callback(on_key_press)

# =============================================================================
# Drawing Helpers
# =============================================================================

def draw_robot(px, py, theta, fill_color, outline_color, size=ROBOT_HALF_SIZE):
    """Draw a square robot with a heading indicator."""
    cos_t = math.cos(theta)
    sin_t = math.sin(theta)
    
    # Robot corners (rotated square)
    corners = []
    for dx, dy in [(-1, -1), (1, -1), (1, 1), (-1, 1)]:
        rx = px + (dx * cos_t - dy * sin_t) * size
        ry = py + (dx * sin_t + dy * cos_t) * size
        corners.append((rx, ry))
    
    # Draw filled robot
    arcade.draw_polygon_filled(corners, fill_color)
    arcade.draw_polygon_outline(corners, outline_color, 2)
    
    # Draw heading arrow
    arrow_len = size * 1.3
    arrow_x = px + cos_t * arrow_len
    arrow_y = py + sin_t * arrow_len
    arcade.draw_line(px, py, arrow_x, arrow_y, outline_color, 3)

def draw_ghost_robot(px, py, theta, color, size=ROBOT_HALF_SIZE):
    """Draw a ghost (outline only) robot."""
    cos_t = math.cos(theta)
    sin_t = math.sin(theta)
    
    corners = []
    for dx, dy in [(-1, -1), (1, -1), (1, 1), (-1, 1)]:
        rx = px + (dx * cos_t - dy * sin_t) * size
        ry = py + (dx * sin_t + dy * cos_t) * size
        corners.append((rx, ry))
    
    arcade.draw_polygon_outline(corners, color, 2)
    
    # Heading indicator
    arrow_x = px + cos_t * size * 1.3
    arrow_y = py + sin_t * size * 1.3
    arcade.draw_line(px, py, arrow_x, arrow_y, color, 2)

def draw_swerve_modules(px, py, theta, config, theme, size=0.4):
    """Draw swerve module positions."""
    canvas = renderer.drawing_canvas
    cos_t = math.cos(theta)
    sin_t = math.sin(theta)
    
    # Get module offsets from swerve config
    offsets = [
        config.top_left_offset,
        config.top_right_offset,
        config.bottom_right_offset,
        config.bottom_left_offset,
    ]
    
    for j, offset in enumerate(offsets):
        ox = offset[0].to(Meter) if hasattr(offset[0], 'to') else float(offset[0])
        oy = offset[1].to(Meter) if hasattr(offset[1], 'to') else float(offset[1])
        
        # Rotate offset by robot heading
        wx = ox * cos_t - oy * sin_t
        wy = ox * sin_t + oy * cos_t
        
        mpx = px + canvas.render_scale * wx
        mpy = py + canvas.render_scale * wy
        
        arcade.draw_circle_filled(mpx, mpy, 5, theme.module_colors[j])

def draw_tooltip(x, y, lines, theme):
    """Draw tooltip with multiple lines."""
    padding = 8
    line_height = 18
    width = max(len(line) * 8 for line in lines) + padding * 2
    height = len(lines) * line_height + padding * 2
    
    # Draw background (lbwh = left, bottom, width, height)
    arcade.draw_lbwh_rectangle_filled(x, y, width, height, theme.tooltip_background)
    arcade.draw_lbwh_rectangle_outline(x, y, width, height, theme.tooltip_border, 1)
    
    # Draw text
    for i, line in enumerate(lines):
        arcade.draw_text(
            line, x + padding, y + height - padding - (i + 1) * line_height + 4,
            theme.tooltip_text, 12
        )

# =============================================================================
# Custom On Draw
# =============================================================================

original_on_draw = renderer.on_draw

def custom_on_draw():
    global mouse_world_x, mouse_world_y, hovered_robot_state
    
    # Call original renderer
    original_on_draw()
    
    canvas = renderer.drawing_canvas
    theme = renderer.theme
    display_level = renderer.display_level
    
    # Update mouse world position
    mouse_world_x, mouse_world_y = renderer.screen_to_world(
        renderer._mouse_x, renderer._mouse_y
    )
    
    # --- Draw Swerve Debug (if DEBUG mode) ---
    if display_level == DisplayLevel.DEBUG:
        for traj, constr in trajectories:
            if hasattr(traj, 'points') and len(traj.points) > 0:
                step = max(1, len(traj.points) // 12)
                for i in range(0, len(traj.points), step):
                    pt = traj.points[i]
                    px = canvas.to_pixels(pt.x)
                    py = canvas.to_pixels(pt.y)
                    theta = pt.theta.to(Radian)
                    
                    # Draw module positions
                    draw_swerve_modules(px, py, theta, constr.swerve_config, theme)
    
    # --- Draw Animated Robot ---
    hovered_robot_state = None
    if trajectories:
        total_time = sum(t.get_travel_time().to(Second) for t, _ in trajectories)
        if total_time > 0:
            anim_time = elapsed_time % total_time
            
            accum = 0.0
            for traj, constr in trajectories:
                duration = traj.get_travel_time().to(Second)
                if accum + duration > anim_time:
                    local_time = Second(anim_time - accum)
                    state = traj.get_at_time(local_time)
                    
                    px = canvas.to_pixels(state.x)
                    py = canvas.to_pixels(state.y)
                    theta = state.theta.to(Radian)
                    
                    # Draw robot
                    draw_robot(px, py, theta, theme.robot_fill, theme.robot_outline)
                    
                    # Draw swerve modules on animated robot if debug
                    if display_level == DisplayLevel.DEBUG:
                        draw_swerve_modules(px, py, theta, constr.swerve_config, theme, 0.3)
                    
                    # Check if mouse is over robot for hover
                    dist = math.sqrt((renderer._mouse_x - px)**2 + (renderer._mouse_y - py)**2)
                    if dist < ROBOT_HALF_SIZE * 1.5:
                        hovered_robot_state = state
                    
                    break
                accum += duration
    
    # --- Draw Ghost at Current Position ---
    cx = canvas.to_pixels(current_pos[0])
    cy = canvas.to_pixels(current_pos[1])
    theta = current_pos[2].to(Radian) if hasattr(current_pos[2], 'to') else 0
    draw_ghost_robot(cx, cy, theta, theme.ghost_outline)
    
    # --- Draw Ghost at Cursor (with destination heading) ---
    cursor_px = renderer._mouse_x
    cursor_py = renderer._mouse_y
    dest_theta = destination_heading.to(Radian)
    draw_ghost_robot(cursor_px, cursor_py, dest_theta, theme.ghost_outline, ROBOT_HALF_SIZE * 0.8)
    
    # --- Hover Tooltip ---
    if hovered_robot_state is not None:
        state = hovered_robot_state
        lines = [
            f"Position: ({state.x.to(Meter):.2f}, {state.y.to(Meter):.2f})",
            f"Heading: {state.theta.to(Degree):.1f}°",
        ]
        if hasattr(state, 'velocity'):
            lines.append(f"Velocity: {state.velocity.to(MetersPerSecond):.2f} m/s")
        draw_tooltip(renderer._mouse_x + 20, renderer._mouse_y, lines, theme)

renderer.on_draw = custom_on_draw

# =============================================================================
# Run
# =============================================================================

run()
