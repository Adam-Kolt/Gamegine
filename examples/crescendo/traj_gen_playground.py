from typing import List, Tuple
import pygame
from examples.crescendo.crescendo import Crescendo
from gamegine.analysis import pathfinding
from gamegine.analysis.meshing import TriangulatedGraph
from gamegine.analysis.trajectory.lib.TrajGen import (
    SolverConfig,
    SwerveRobotConstraints,
    SwerveTrajectory,
    SwerveTrajectoryProblemBuilder,
    TrajectoryBuilderConfig,
    Waypoint,
)
from gamegine.analysis.trajectory.lib.constraints.avoidance import SafetyCorridor
from gamegine.analysis.trajectory.lib.constraints.constraints import VelocityEquals
from gamegine.reference import gearing, motors
from gamegine.reference.swerve import SwerveConfig, SwerveModule
from gamegine.render.renderer import Renderer
from gamegine.representation.bounds import Cylinder, ExpandedObjectBounds, Transform3D
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
    SpatialMeasurement,
)
from gamegine.utils.NCIM.Dimensions.temporal import Second
from gamegine.render import helpers
from gamegine.render.style import Palette
import math


expanded_obstacles = ExpandedObjectBounds(
    Crescendo.get_obstacles(),
    robot_radius=Inch(20),
    discretization_quality=16,  # 24.075
)
slightly_more_expanded_obstacles = ExpandedObjectBounds(
    Crescendo.get_obstacles(),
    robot_radius=Inch(20) + Inch(2),
    discretization_quality=16,  # 24.075
)
expanded_obstacles_block = [
    obstacle.get_bounded_rectangle() for obstacle in expanded_obstacles
]
# map = VisibilityGraph(expanded_obstacles_v, points, Crescendo.field_size)
map = TriangulatedGraph(
    slightly_more_expanded_obstacles, Feet(2), Crescendo.get_field_size()
)


def CreatePath(
    start: Tuple[SpatialMeasurement, SpatialMeasurement],
    end: Tuple[SpatialMeasurement, SpatialMeasurement],
) -> pathfinding.Path:
    path = pathfinding.findPath(
        map,
        start,
        end,
        pathfinding.AStar,
        pathfinding.InitialConnectionPolicy.ConnectToClosest,
    )
    path.shortcut(expanded_obstacles)
    return path


corridors = []
paths = []


def CreateTrajectory(
    start: Tuple[SpatialMeasurement, SpatialMeasurement],
    end: Tuple[SpatialMeasurement, SpatialMeasurement],
    start_angle: AngularMeasurement = Degree(0),
    end_angle: AngularMeasurement = Degree(0),
):
    path = CreatePath(start, end)

    builder = SwerveTrajectoryProblemBuilder()
    builder.waypoint(
        Waypoint(start[0], start[1]).given(
            VelocityEquals(MetersPerSecond(0), MetersPerSecond(0)),
        )
    )
    builder.waypoint(
        Waypoint(end[0], end[1]).given(
            VelocityEquals(MetersPerSecond(0), MetersPerSecond(0)),
        )
    )
    builder.guide_pathes([path])
    builder.points_constraint(SafetyCorridor(expanded_obstacles))

    trajectory = builder.generate(
        TrajectoryBuilderConfig(
            trajectory_resolution=Centimeter(15),
            stretch_factor=1.5,
            min_spacing=Centimeter(5),
        )
    ).solve(
        SwerveRobotConstraints(
            MeterPerSecondSquared(8),
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
        ),
        SolverConfig(timeout=10, max_iterations=10000, solution_tolerance=1e-9),
    )

    return trajectory


def Destinations(
    places: List[Tuple[SpatialMeasurement, SpatialMeasurement]]
) -> List[SwerveTrajectory]:
    out = []
    for i in range(len(places) - 1):
        start = places[i]
        end = places[i + 1]
        out.append(CreateTrajectory(start, end))
    return out


# safe_corridor = trajectory_generator.GetSafeCorridor()
Current = (Feet(6), Inch(64.081) + Inch(82.645) / 2)
trajectories = []
renderer = Renderer()

renderer.set_game(Crescendo)
renderer.set_render_scale(Centimeter(1))
renderer.init_display()
print("Game set and display initialized")





loop = True
clock = pygame.time.Clock()
current_time = Second(0)

while loop:
    events = renderer.loop()
    if events is False:
        loop = False
        break
        
    dt_ms = clock.tick(60)
    current_time += Second(dt_ms / 1000.0)

    for event in events:
        if event.type == pygame.MOUSEBUTTONUP:
            pos = pygame.mouse.get_pos()
            x, y = (
                Renderer.render_scale * pos[0],
                Renderer.render_scale * pos[1],
            )

            trajectories.append(CreateTrajectory(Current, (x, y)))
            Current = (x, y)
        if event.type == pygame.KEYUP:
            if event.key == pygame.K_c:
                trajectories = []
                paths = []
                current_time = Second(0)

    renderer.draw_element(map)

    renderer.draw_elements(expanded_obstacles)
    renderer.draw_elements(trajectories)
    renderer.draw_elements(paths)
    renderer.draw_static_elements()
    
    # Animation Logic
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
                    # distinct time within this trajectory
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

    renderer.render_frame()
