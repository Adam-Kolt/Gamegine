#!/usr/bin/env python3
"""
Test script for validating safety corridor obstacle avoidance.
Tests that:
1. Corridors are generated for the entire path (no gaps)
2. All trajectory points are within a corridor
3. Trajectory does not intersect obstacles
"""

import sys
sys.path.insert(0, '.')

from gamegine.utils.NCIM.Dimensions.spatial import Meter, Centimeter, Feet, Inch
from gamegine.utils.NCIM.ComplexDimensions.velocity import MetersPerSecond
from gamegine.utils.NCIM.ComplexDimensions.acceleration import MeterPerSecondSquared
from gamegine.utils.NCIM.ComplexDimensions.alpha import RadiansPerSecondSquared
from gamegine.utils.NCIM.ComplexDimensions.omega import RadiansPerSecond
from gamegine.utils.NCIM.Dimensions.angular import Degree
from gamegine.representation.bounds import Rectangle

from examples.crescendo.crescendo import Crescendo
from gamegine.analysis import pathfinding
from gamegine.analysis.trajectory.lib.TrajGen import (
    SwerveTrajectoryProblemBuilder,
    TrajectoryBuilderConfig,
    SwerveRobotConstraints,
    Waypoint,
)
from gamegine.analysis.trajectory.lib.constraints.avoidance import (
    SafetyCorridor,
    GenerateSafeCorridors,
    SAFETY_CORRIDOR_DEBUG,
)
from gamegine.analysis.trajectory.lib.constraints.constraints import VelocityEquals

def run_test():
    print("=" * 60)
    print("Safety Corridor Obstacle Avoidance Test")
    print("=" * 60)
    
    # Setup
    game = Crescendo
    obstacles = game.get_obstacle_bounds()
    expanded_obstacles = [obs.buffered(Inch(18)) for obs in obstacles]
    
    # Create pathfinding mesh
    mesh = pathfinding.createMesh(expanded_obstacles, Feet(2), game.get_field_size())
    
    # Test case: Path that must go around obstacles
    start = (Meter(1.46), Meter(1.25))
    end = (Meter(13.11), Meter(5.27))
    
    print(f"\nTest: {start} -> {end}")
    
    # Find A* path
    path = pathfinding.findPath(start, end, expanded_obstacles, mesh)
    if path is None:
        print("FAIL: No path found!")
        return False
    
    astar_points = path.get_points()
    print(f"A* path has {len(astar_points)} points")
    
    # Build trajectory
    builder = SwerveTrajectoryProblemBuilder()
    builder.waypoint(Waypoint(start[0], start[1]).given(VelocityEquals(MetersPerSecond(0), MetersPerSecond(0))))
    builder.waypoint(Waypoint(end[0], end[1]).given(VelocityEquals(MetersPerSecond(0), MetersPerSecond(0))))
    builder.guide_pathes([path])
    builder.points_constraint(SafetyCorridor(expanded_obstacles, guide_path=astar_points))
    
    config = TrajectoryBuilderConfig(
        trajectory_resolution=Centimeter(15),
        stretch_factor=1.5,
        min_spacing=Centimeter(5),
    )
    
    # Generate problem (this also generates corridors)
    SAFETY_CORRIDOR_DEBUG.clear()
    problem = builder.generate(config)
    
    print(f"\nGenerated {len(SAFETY_CORRIDOR_DEBUG)} corridors")
    
    # Solve
    trajectory = problem.solve(SwerveRobotConstraints(
        MeterPerSecondSquared(5),
        MetersPerSecond(6),
        RadiansPerSecondSquared(3.14),
        RadiansPerSecond(3.14),
    ))
    
    print(f"Trajectory has {len(trajectory.points)} points")
    print(f"Solver exit status: {problem.problem.status()}")
    
    # Validation 1: Check corridor coverage
    print("\n--- Validation 1: Corridor Coverage ---")
    corridors = SAFETY_CORRIDOR_DEBUG
    uncovered_points = 0
    
    for i, point in enumerate(trajectory.points):
        px = float(point.x.to(Meter))
        py = float(point.y.to(Meter))
        
        in_corridor = False
        for c in corridors:
            min_x = float(c.get_min_x().to(Meter))
            max_x = float(c.get_max_x().to(Meter))
            min_y = float(c.get_min_y().to(Meter))
            max_y = float(c.get_max_y().to(Meter))
            
            if min_x <= px <= max_x and min_y <= py <= max_y:
                in_corridor = True
                break
        
        if not in_corridor:
            uncovered_points += 1
            if uncovered_points <= 5:  # Only show first 5
                print(f"  Point {i} at ({px:.2f}, {py:.2f}) NOT in any corridor!")
    
    if uncovered_points > 0:
        print(f"  WARNING: {uncovered_points} points not covered by corridors")
    else:
        print("  OK: All trajectory points are within corridors")
    
    # Validation 2: Check obstacle intersections
    print("\n--- Validation 2: Obstacle Intersections ---")
    obstacle_rects = [obs.get_bounded_rectangle() for obs in expanded_obstacles]
    
    intersections = 0
    for i, point in enumerate(trajectory.points):
        px = float(point.x.to(Meter))
        py = float(point.y.to(Meter))
        
        for j, obs in enumerate(obstacle_rects):
            min_x = float(obs.get_min_x().to(Meter))
            max_x = float(obs.get_max_x().to(Meter))
            min_y = float(obs.get_min_y().to(Meter))
            max_y = float(obs.get_max_y().to(Meter))
            
            if min_x <= px <= max_x and min_y <= py <= max_y:
                intersections += 1
                if intersections <= 5:
                    print(f"  Point {i} at ({px:.2f}, {py:.2f}) INSIDE obstacle {j}!")
                break
    
    if intersections > 0:
        print(f"  FAIL: {intersections} points inside obstacles!")
    else:
        print("  OK: No trajectory points inside obstacles")
    
    # Validation 3: Check trajectory deviation
    print("\n--- Validation 3: Trajectory Deviation ---")
    direct_distance = ((end[0] - start[0])**2 + (end[1] - start[1])**2)**0.5
    actual_distance = trajectory.get_travel_length()
    deviation_ratio = float(actual_distance.to(Meter)) / float(direct_distance.to(Meter))
    
    print(f"  Direct distance: {direct_distance.to(Meter):.2f} m")
    print(f"  Actual distance: {actual_distance.to(Meter):.2f} m")
    print(f"  Deviation ratio: {deviation_ratio:.2f}x")
    
    if deviation_ratio > 1.1:
        print("  OK: Trajectory deviates to avoid obstacles")
    else:
        print("  WARNING: Trajectory may be taking a straight line (ignoring obstacles)")
    
    # Summary
    print("\n" + "=" * 60)
    if intersections == 0 and uncovered_points == 0:
        print("TEST PASSED: Trajectory avoids obstacles and respects corridors")
        return True
    else:
        print("TEST FAILED: Issues detected")
        return False

if __name__ == "__main__":
    success = run_test()
    sys.exit(0 if success else 1)
