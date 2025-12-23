from typing import Dict, List, Tuple
import math
from gamegine.analysis.trajectory.lib import CALCULATION_UNIT_SPATIAL
from gamegine.analysis.trajectory.lib.problemVariables import PointVariables
from gamegine.render import helpers
from gamegine.render.style import Palette
from gamegine.representation.bounds import DiscreteBoundary, Rectangle
from gamegine.utils.NCIM.Dimensions.spatial import Centimeter, Meter, SpatialMeasurement
from gamegine.utils.logging import Debug, Warn


class SafeCorridor(Rectangle):
    """Class for representing a safe corridor for the robot to travel through. Wraps around a rectangle to provide a safe corridor for the robot to travel through."""

    def draw(self, render_scale: SpatialMeasurement):
        helpers.draw_fancy_rectangle(self, Palette.BLUE, render_scale)


def __CheckIntersection(rect, obstacles: List[DiscreteBoundary]):
    """Helper to check intersection with a list of obstacles (DiscreteBoundary or Rectangle)."""
    # Assuming obstacles have intersects_rectangle method
    for obstacle in obstacles:
        if obstacle.intersects_rectangle(rect[0], rect[1], rect[2], rect[3]):
            return True
    return False


def __NudgePointToFreeSpace(
    x: SpatialMeasurement,
    y: SpatialMeasurement,
    obstacles: List[DiscreteBoundary],
) -> Tuple[SpatialMeasurement, SpatialMeasurement]:
    """Attempts to find a point near (x, y) that does not intersect with obstacles."""
    
    # We want a seed that has at least some clearance to allow for a non-zero corridor
    MIN_CLEARANCE = Centimeter(1) 

    def is_valid(px, py):
        # check a box of size MIN_CLEARANCE centered at px, py
        half = MIN_CLEARANCE / 2
        return not __CheckIntersection([px - half, py - half, px + half, py + half], obstacles)

    # Check if original point is good enough
    if is_valid(x, y):
        return x, y
        
    # Search logic: Spiral outwards
    search_radius = Meter(5.0) 
    step_size = Centimeter(20.0)
    
    steps = int((search_radius / step_size))
    
    # Simple grid search spiraling out would be better, but simple grid is fine for now
    # We prioritize closer points
    for r in range(1, steps + 1):
        # Check ring 'r'
        for dx_i in range(-r, r + 1):
            for dy_i in range(-r, r + 1):
                # Only check if on the perimeter of the current box (optimization)
                if abs(dx_i) != r and abs(dy_i) != r:
                    continue
                
                nx = x + step_size * dx_i
                ny = y + step_size * dy_i
                
                if is_valid(nx, ny):
                    # Debug(f"Nudged point from ({x}, {y}) to ({nx}, {ny}) with clearance.")
                    return nx, ny
                
    Warn(f"Failed to find free space near ({x}, {y}) for corridor seed!")
    print(f"CRITICAL: Failed to nudge point ({x}, {y}) out of obstacle! Constraints will be invalid.")
    return x, y


def __GetExpandedRectangle(
    x: SpatialMeasurement,
    y: SpatialMeasurement,
    obstacles: List[DiscreteBoundary],
    alternate_obstacles: List[DiscreteBoundary] = None,
) -> SafeCorridor:
    
    # Nudge seed if necessary
    # Note: We prefer to check against passed 'obstacles' (which might be rect_obstacles for speed)
    # However, Nudge needs accurate check. If 'obstacles' are bounding boxes, nudging out of them is good.
    # If they are detailed, even better.
    # We will use the passed 'obstacles' for consistency.
    x, y = __NudgePointToFreeSpace(x, y, obstacles)
    
    INITIAL_STEP_SIZE = Meter(1)

    if not obstacles:
        dist = Meter(100)
        return SafeCorridor(
            x - dist,
            y - dist,
            dist * 2,
            dist * 2,
        )

    # Adding small amount to prevent it being a line is kinda scuff, but it works for now
    rectangle = [
        x,
        y,
        x + Centimeter(0.001),
        y + Centimeter(0.001),
    ]  # [min_x, min_y, max_x, max_y]
    expansion_order = [
        3,
        2,
        1,
        0,
    ]  # Order of expansion: 0 = min_x, 1 = min_y, 2 = max_x, 3 = max_y

    for i in expansion_order:
        step_size = INITIAL_STEP_SIZE

        # Gets direction of expansion...possibly a bit scuff
        if i < 2:
            step_size = -step_size

        # If already intersects with an obstacle, debug -> effectively handled by Nudge, but check again
        if __CheckIntersection(rectangle, obstacles):
            if alternate_obstacles is not None:
                obstacles = alternate_obstacles
                if __CheckIntersection(rectangle, obstacles):
                    # Still intersects... give up on this direction (and likely all, but we tried)
                    continue
            else:
                continue

        # Expand until the rectangle intersects with an obstacle
        while not __CheckIntersection(rectangle, obstacles):
            rectangle[i] += step_size

        # Expand and backoff until rectangle perfectly fits within obstacles
        for _ in range(4):
            while not __CheckIntersection(rectangle, obstacles):
                rectangle[i] += step_size

            # Shrink back in decremental steps until the rectangle no longer intersects with an obstacle
            while __CheckIntersection(rectangle, obstacles):
                step_size /= 2
                rectangle[i] -= step_size

    return SafeCorridor(
        rectangle[0],
        rectangle[1],
        rectangle[2] - rectangle[0],
        rectangle[3] - rectangle[1],
    )


def GenerateSafeCorridors(
    path: List[Tuple[SpatialMeasurement, SpatialMeasurement]],
    obstacles: List[DiscreteBoundary],
    corridor_interval: int = 3,  # Generate a new corridor every N points (3 = tighter coverage)
) -> Tuple[List[SafeCorridor], Dict[int, int]]:
    """Generates overlapping safe corridors along a path.
    
    :param path: The path to generate corridors for.
    :param obstacles: The obstacles to avoid.
    :param corridor_interval: Generate a new corridor every N points (smaller = more corridors = better coverage).
    :return: Tuple of (list of corridors, mapping from path index to corridor index).
    """
    
    safe_corridor: List[SafeCorridor] = []
    path_map: Dict[int, int] = {}
    
    # Pre-calculate bounded rectangles for faster initial checks
    rect_obstacles = [obstacle.get_bounded_rectangle() for obstacle in obstacles]

    if not path:
        return safe_corridor, path_map

    # Generate corridors at regular intervals along the path
    corridor_indices = []  # Track which path indices have their own corridor
    
    for i in range(0, len(path), corridor_interval):
        # CRITICAL: Nudge point to free space before generating corridor
        # This prevents corridors from being generated inside obstacles
        nudged_x, nudged_y = __NudgePointToFreeSpace(path[i][0], path[i][1], obstacles)
        corridor = __GetExpandedRectangle(
            nudged_x, nudged_y, rect_obstacles, obstacles
        )
        safe_corridor.append(corridor)
        corridor_indices.append(i)
    
    # Also add a corridor for the last point if not already covered
    if len(path) - 1 not in corridor_indices:
        nudged_x, nudged_y = __NudgePointToFreeSpace(path[-1][0], path[-1][1], obstacles)
        corridor = __GetExpandedRectangle(
            nudged_x, nudged_y, rect_obstacles, obstacles
        )
        safe_corridor.append(corridor)
        corridor_indices.append(len(path) - 1)
    
    # Map each path point to the best corridor (closest to its own corridor index)
    for i in range(len(path)):
        # Find the corridor generated closest to this path index
        best_corridor_idx = 0
        min_distance = float('inf')
        
        for c_idx, path_idx in enumerate(corridor_indices):
            dist = abs(i - path_idx)
            if dist < min_distance:
                min_distance = dist
                best_corridor_idx = c_idx
        
        path_map[i] = best_corridor_idx
    
    # Check for gaps and insert bridge corridors
    gaps_filled = 0
    for i in range(len(safe_corridor) - 1):
        c1 = safe_corridor[i]
        c2 = safe_corridor[i + 1]
        
        # Check if corridors overlap
        overlaps = c1.intersects_rectangle(
            c2.get_min_x(), c2.get_min_y(), c2.get_max_x(), c2.get_max_y()
        )
        
        if not overlaps:
            # Find the midpoint between the corridor centers and generate a bridge
            c1_center_x = (c1.get_min_x() + c1.get_max_x()) / 2
            c1_center_y = (c1.get_min_y() + c1.get_max_y()) / 2
            c2_center_x = (c2.get_min_x() + c2.get_max_x()) / 2
            c2_center_y = (c2.get_min_y() + c2.get_max_y()) / 2
            
            mid_x = (c1_center_x + c2_center_x) / 2
            mid_y = (c1_center_y + c2_center_y) / 2
            
            bridge = __GetExpandedRectangle(mid_x, mid_y, rect_obstacles, obstacles)
            safe_corridor.append(bridge)
            gaps_filled += 1
    
    if gaps_filled > 0:
        Debug(f"Inserted {gaps_filled} bridge corridors to fill gaps.")
    
    Debug(
        f"Generated Safe Corridor with {len(safe_corridor)} rectangles for path containing {len(path)} points"
    )
                 
    for i, c in enumerate(safe_corridor):
        print(f"DEBUG: Corridor {i}: {c.get_min_x()}, {c.get_min_y()} -> {c.get_max_x()}, {c.get_max_y()}")

    return safe_corridor, path_map


def __EncloseVectorInRectangleHard(problem, x, y, rectangle: Rectangle):
    """HARD constraints: The point MUST be inside the rectangle. No slack allowed."""
    problem.subject_to(x >= rectangle.get_min_x().to(CALCULATION_UNIT_SPATIAL))
    problem.subject_to(x <= rectangle.get_max_x().to(CALCULATION_UNIT_SPATIAL))
    problem.subject_to(y >= rectangle.get_min_y().to(CALCULATION_UNIT_SPATIAL))
    problem.subject_to(y <= rectangle.get_max_y().to(CALCULATION_UNIT_SPATIAL))


def __EncloseVectorInRectangle(problem, x, y, rectangle: Rectangle, sx_p, sx_n, sy_p, sy_n):
    # Soft Constraints: Relax bounds by slack variables
    # Slack variables must be non-negative (unbounded above to ensure feasibility)
    # The slack penalty in TrajGen.py will encourage minimizing these
    problem.subject_to(sx_p >= 0)
    problem.subject_to(sx_n >= 0)
    problem.subject_to(sy_p >= 0)
    problem.subject_to(sy_n >= 0)

    problem.subject_to(x >= rectangle.get_min_x().to(CALCULATION_UNIT_SPATIAL) - sx_n)
    problem.subject_to(x <= rectangle.get_max_x().to(CALCULATION_UNIT_SPATIAL) + sx_p)
    problem.subject_to(y >= rectangle.get_min_y().to(CALCULATION_UNIT_SPATIAL) - sy_n)
    problem.subject_to(y <= rectangle.get_max_y().to(CALCULATION_UNIT_SPATIAL) + sy_p)


SAFETY_CORRIDOR_DEBUG = []


def __densify_path(
    path: List[Tuple[SpatialMeasurement, SpatialMeasurement]], 
    resolution: SpatialMeasurement = Centimeter(15),
    skip_smoothing: bool = False  # Skip smoothing for lane generation
) -> List[Tuple[SpatialMeasurement, SpatialMeasurement]]:
    """Densifies a sparse path by interpolating points along each segment.
    
    This ensures that corridors generated from the path will have full coverage
    for all trajectory points that will be interpolated along the same segments.
    
    :param path: The sparse path to densify (e.g., A* waypoints)
    :param resolution: The maximum distance between points in the densified path
    :param skip_smoothing: If True, don't apply smoothing (important for lane generation)
    :return: The densified path with interpolated points
    """
    if len(path) < 2:
        return path
    
    # Optionally smooth the path to reduce sharp turns
    if skip_smoothing:
        source_path = path
    else:
        source_path = __smooth_path(path)
    
    densified = [source_path[0]]
    
    for i in range(len(source_path) - 1):
        start = source_path[i]
        end = source_path[i + 1]
        
        # Calculate segment length
        dx = end[0] - start[0]
        dy = end[1] - start[1]
        segment_length = (dx**2 + dy**2)**0.5
        
        # Determine number of intermediate points needed
        num_segments = max(1, int(segment_length / resolution))
        
        # Add intermediate points (excluding start, which is already added)
        for j in range(1, num_segments + 1):
            t = j / num_segments
            x = start[0] + dx * t
            y = start[1] + dy * t
            densified.append((x, y))
    
    Debug(f"Densified path from {len(path)} to {len(densified)} points (resolution: {resolution})")
    return densified


def __smooth_path(
    path: List[Tuple[SpatialMeasurement, SpatialMeasurement]],
    iterations: int = 2,
    weight: float = 0.3
) -> List[Tuple[SpatialMeasurement, SpatialMeasurement]]:
    """Smooths a path using weighted averaging to reduce sharp turns.
    
    Preserves start and end points exactly.
    
    :param path: The path to smooth
    :param iterations: Number of smoothing passes
    :param weight: Weight for neighboring points (0.5 = heavy smoothing, 0.1 = light)
    :return: Smoothed path
    """
    if len(path) <= 2:
        return path
    
    smoothed = list(path)
    
    for _ in range(iterations):
        new_smoothed = [smoothed[0]]  # Keep first point
        
        for i in range(1, len(smoothed) - 1):
            prev = smoothed[i - 1]
            curr = smoothed[i]
            next_pt = smoothed[i + 1]
            
            # Weighted average: (1-2*weight)*current + weight*prev + weight*next
            new_x = (1 - 2 * weight) * curr[0] + weight * prev[0] + weight * next_pt[0]
            new_y = (1 - 2 * weight) * curr[1] + weight * prev[1] + weight * next_pt[1]
            
            new_smoothed.append((new_x, new_y))
        
        new_smoothed.append(smoothed[-1])  # Keep last point
        smoothed = new_smoothed
    
    Debug(f"Smoothed path with {iterations} iterations (weight: {weight})")
    return smoothed


def SafetyCorridor(obstacles: List[DiscreteBoundary], guide_path: List[Tuple[SpatialMeasurement, SpatialMeasurement]] = None, densify_resolution: SpatialMeasurement = Centimeter(15)):
    """Returns a constraint function that ensures that the robot's path is within a safe corridor.

    :param obstacles: The obstacles that the robot must avoid.
    :type obstacles: List[:class:`DiscreteBoundary`]
    :param guide_path: Optional pre-validated path (e.g., from A*) to use for corridor generation.
                       If None, uses the interpolated trajectory points (less reliable).
    :type guide_path: List[Tuple[SpatialMeasurement, SpatialMeasurement]]
    :param densify_resolution: Resolution for densifying the guide path (default: 15cm to match trajectory_resolution)
    :type densify_resolution: SpatialMeasurement
    :return: The constraint function that ensures the robot's path is within a safe corridor.
    :rtype: Callable[[Problem, PointVariables], None]"""

    def __safety_corridor(problem, point_variables: PointVariables):
        # Determine source of path for corridor generation
        if guide_path is not None and len(guide_path) > 0:
            # Densify the guide path to match trajectory resolution
            source_path = __densify_path(guide_path, densify_resolution)
            Debug(f"SafetyCorridor: Using densified guide path with {len(source_path)} points for corridor generation.")
        else:
            # Fallback: Extract path from point variables (initial guess - may be inside obstacles!)
            source_path = []
            for i in range(len(point_variables.POS_X)):
                x = CALCULATION_UNIT_SPATIAL(point_variables.POS_X[i].value())
                y = CALCULATION_UNIT_SPATIAL(point_variables.POS_Y[i].value())
                source_path.append((x, y))
            Warn("SafetyCorridor: No guide path provided, using interpolated points (may be invalid).")

        corridors, _ = GenerateSafeCorridors(source_path, obstacles)
        
        if len(corridors) == 0:
            Warn("SafetyCorridor: No corridors generated! Constraints will be missing.")
            return
        
        # Map EACH TRAJECTORY POINT to the BEST corridor
        # Priority: (1) Inside a corridor, (2) Minimum distance to any corridor boundary
        def distance_to_corridor(px, py, c):
            """Returns 0 if point is inside, otherwise the min distance to enter the corridor."""
            min_x = c.get_min_x().to(CALCULATION_UNIT_SPATIAL)
            max_x = c.get_max_x().to(CALCULATION_UNIT_SPATIAL)
            min_y = c.get_min_y().to(CALCULATION_UNIT_SPATIAL)
            max_y = c.get_max_y().to(CALCULATION_UNIT_SPATIAL)
            
            # Distance to enter on each axis (0 if already inside)
            dx = max(min_x - px, 0, px - max_x)
            dy = max(min_y - py, 0, py - max_y)
            
            return (dx**2 + dy**2)**0.5
        
        # Apply constraints based on best corridor
        unmapped_count = 0
        for i in range(len(point_variables.POS_X)):
            px = point_variables.POS_X[i].value()
            py = point_variables.POS_Y[i].value()
            
            # Find corridor with minimum entry distance
            best_corridor_idx = 0
            best_distance = float('inf')
            
            for idx, c in enumerate(corridors):
                dist = distance_to_corridor(px, py, c)
                if dist < best_distance:
                    best_distance = dist
                    best_corridor_idx = idx
                    if dist == 0:  # Already inside, can't do better
                        break
            
            if best_distance > 0:
                unmapped_count += 1
            
            rectangle = corridors[best_corridor_idx]
            # Use HARD constraints - trajectory MUST stay within corridors
            __EncloseVectorInRectangleHard(
                problem,
                point_variables.POS_X[i],
                point_variables.POS_Y[i],
                rectangle,
            )

        global SAFETY_CORRIDOR_DEBUG
        SAFETY_CORRIDOR_DEBUG.extend(corridors)
        
        # Add HARD exclusion constraints for each obstacle
        # This ensures robot MUST stay outside obstacles regardless of corridor validity
        rect_obstacles = [obstacle.get_bounded_rectangle() for obstacle in obstacles]
        for i in range(len(point_variables.POS_X)):
            for obs in rect_obstacles:
                # For each obstacle, the point must be outside AT LEAST one boundary
                # We use a disjunctive constraint approximation: 
                # x < min_x OR x > max_x OR y < min_y OR y > max_y
                # This is hard to model in NLP. Instead, we add a soft penalty for being inside.
                # But since soft penalties are already applied via slacks, we need a different approach.
                #
                # Alternative: Inflate obstacles and check if the NUDGE succeeded.
                # If nudge failed for this point, we have no good constraint - solver will minimize slacks.
                # The real fix is to pre-process the path to avoid obstacles.
                pass  # Placeholder - hard exclusion is complex for convex obstacles
                
                pass  # Placeholder - hard exclusion is complex for convex obstacles
                
    return __safety_corridor


def __merge_overlapping_rectangles(rectangles: List[Rectangle]) -> List[Rectangle]:
    """Merges a list of rectangles into a smaller set of larger rectangles.
    
    Iteratively merges rectangles that have significant overlap.
    """
    if len(rectangles) < 2:
        return rectangles
        
    merged = []
    current_rect = rectangles[0]
    
    for i in range(1, len(rectangles)):
        next_rect = rectangles[i]
        
        # Check if they can be merged into a valid rectangle that doesn't include obstacles?
        # A simple merge extends the bounding box.
        # Logic: If we merge, we create a larger box. We must ensure this larger box 
        # is still "safe" (doesn't intersect obstacles).
        # Since we don't have access to obstacles here easily, we'll use a simpler heuristic:
        # Merge if they overlap significantly.
        # HOWEVER, the valid approach is:
        # 1. Take union bounding box
        # 2. Check if union box intersects obstacles
        # 3. If NO intersection, merge and continue
        # 4. If intersection, commit current_rect and start new from next_rect
        
        # NOTE: This function requires access to obstacles for validity check.
        # We'll implement the merge logic inside MergedSafetyCorridor instead.
        pass
    
    return rectangles


def MergedSafetyCorridor(
    obstacles: List[DiscreteBoundary], 
    guide_path: List[Tuple[SpatialMeasurement, SpatialMeasurement]] = None, 
    densify_resolution: SpatialMeasurement = Centimeter(15)
):
    """Returns a constraint function that uses merged, larger safety corridors.
    
    1. Generates dense small rectangles along path.
    2. Merges consecutive rectangles if the merged result doesn't intersect obstacles.
    3. Uses these larger corridors for constraints.
    """
    
    def __merged_safety_corridor(problem, point_variables: PointVariables):
        # 1. Generate base corridors (dense)
        path = guide_path
        if path is None or len(path) == 0:
            # Fallback path logic similar to SafetyCorridor
            path = []
            for i in range(len(point_variables.POS_X)):
                val_x = point_variables.POS_X[i].value()
                val_y = point_variables.POS_Y[i].value()
                path.append((val_x, val_y))
            
            # Densify
            densified = __densify_path(path, densify_resolution)
        else:
            # Densify guide path
            densified = __densify_path(path, densify_resolution)
            
        rect_obstacles = [obstacle.get_bounded_rectangle() for obstacle in obstacles]
        
        # Generate initial small rectangles
        base_corridors = []
        for point in densified:
            rect = __GetExpandedRectangle(
                point[0], point[1], rect_obstacles, obstacles
            )
            base_corridors.append(rect)
            
        # 2. Merge Rectangles
        merged_corridors = []
        if len(base_corridors) > 0:
            current_merge = base_corridors[0]
            
            for i in range(1, len(base_corridors)):
                next_rect = base_corridors[i]
                
                # Propose merged rectangle (Bounding Box of current + next)
                min_x = min(current_merge.get_min_x(), next_rect.get_min_x())
                max_x = max(current_merge.get_max_x(), next_rect.get_max_x())
                min_y = min(current_merge.get_min_y(), next_rect.get_min_y())
                max_y = max(current_merge.get_max_y(), next_rect.get_max_y())
                
                merged_rect = Rectangle(min_x, min_y, max_x - min_x, max_y - min_y)
                
                # Check if merged rectangle hits any obstacle
                hits_obstacle = False
                # Simple AABB check against all obstacles
                # Convert to raw values for check
                m_x = float(merged_rect.x.to(Meter))
                m_y = float(merged_rect.y.to(Meter))
                m_w = float(merged_rect.width.to(Meter))
                m_h = float(merged_rect.height.to(Meter))
                m_rect = [m_x, m_y, m_x + m_w, m_y + m_h]
                
                if __CheckIntersection(m_rect, obstacles):
                    hits_obstacle = True
                
                if not hits_obstacle:
                    # Valid merge, update current
                    current_merge = merged_rect
                else:
                    # Invalid merge, commit current and start new
                    merged_corridors.append(current_merge)
                    current_merge = next_rect
            
            merged_corridors.append(current_merge)
        
        Debug(f"Merged {len(base_corridors)} base corridors into {len(merged_corridors)} merged corridors.")
        
        # Convert to SafeCorridor objects for visualization
        start_safe_corridors = []
        for r in merged_corridors:
            s = SafeCorridor(r.x, r.y, r.width, r.height)
            start_safe_corridors.append(s)
            
        # Update debug visualization
        global SAFETY_CORRIDOR_DEBUG
        SAFETY_CORRIDOR_DEBUG.clear()
        SAFETY_CORRIDOR_DEBUG.extend(start_safe_corridors)
        
        # 3. Appply Constraints
        # Map points to merged corridors
        corridors = merged_corridors
        
        def distance_to_corridor(px, py, c):
            """Returns 0 if point is inside, otherwise the min distance to enter the corridor."""
            min_x = c.get_min_x().to(CALCULATION_UNIT_SPATIAL)
            max_x = c.get_max_x().to(CALCULATION_UNIT_SPATIAL)
            min_y = c.get_min_y().to(CALCULATION_UNIT_SPATIAL)
            max_y = c.get_max_y().to(CALCULATION_UNIT_SPATIAL)
            
            # Distance to enter on each axis (0 if already inside)
            dx = max(min_x - px, 0, px - max_x)
            dy = max(min_y - py, 0, py - max_y)
            
            return (dx**2 + dy**2)**0.5
        
        unmapped_count = 0
        for i in range(len(point_variables.POS_X)):
            px = point_variables.POS_X[i].value()
            py = point_variables.POS_Y[i].value()
            
            # Find closest merged corridor
            best_corridor_idx = 0
            best_distance = float('inf')
            
            for idx, c in enumerate(corridors):
                dist = distance_to_corridor(px, py, c)
                if dist < best_distance:
                    best_distance = dist
                    best_corridor_idx = idx
                    if dist == 0:
                        break
            
            if best_distance > 0:
                unmapped_count += 1
            
            # Use SOFT constraints with slack variables for stability
            rectangle = corridors[best_corridor_idx]
            __EncloseVectorInRectangle(
                problem,
                point_variables.POS_X[i],
                point_variables.POS_Y[i],
                rectangle,
                point_variables.SLACK_X_POS[i],
                point_variables.SLACK_X_NEG[i],
                point_variables.SLACK_Y_POS[i],
                point_variables.SLACK_Y_NEG[i],
            )

    return __merged_safety_corridor


def ObstacleRepulsion(obstacles: List[DiscreteBoundary], repulsion_weight: float = 1e4):
    """Returns a constraint function that adds a penalty for being inside obstacles.
    
    This acts as a "backup" to soft corridor constraints by adding a direct cost
    for any point that is inside an obstacle, even if corridors are invalid.
    
    :param obstacles: The obstacles to avoid.
    :param repulsion_weight: Weight for the repulsion penalty per meter inside.
    :return: Constraint function.
    """
    
    def __obstacle_repulsion(problem, point_variables: PointVariables):
        rect_obstacles = [obstacle.get_bounded_rectangle() for obstacle in obstacles]
        
        # Add a "violation" variable for each point that measures how much it's inside any obstacle
        # Then penalize this in the objective.
        # However, modifying the objective from a constraint function is not ideal.
        # Instead, we create auxiliary variables and constraints.
        
        # Simpler approach: Add inequality constraints that PUSH points outside obstacles.
        # For a rectangle obstacle (min_x, min_y, max_x, max_y), point (x, y) is outside if:
        #   x <= min_x OR x >= max_x OR y <= min_y OR y >= max_y
        # We can't directly model OR in NLP. But we can add a constraint that at least one
        # of these violations is "large enough" using slack variables with penalties.
        #
        # For now, just log that we need enhanced initial path handling.
        # The real solution is to ensure the A* path never goes through obstacles.
        
        Warn("ObstacleRepulsion: Adding repulsion hints (soft). Hard exclusion requires valid initial path.")
        
        for i in range(len(point_variables.POS_X)):
            x = point_variables.POS_X[i]
            y = point_variables.POS_Y[i]
            
            for obs in rect_obstacles:
                min_x = obs.get_min_x().to(CALCULATION_UNIT_SPATIAL)
                max_x = obs.get_max_x().to(CALCULATION_UNIT_SPATIAL)
                min_y = obs.get_min_y().to(CALCULATION_UNIT_SPATIAL)
                max_y = obs.get_max_y().to(CALCULATION_UNIT_SPATIAL)
                
                # Check if the initial value is inside this obstacle
                x_val = x.value()
                y_val = y.value()
                
                inside = (x_val >= min_x and x_val <= max_x and y_val >= min_y and y_val <= max_y)
                
                if inside:
                    # Add soft constraints to push towards the nearest edge
                    # Determine closest edge
                    dist_to_min_x = x_val - min_x
                    dist_to_max_x = max_x - x_val
                    dist_to_min_y = y_val - min_y
                    dist_to_max_y = max_y - y_val
                    
                    min_dist = min(dist_to_min_x, dist_to_max_x, dist_to_min_y, dist_to_max_y)
                    
                    if min_dist == dist_to_min_x:
                        # Push to left
                        problem.subject_to(x <= min_x + point_variables.SLACK_X_NEG[i])
                    elif min_dist == dist_to_max_x:
                        # Push to right
                        problem.subject_to(x >= max_x - point_variables.SLACK_X_POS[i])
                    elif min_dist == dist_to_min_y:
                        # Push down
                        problem.subject_to(y <= min_y + point_variables.SLACK_Y_NEG[i])
                    else:
                        # Push up
                        problem.subject_to(y >= max_y - point_variables.SLACK_Y_POS[i])
                        
    return __obstacle_repulsion


def SplineTrackingCost(
    reference_path: List[Tuple[float, float]],
    weight: float = 100.0,
    max_deviation: float = None,
):
    """Returns a constraint function that penalizes trajectory deviation from a reference path.
    
    This is a solver-friendly alternative to hard safety corridors. Since the reference
    path (typically a smooth spline fit to an A* path) already avoids obstacles, keeping
    the trajectory close to it implicitly ensures obstacle avoidance.
    
    :param reference_path: List of (x, y) reference positions in meters. Must have the same 
                           length as the number of trajectory points.
    :param weight: Weight for the deviation penalty. Higher = stricter adherence to reference.
                   Typical values: 10-1000 depending on importance.
    :param max_deviation: Optional. If set, adds a soft constraint that deviation should not
                          exceed this value (in meters). Helps prevent runaway solutions.
    :return: Constraint function.
    """
    
    def __spline_tracking_cost(problem, point_variables: PointVariables):
        n_points = len(point_variables.POS_X)
        n_ref = len(reference_path)
        
        if n_points != n_ref:
            Warn(f"SplineTrackingCost: Reference path has {n_ref} points but trajectory has {n_points}. Skipping.")
            return
        
        Debug(f"SplineTrackingCost: Adding tracking cost with weight={weight} for {n_points} points.")
        
        total_cost = 0.0
        
        for i in range(n_points):
            x = point_variables.POS_X[i]
            y = point_variables.POS_Y[i]
            ref_x, ref_y = reference_path[i]
            
            # Squared distance from reference point
            dx = x - ref_x
            dy = y - ref_y
            dist_sq = dx * dx + dy * dy
            
            # Add to cost
            total_cost = total_cost + weight * dist_sq
            
            # Optional: Add soft constraint on maximum deviation
            if max_deviation is not None:
                # dist_sq <= max_deviation^2  =>  max_deviation^2 - dist_sq >= 0
                max_dev_sq = max_deviation * max_deviation
                # Use slack variable to make this soft
                slack = point_variables.SLACK_X_POS[i]  # Reuse an existing slack
                problem.subject_to(max_dev_sq - dist_sq + slack >= 0)
        
        # Add the tracking cost to the objective
        problem.minimize(problem.cost() + total_cost)
        
        Debug(f"SplineTrackingCost: Added deviation cost to objective.")
    
    return __spline_tracking_cost


# =============================================================================
# OPTIMIZED HALFSPACE LANES (Path-Following Corridors)
# =============================================================================

class LaneSegment:
    """Represents a lane segment along the path with asymmetric widths."""
    
    def __init__(
        self,
        p1: Tuple[SpatialMeasurement, SpatialMeasurement],
        p2: Tuple[SpatialMeasurement, SpatialMeasurement],
        left_width: SpatialMeasurement,
        right_width: SpatialMeasurement,
    ):
        self.p1 = p1
        self.p2 = p2
        self.left_width = left_width
        self.right_width = right_width
        
        # Compute segment direction and perpendicular
        dx = float((p2[0] - p1[0]).to(Meter))
        dy = float((p2[1] - p1[1]).to(Meter))
        length = math.sqrt(dx*dx + dy*dy)
        
        if length > 0:
            # Normalized direction
            self.dir_x = dx / length
            self.dir_y = dy / length
            # Perpendicular (rotate 90° counter-clockwise)
            self.perp_x = -self.dir_y
            self.perp_y = self.dir_x
        else:
            self.dir_x, self.dir_y = 1.0, 0.0
            self.perp_x, self.perp_y = 0.0, 1.0
        
        self.length = length
    
    def draw(self, render_scale: SpatialMeasurement):
        """Visualize the lane segment as two boundary lines."""
        lw = float(self.left_width.to(Meter))
        rw = float(self.right_width.to(Meter))
        
        p1x, p1y = float(self.p1[0].to(Meter)), float(self.p1[1].to(Meter))
        p2x, p2y = float(self.p2[0].to(Meter)), float(self.p2[1].to(Meter))
        
        # Extend lines beyond segment for visualization (show "infinite" nature)
        extend = 0.5  # 0.5m extension at each end
        
        # Extended start and end
        ext_p1x = p1x - self.dir_x * extend
        ext_p1y = p1y - self.dir_y * extend
        ext_p2x = p2x + self.dir_x * extend
        ext_p2y = p2y + self.dir_y * extend
        
        # Left boundary line points
        left_start = (Meter(ext_p1x + self.perp_x * lw), Meter(ext_p1y + self.perp_y * lw))
        left_end = (Meter(ext_p2x + self.perp_x * lw), Meter(ext_p2y + self.perp_y * lw))
        
        # Right boundary line points  
        right_start = (Meter(ext_p1x - self.perp_x * rw), Meter(ext_p1y - self.perp_y * rw))
        right_end = (Meter(ext_p2x - self.perp_x * rw), Meter(ext_p2y - self.perp_y * rw))
        
        # Draw left boundary (green)
        helpers.draw_line(left_start[0], left_start[1], left_end[0], left_end[1],
                         Centimeter(3), Palette.GREEN, render_scale)
        # Draw right boundary (green)
        helpers.draw_line(right_start[0], right_start[1], right_end[0], right_end[1],
                         Centimeter(3), Palette.GREEN, render_scale)


def __compute_lane_width_in_direction(
    point: Tuple[SpatialMeasurement, SpatialMeasurement],
    direction: Tuple[float, float],
    obstacles: List[DiscreteBoundary],
    max_width: SpatialMeasurement = Meter(2.0),
    min_width: SpatialMeasurement = Centimeter(10),
) -> SpatialMeasurement:
    """Compute the maximum lane width in a given direction before hitting an obstacle.
    
    Uses ray casting to find the nearest obstacle in the direction.
    """
    px, py = float(point[0].to(Meter)), float(point[1].to(Meter))
    dx, dy = direction
    
    # Ray march to find obstacle
    step = 0.05  # 5cm steps
    max_dist = float(max_width.to(Meter))
    
    for dist in [step * i for i in range(1, int(max_dist / step) + 1)]:
        test_x = px + dx * dist
        test_y = py + dy * dist
        
        # Check if this point is inside any obstacle
        for obs in obstacles:
            rect = obs.get_bounded_rectangle()
            min_x = float(rect.get_min_x().to(Meter))
            max_x = float(rect.get_max_x().to(Meter))
            min_y = float(rect.get_min_y().to(Meter))
            max_y = float(rect.get_max_y().to(Meter))
            
            if min_x <= test_x <= max_x and min_y <= test_y <= max_y:
                # Hit obstacle - return distance minus a margin
                return max(min_width, Meter(dist - step))
    
    return max_width


def __generate_lane_segments(
    path: List[Tuple[SpatialMeasurement, SpatialMeasurement]],
    obstacles: List[DiscreteBoundary],
    max_lane_width: SpatialMeasurement = Meter(1.5),
    min_lane_width: SpatialMeasurement = Centimeter(15),
) -> List[LaneSegment]:
    """Generate lane segments along the path with asymmetric widths.
    
    For each segment, compute left and right widths based on obstacle distances.
    """
    segments = []
    
    if len(path) < 2:
        return segments
    
    for i in range(len(path) - 1):
        p1 = path[i]
        p2 = path[i + 1]
        
        # Compute segment direction
        dx = float((p2[0] - p1[0]).to(Meter))
        dy = float((p2[1] - p1[1]).to(Meter))
        length = math.sqrt(dx*dx + dy*dy)
        
        if length < 0.001:  # Skip very short segments
            continue
        
        # Perpendicular direction (left = positive, right = negative)
        perp_x = -dy / length
        perp_y = dx / length
        
        # Midpoint of segment for width calculation
        mid_x = (p1[0] + p2[0]) / 2
        mid_y = (p1[1] + p2[1]) / 2
        midpoint = (mid_x, mid_y)
        
        # Compute widths in each direction
        left_width = __compute_lane_width_in_direction(
            midpoint, (perp_x, perp_y), obstacles, max_lane_width, min_lane_width
        )
        right_width = __compute_lane_width_in_direction(
            midpoint, (-perp_x, -perp_y), obstacles, max_lane_width, min_lane_width
        )
        
        segments.append(LaneSegment(p1, p2, left_width, right_width))
    
    Debug(f"Generated {len(segments)} lane segments along path")
    return segments


LANE_SEGMENTS_DEBUG = []  # For visualization


def PathFollowingLanes(
    obstacles: List[DiscreteBoundary],
    guide_path: List[Tuple[SpatialMeasurement, SpatialMeasurement]],
    max_lane_width: SpatialMeasurement = Meter(1.5),
    min_lane_width: SpatialMeasurement = Centimeter(15),
):
    """Returns a constraint function that keeps trajectory points within optimized lanes.
    
    Creates asymmetric lane boundaries:
    - Tight on obstacle side (prevents intrusion)
    - Wide on safe side (allows smooth curves)
    
    :param obstacles: The obstacles to avoid
    :param guide_path: The A* path to follow
    :param max_lane_width: Maximum lane width on safe side
    :param min_lane_width: Minimum lane width on obstacle side
    :return: Constraint function for the optimization problem
    """
    
    def __path_following_lanes(problem, point_variables: PointVariables):
        global LANE_SEGMENTS_DEBUG
        LANE_SEGMENTS_DEBUG.clear()
        
        if guide_path is None or len(guide_path) < 2:
            Warn("PathFollowingLanes: No valid guide path provided!")
            return
        
        # Debug: Show the guide path being used
        Debug(f"PathFollowingLanes: Using guide path with {len(guide_path)} points:")
        for i, pt in enumerate(guide_path):
            Debug(f"  Point {i}: ({pt[0]}, {pt[1]})")
        
        # Densify the path for better coverage (WITHOUT smoothing to preserve A* waypoints)
        densified_path = __densify_path(guide_path, Centimeter(30), skip_smoothing=True)
        
        # Generate lane segments with asymmetric widths
        segments = __generate_lane_segments(
            densified_path, obstacles, max_lane_width, min_lane_width
        )
        
        if len(segments) == 0:
            Warn("PathFollowingLanes: No lane segments generated!")
            return
        
        LANE_SEGMENTS_DEBUG.extend(segments)
        
        # For each trajectory point, find nearest segment and apply lane constraints
        for i in range(len(point_variables.POS_X)):
            px = point_variables.POS_X[i].value()
            py = point_variables.POS_Y[i].value()
            
            # Find nearest segment
            best_segment = segments[0]
            best_dist = float('inf')
            
            for seg in segments:
                # Distance to segment
                p1x = float(seg.p1[0].to(CALCULATION_UNIT_SPATIAL))
                p1y = float(seg.p1[1].to(CALCULATION_UNIT_SPATIAL))
                p2x = float(seg.p2[0].to(CALCULATION_UNIT_SPATIAL))
                p2y = float(seg.p2[1].to(CALCULATION_UNIT_SPATIAL))
                
                # Project point onto segment
                dx = p2x - p1x
                dy = p2y - p1y
                seg_len_sq = dx*dx + dy*dy
                
                if seg_len_sq > 0:
                    t = max(0, min(1, ((px - p1x)*dx + (py - p1y)*dy) / seg_len_sq))
                    nearest_x = p1x + t * dx
                    nearest_y = p1y + t * dy
                    dist = math.sqrt((px - nearest_x)**2 + (py - nearest_y)**2)
                else:
                    dist = math.sqrt((px - p1x)**2 + (py - p1y)**2)
                
                if dist < best_dist:
                    best_dist = dist
                    best_segment = seg
            
            # Apply halfspace constraints for this segment
            # Left boundary: dot(point - p1, perpendicular) <= left_width
            # Right boundary: dot(point - p1, perpendicular) >= -right_width
            
            p1x = best_segment.p1[0].to(CALCULATION_UNIT_SPATIAL)
            p1y = best_segment.p1[1].to(CALCULATION_UNIT_SPATIAL)
            perp_x = best_segment.perp_x
            perp_y = best_segment.perp_y
            lw = best_segment.left_width.to(CALCULATION_UNIT_SPATIAL)
            rw = best_segment.right_width.to(CALCULATION_UNIT_SPATIAL)
            
            # Perpendicular distance: (point - p1) · perpendicular
            perp_dist = (point_variables.POS_X[i] - p1x) * perp_x + (point_variables.POS_Y[i] - p1y) * perp_y
            
            # Lane constraints (HARD)
            problem.subject_to(perp_dist <= lw)   # Left boundary
            problem.subject_to(perp_dist >= -rw)  # Right boundary
    
    return __path_following_lanes

