from typing import Callable, List, Union, Tuple
import shapely.geometry as sg
from gamegine.representation.bounds import Boundary
from gamegine.representation.robot import Robot, SwerveRobot
from gamegine.simulation.game import GameState
from gamegine.simulation.logic import Condition
from gamegine.utils.NCIM.Dimensions.spatial import Meter

# --- Combinators ---

class And(object):
    def __init__(self, conditions: List[Condition]):
        self.conditions = conditions
    def __call__(self, gs: GameState) -> bool:
        return all(c(gs) for c in self.conditions)

class Or(object):
    def __init__(self, conditions: List[Condition]):
        self.conditions = conditions
    def __call__(self, gs: GameState) -> bool:
        return any(c(gs) for c in self.conditions)

class Not(object):
    def __init__(self, condition: Condition):
        self.condition = condition
    def __call__(self, gs: GameState) -> bool:
        return not self.condition(gs)

class Xor(object):
    def __init__(self, c1: Condition, c2: Condition):
        self.c1 = c1
        self.c2 = c2
    def __call__(self, gs: GameState) -> bool:
        return self.c1(gs) ^ self.c2(gs)

# --- Time ---

def TimeGreaterThan(time: float) -> Condition:
    def condition(gs: GameState) -> bool:
        return gs.current_time.get() > time
    return condition

# --- Geometry ---

def RobotInZone(robot_name: str, zone: Boundary) -> Condition:
    """
    Returns a condition that is True if the robot's center is within the zone's 2D projection.
    
    :param robot_name: Name of the robot to check.
    :param zone: The Boundary object representing the zone.
    """
    # Pre-compute zone polygon if it is static.
    # Note: If zone moves, this optimization is invalid. Assuming static zones for now.
    vertices = zone.get_shape_world_vertices_2d()
    # Convert to pure float tuples for Shapely, assuming Meters.
    # Since bounds.py uses SpatialMeasurement, we convert to Meter.
    poly_points = [(v[0].to(Meter), v[1].to(Meter)) for v in vertices]
    zone_poly = sg.Polygon(poly_points)

    def condition(gs: GameState) -> bool:
        robots_space = gs.get("robots")
        if not robots_space:
            return False
            
        robot_state = robots_space.values.get(robot_name)
        if not robot_state: 
            # Check if it is a sub-space (as verified in robot.py -> RobotState is a StateSpace)
            # `robots` space contains `RobotState` objects as subspaces probably?
            # Re-reading match.py: self.game_state.createSpace("robots").
            # Re-reading robot.py: RobotState IS a StateSpace.
            # So gs.get("robots") returns a StateSpace.
            # We need to find the RobotState within it.
            # "robots" space likely has children spaces keyed by robot name.
            # Let's assume standard structure: gs.get("robots").get(robot_name) -> RobotState
            if robot_name not in robots_space.spaces:
                 return False
            robot_state = robots_space.get(robot_name)
            
        else:
             # It was found in values? Unlikely if it is a RobotState object.
             # Wait, generic StateSpace doesn't mix values and spaces well for the same key.
             # Assume it is a space.
             pass
             
        # Actually logic above is messy. Let's stick to standard access.
        try:
             robot_state = gs.get("robots").get(robot_name)
        except KeyError:
             return False

        # Access x, y ValueEntries
        try:
            rx = robot_state.getValue("x").get().to(Meter)
            ry = robot_state.getValue("y").get().to(Meter)
        except KeyError:
            return False # State not fully initialized
            
        return zone_poly.contains(sg.Point(rx, ry))

    return condition


def RobotsContacting(robot1_name: str, robot2_name: str, robots_config: dict[str, Robot]) -> Condition:
    """
    Returns a condition that is True if the two robots are touching (distance < sum of radii).
    
    :param robot1_name: Name of first robot.
    :param robot2_name: Name of second robot.
    :param robots_config: Dictionary mapping robot names to Robot definitions (for radii).
    """
    r1_radius = robots_config[robot1_name].get_bounding_radius().to(Meter)
    r2_radius = robots_config[robot2_name].get_bounding_radius().to(Meter)
    threshold_sq = (r1_radius + r2_radius) ** 2

    def condition(gs: GameState) -> bool:
        try:
            r1_state = gs.get("robots").get(robot1_name)
            r2_state = gs.get("robots").get(robot2_name)
            
            x1 = r1_state.getValue("x").get().to(Meter)
            y1 = r1_state.getValue("y").get().to(Meter)
            
            x2 = r2_state.getValue("x").get().to(Meter)
            y2 = r2_state.getValue("y").get().to(Meter)
            
            dist_sq = (x1 - x2)**2 + (y1 - y2)**2
            return dist_sq <= threshold_sq
        except KeyError:
            return False

    return condition

def RobotTouchingObject(robot_name: str, object_boundary: Boundary) -> Condition:
    """
    Returns a condition that is True if the robot is touching the object (circle-polygon collision).
    This is approximate: checks if robot center is within distance `radius` of the object polygon.
    """
    # Pre-compute object polygon
    vertices = object_boundary.get_shape_world_vertices_2d()
    poly_points = [(v[0].to(Meter), v[1].to(Meter)) for v in vertices]
    obj_poly = sg.Polygon(poly_points)
    
    # We need the robot radius. This makes the signature tricky. 
    # Ideally we'd pass the Robot definition here or a radius.
    # Let's accept a radius or require binding it.
    # Better: factory that takes the robot CONFIG_DICT so we can lookup radius.
    pass 
    
# Revised factory for broader usage
def RobotTouchingZoneWithRadius(robot_name: str, zone: Boundary, radius: float) -> Condition:
    """
    Checks if robot (approximated as circle of radius `radius`) touches the zone.
    """
    vertices = zone.get_shape_world_vertices_2d()
    poly_points = [(v[0].to(Meter), v[1].to(Meter)) for v in vertices]
    zone_poly = sg.Polygon(poly_points)
    
    def condition(gs: GameState) -> bool:
        try:
            r_state = gs.get("robots").get(robot_name)
            rx = r_state.getValue("x").get().to(Meter)
            ry = r_state.getValue("y").get().to(Meter)
            
            point = sg.Point(rx, ry)
            distance = zone_poly.distance(point) # 0 if inside
            return distance <= radius
        except KeyError:
            return False
    return condition
