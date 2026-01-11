from __future__ import annotations

# Rendering is handled by gamegine.render.handlers, not embedded here
from gamegine.representation.bounds import BoundedObject
from gamegine.simulation.state import StateSpace, ValueEntry
from gamegine.utils.NCIM.Dimensions.spatial import SpatialMeasurement, Meter
from gamegine.utils.NCIM.Dimensions.angular import AngularMeasurement, Degree, Radian
from typing import TYPE_CHECKING, Dict, List, Optional, Tuple
from dataclasses import dataclass, field
import math

if TYPE_CHECKING:
    from gamegine.representation.gamepiece import Gamepiece
    from gamegine.analysis.trajectory.lib.TrajGen import SwerveTrajectory


# =============================================================================
# SIDE ANGLE MAPPING
# =============================================================================

# Center angles for each side (relative to robot heading, 0 = forward)
SIDE_ANGLES = {
    "front": 0.0,
    "right": math.pi / 2,    # 90°
    "back": math.pi,         # 180°
    "left": -math.pi / 2,    # -90° (or 270°)
}


def normalize_angle(angle: float) -> float:
    """Normalize angle to [-pi, pi]."""
    while angle > math.pi:
        angle -= 2 * math.pi
    while angle < -math.pi:
        angle += 2 * math.pi
    return angle


def is_angle_in_arc(angle: float, center: float, arc_half: float) -> bool:
    """Check if angle is within arc centered at center with half-width arc_half."""
    diff = normalize_angle(angle - center)
    return abs(diff) <= arc_half


# =============================================================================
# GAMEPIECE STATE WITH VELOCITY
# =============================================================================

class GamepieceState(StateSpace):
    """State of a gamepiece including position, velocity, and ownership.

    :param x: The x-coordinate of the gamepiece.
    :param y: The y-coordinate of the gamepiece.
    :param owner: Name of the robot holding this piece, or None if on field.
    :param vel_x: X velocity (m/s), for pushed pieces.
    :param vel_y: Y velocity (m/s), for pushed pieces.
    """

    def __init__(
        self,
        x: SpatialMeasurement,
        y: SpatialMeasurement,
        owner: Optional[str] = None,
        vel_x: float = 0.0,
        vel_y: float = 0.0,
    ):
        super().__init__()
        self.setValue("x", x)
        self.setValue("y", y)
        self._owner = owner
        self.vel_x = vel_x  # m/s
        self.vel_y = vel_y  # m/s

    @property
    def x(self) -> ValueEntry[SpatialMeasurement]:
        return self.getValue("x")

    @x.setter
    def x(self, x: SpatialMeasurement) -> GamepieceState:
        self.setValue("x", x)
        return self

    @property
    def y(self) -> ValueEntry[SpatialMeasurement]:
        return self.getValue("y")

    @y.setter
    def y(self, y: SpatialMeasurement) -> GamepieceState:
        self.setValue("y", y)
        return self

    @property
    def owner(self) -> Optional[str]:
        return self._owner
    
    @owner.setter
    def owner(self, value: Optional[str]):
        self._owner = value
    
    def is_on_field(self) -> bool:
        """Returns True if this piece is on the field (not held by a robot)."""
        return self._owner is None
    
    def update_physics(self, dt: float, friction: float = 3.0):
        """Update piece position based on velocity, apply friction.
        
        :param dt: Time step in seconds
        :param friction: Friction coefficient (velocity decay per second)
        """
        if not self.is_on_field():
            return  # Held pieces don't move
        
        if abs(self.vel_x) < 0.01 and abs(self.vel_y) < 0.01:
            return  # Not moving
        
        # Update position
        x_m = float(self.x.value.to(Meter))
        y_m = float(self.y.value.to(Meter))
        
        x_m += self.vel_x * dt
        y_m += self.vel_y * dt
        
        self.x = Meter(x_m)
        self.y = Meter(y_m)
        
        # Apply friction (exponential decay)
        decay = math.exp(-friction * dt)
        self.vel_x *= decay
        self.vel_y *= decay
        
        # Stop if very slow
        if abs(self.vel_x) < 0.05:
            self.vel_x = 0.0
        if abs(self.vel_y) < 0.05:
            self.vel_y = 0.0


class GamepieceInstance(BoundedObject):
    def __init__(
        self, gamepiece: Gamepiece, x: SpatialMeasurement, y: SpatialMeasurement
    ):
        super().__init__(gamepiece.bounds)
        self.gamepiece = gamepiece
        self.x = x
        self.y = y


# =============================================================================
# INTAKE CONFIG (ENHANCED)
# =============================================================================

@dataclass
class IntakeConfig:
    """Configuration for a robot's intake system.
    
    :param sides: List of sides with intake capability (e.g., ["front", "left"])
    :param pickup_radius: Distance from robot center for auto-pickup
    :param capacity: Maximum number of pieces the robot can hold
    :param intake_arc: Angular width of each intake side (default 90°)
    """
    sides: List[str]
    pickup_radius: SpatialMeasurement
    capacity: int
    intake_arc: AngularMeasurement = field(default_factory=lambda: Degree(90))


# =============================================================================
# SHOT RESULT
# =============================================================================

@dataclass
class ShotResult:
    """Result of a shooting attempt."""
    success: bool
    piece_id: str
    landing_x: SpatialMeasurement
    landing_y: SpatialMeasurement
    flight_time: float  # seconds


# =============================================================================
# GAMEPIECE MANAGER (ENHANCED)
# =============================================================================

class GamepieceManager:
    """Manages gamepieces with physics-based interactions."""
    
    def __init__(self):
        self.pieces: Dict[str, GamepieceState] = {}
        self._piece_counter = 0
        # Intake state: robot_name -> is_intaking
        self.intake_active: Dict[str, bool] = {}
    
    def spawn_piece(
        self, 
        x: SpatialMeasurement, 
        y: SpatialMeasurement, 
        name: Optional[str] = None
    ) -> str:
        """Spawn a new gamepiece at the given location."""
        if name is None:
            name = f"piece_{self._piece_counter}"
            self._piece_counter += 1
        
        self.pieces[name] = GamepieceState(x, y, owner=None)
        return name
    
    def get_piece(self, piece_id: str) -> Optional[GamepieceState]:
        """Get a piece by ID."""
        return self.pieces.get(piece_id)
    
    def get_available_pieces(self) -> List[str]:
        """Get IDs of all pieces currently on the field (not held)."""
        return [pid for pid, state in self.pieces.items() if state.is_on_field()]
    
    def get_robot_pieces(self, robot_name: str) -> List[str]:
        """Get IDs of all pieces held by a robot."""
        return [pid for pid, state in self.pieces.items() if state.owner == robot_name]
    
    # -------------------------------------------------------------------------
    # INTAKE STATE
    # -------------------------------------------------------------------------
    
    def set_intake_active(self, robot_name: str, active: bool):
        """Set whether a robot's intake is active."""
        self.intake_active[robot_name] = active
    
    def is_intake_active(self, robot_name: str) -> bool:
        """Check if a robot's intake is active."""
        return self.intake_active.get(robot_name, False)
    
    # -------------------------------------------------------------------------
    # INTAKE SIDE DETECTION
    # -------------------------------------------------------------------------
    
    def is_piece_on_intake_side(
        self,
        piece_x: SpatialMeasurement,
        piece_y: SpatialMeasurement,
        robot_x: SpatialMeasurement,
        robot_y: SpatialMeasurement,
        robot_heading: AngularMeasurement,
        intake_config: IntakeConfig,
    ) -> bool:
        """Check if a piece is on one of the robot's intake sides.
        
        :param piece_x: Piece X position
        :param piece_y: Piece Y position
        :param robot_x: Robot X position
        :param robot_y: Robot Y position
        :param robot_heading: Robot heading (0 = forward along +x)
        :param intake_config: Intake configuration
        :returns: True if piece is within an intake side's arc
        """
        # Vector from robot to piece
        dx = float(piece_x.to(Meter)) - float(robot_x.to(Meter))
        dy = float(piece_y.to(Meter)) - float(robot_y.to(Meter))
        
        if abs(dx) < 0.001 and abs(dy) < 0.001:
            return True  # On top of robot
        
        # Angle from robot to piece (world frame)
        angle_to_piece = math.atan2(dy, dx)
        
        # Convert to robot-relative angle
        heading_rad = float(robot_heading.to(Radian))
        relative_angle = normalize_angle(angle_to_piece - heading_rad)
        
        # Check each intake side
        arc_half = float(intake_config.intake_arc.to(Radian)) / 2
        
        for side in intake_config.sides:
            if side not in SIDE_ANGLES:
                continue
            side_center = SIDE_ANGLES[side]
            if is_angle_in_arc(relative_angle, side_center, arc_half):
                return True
        
        return False
    
    # -------------------------------------------------------------------------
    # PICKUP (ENHANCED)
    # -------------------------------------------------------------------------
    
    def pickup_piece(self, piece_id: str, robot_name: str) -> bool:
        """Attempt to pick up a piece (basic - no side check)."""
        if piece_id not in self.pieces:
            return False
        
        state = self.pieces[piece_id]
        if not state.is_on_field():
            return False
        
        state.owner = robot_name
        state.vel_x = 0.0
        state.vel_y = 0.0
        return True
    
    def try_pickup_with_intake(
        self,
        robot_name: str,
        robot_x: SpatialMeasurement,
        robot_y: SpatialMeasurement,
        robot_heading: AngularMeasurement,
        intake_config: IntakeConfig,
    ) -> Optional[str]:
        """Try to pick up a piece if intake is active and piece is on intake side.
        
        :returns: piece_id if pickup successful, None otherwise
        """
        if not self.is_intake_active(robot_name):
            return None
        
        held = len(self.get_robot_pieces(robot_name))
        if held >= intake_config.capacity:
            return None
        
        # Find pieces within pickup radius
        nearby = self.get_pieces_near_point(robot_x, robot_y, intake_config.pickup_radius)
        
        for pid in nearby:
            state = self.pieces[pid]
            if self.is_piece_on_intake_side(
                state.x.value, state.y.value,
                robot_x, robot_y, robot_heading,
                intake_config
            ):
                if self.pickup_piece(pid, robot_name):
                    return pid
        
        return None
    
    # -------------------------------------------------------------------------
    # DROP AND SHOOT
    # -------------------------------------------------------------------------
    
    def drop_piece(self, piece_id: str, x: SpatialMeasurement, y: SpatialMeasurement) -> bool:
        """Drop a piece at a location."""
        if piece_id not in self.pieces:
            return False
        
        state = self.pieces[piece_id]
        state.owner = None
        state.x = x
        state.y = y
        state.vel_x = 0.0
        state.vel_y = 0.0
        return True
    
    def shoot_piece(
        self,
        piece_id: str,
        target_x: SpatialMeasurement,
        target_y: SpatialMeasurement,
        shooter_x: SpatialMeasurement,
        shooter_y: SpatialMeasurement,
        base_accuracy: float = 0.9,
        accuracy_falloff: float = 0.05,  # per meter
        deterministic: bool = False,
    ) -> ShotResult:
        """Shoot a held piece toward a target location.
        
        :param piece_id: ID of piece to shoot
        :param target_x: Target X coordinate
        :param target_y: Target Y coordinate
        :param shooter_x: Shooter X position
        :param shooter_y: Shooter Y position
        :param base_accuracy: Accuracy at 0 distance (0.0-1.0)
        :param accuracy_falloff: Accuracy decrease per meter
        :param deterministic: If True, use deterministic outcome
        :returns: ShotResult with landing position
        """
        import random
        
        if piece_id not in self.pieces:
            return ShotResult(False, piece_id, Meter(0), Meter(0), 0.0)
        
        state = self.pieces[piece_id]
        
        # Calculate distance
        dx = float(target_x.to(Meter)) - float(shooter_x.to(Meter))
        dy = float(target_y.to(Meter)) - float(shooter_y.to(Meter))
        distance = math.sqrt(dx*dx + dy*dy)
        
        # Calculate accuracy
        accuracy = base_accuracy - (distance * accuracy_falloff)
        accuracy = max(0.0, min(1.0, accuracy))
        
        # Flight time (assume 5 m/s effective speed)
        flight_time = distance / 5.0
        
        # Determine if shot succeeds (piece lands near target)
        if deterministic:
            success = accuracy > 0.5
        else:
            success = random.random() < accuracy
        
        if success:
            # Land at target
            landing_x = target_x
            landing_y = target_y
        else:
            # Miss - land somewhere nearby with error proportional to distance
            error_radius = distance * 0.3 * (1.0 - accuracy)
            error_angle = random.uniform(0, 2 * math.pi)
            landing_x = Meter(float(target_x.to(Meter)) + error_radius * math.cos(error_angle))
            landing_y = Meter(float(target_y.to(Meter)) + error_radius * math.sin(error_angle))
        
        # Release the piece
        state.owner = None
        state.x = landing_x
        state.y = landing_y
        state.vel_x = 0.0
        state.vel_y = 0.0
        
        return ShotResult(success, piece_id, landing_x, landing_y, flight_time)
    
    # -------------------------------------------------------------------------
    # PUSHING
    # -------------------------------------------------------------------------
    
    def push_pieces_from_robot(
        self,
        robot_x: SpatialMeasurement,
        robot_y: SpatialMeasurement,
        robot_radius: SpatialMeasurement,
        push_speed: float = 2.0,  # m/s
    ) -> List[str]:
        """Push pieces away from robot (when not intaking).
        
        :param robot_x: Robot X position
        :param robot_y: Robot Y position  
        :param robot_radius: Robot collision radius
        :param push_speed: Speed to push pieces (m/s)
        :returns: List of pushed piece IDs
        """
        pushed = []
        rx = float(robot_x.to(Meter))
        ry = float(robot_y.to(Meter))
        radius_m = float(robot_radius.to(Meter))
        
        for pid, state in self.pieces.items():
            if not state.is_on_field():
                continue
            
            px = float(state.x.value.to(Meter))
            py = float(state.y.value.to(Meter))
            
            dx = px - rx
            dy = py - ry
            dist = math.sqrt(dx*dx + dy*dy)
            
            if dist < radius_m and dist > 0.01:
                # Piece is inside robot radius - push it out
                # Direction: from robot center toward piece
                nx = dx / dist
                ny = dy / dist
                
                # Set velocity
                state.vel_x = nx * push_speed
                state.vel_y = ny * push_speed
                
                # Move outside robot radius immediately
                new_x = rx + nx * (radius_m + 0.1)
                new_y = ry + ny * (radius_m + 0.1)
                state.x = Meter(new_x)
                state.y = Meter(new_y)
                
                pushed.append(pid)
        
        return pushed
    
    # -------------------------------------------------------------------------
    # PHYSICS UPDATE
    # -------------------------------------------------------------------------
    
    def update_physics(self, dt: float, friction: float = 3.0):
        """Update physics for all pieces."""
        for state in self.pieces.values():
            state.update_physics(dt, friction)
    
    # -------------------------------------------------------------------------
    # HELPERS
    # -------------------------------------------------------------------------
    
    def get_pieces_near_point(
        self, 
        x: SpatialMeasurement, 
        y: SpatialMeasurement, 
        radius: SpatialMeasurement
    ) -> List[str]:
        """Get pieces within radius of a point."""
        result = []
        radius_m = float(radius.to(Meter))
        x_m = float(x.to(Meter))
        y_m = float(y.to(Meter))
        
        for pid, state in self.pieces.items():
            if not state.is_on_field():
                continue
            
            px = float(state.x.value.to(Meter))
            py = float(state.y.value.to(Meter))
            dist = ((px - x_m)**2 + (py - y_m)**2)**0.5
            
            if dist <= radius_m:
                result.append(pid)
        
        return result
    
    def check_trajectory_pickups(
        self,
        trajectory: 'SwerveTrajectory',
        robot_name: str,
        intake_config: IntakeConfig,
        current_held: int = 0,
    ) -> List[Tuple[float, str]]:
        """Check which pieces would be picked up along a trajectory.
        
        NOTE: This assumes intake is active for the entire trajectory.
        For more realistic behavior, use try_pickup_with_intake per frame.
        """
        pickups = []
        capacity_remaining = intake_config.capacity - current_held
        already_picked = set()
        
        if capacity_remaining <= 0:
            return pickups
        
        cumulative_time = 0.0
        for state in trajectory.points:
            x = state.x
            y = state.y
            heading = state.theta
            
            nearby = self.get_pieces_near_point(x, y, intake_config.pickup_radius)
            
            for pid in nearby:
                if pid in already_picked:
                    continue
                if len(pickups) >= capacity_remaining:
                    break
                
                # Check if on intake side
                piece_state = self.pieces[pid]
                if self.is_piece_on_intake_side(
                    piece_state.x.value, piece_state.y.value,
                    x, y, heading, intake_config
                ):
                    pickups.append((cumulative_time, pid))
                    already_picked.add(pid)
            
            if len(pickups) >= capacity_remaining:
                break
            
            if hasattr(state, 'dt') and state.dt is not None:
                from gamegine.utils.NCIM.Dimensions.temporal import Second
                dt_val = float(state.dt.to(Second)) if hasattr(state.dt, 'to') else float(state.dt)
                cumulative_time += dt_val
        
        return pickups
