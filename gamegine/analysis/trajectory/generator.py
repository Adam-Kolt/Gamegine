from abc import ABC, abstractmethod
from typing import Any, Tuple, List

from gamegine.analysis.trajectory.lib.TrajGen import SwerveTrajectory
from gamegine.analysis import pathfinding
from gamegine.representation.robot import SwerveRobot
from gamegine.utils.NCIM.Dimensions.spatial import SpatialMeasurement
from gamegine.utils.NCIM.Dimensions.angular import AngularMeasurement

class TrajectoryGenerator(ABC):
    """Abstract base class for all trajectory generators."""
    pass

class OfflineTrajectoryGenerator(TrajectoryGenerator):
    """Abstract base class for offline trajectory generators that compute specific paths."""
    
    @abstractmethod
    def generate(
        self,
        robot_name: str,
        robot: SwerveRobot,
        start_state: Tuple[SpatialMeasurement, SpatialMeasurement, AngularMeasurement],
        target_state: Tuple[SpatialMeasurement, SpatialMeasurement, AngularMeasurement],
        path: pathfinding.Path,
        traversal_space: Any,
        constraints: Any = None,
        no_safety_corridor: bool = False
    ) -> SwerveTrajectory:
        """
        Generates a full trajectory from start to target.
        
        :param robot_name: Name of the robot
        :param robot: The robot instance containing physical and drivetrain config
        :param start_state: Tuple of (x, y, heading) for start
        :param target_state: Tuple of (x, y, heading) for target
        :param path: The geometric path to follow guide
        :param traversal_space: Space containing obstacles and mesh
        :param constraints: Optional additional constraints
        :param no_safety_corridor: Whether to ignore safety corridor constraints
        :return: Generated SwerveTrajectory
        """
        pass

class OnlineTrajectoryGenerator(TrajectoryGenerator):
    """Abstract base class for online trajectory generators (MPC, etc)."""
    
    @abstractmethod
    def step(
        self, 
        current_state: Any, 
        target_state: Any, 
        constraints: Any
    ) -> Any:
        """
        Calculates the next control output based on current state.
        STUB: To be implemented in future phases.
        """
        pass
