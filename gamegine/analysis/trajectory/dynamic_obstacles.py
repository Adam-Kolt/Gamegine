"""Dynamic obstacles for time-aware trajectory collision avoidance.

This module provides abstractions for obstacles that vary with time, enabling
multi-robot coordination where trajectories can cross paths spatially as long
as robots don't occupy the same space at the same time.

Key classes:
- DynamicObstacle: Protocol for time-varying obstacles
- TrajectoryObstacle: Wraps a SwerveTrajectory as a moving circular obstacle
"""
from abc import ABC, abstractmethod
from typing import List, Optional, Protocol, Tuple
from dataclasses import dataclass

from gamegine.utils.NCIM.Dimensions.spatial import SpatialMeasurement, Meter


class DynamicObstacle(Protocol):
    """Protocol for obstacles that vary with time.
    
    Used for collision checking during trajectory generation where the
    obstacle's position depends on the query time.
    """
    
    def contains_point_at_time(
        self, 
        x: float, 
        y: float, 
        t: float
    ) -> bool:
        """Check if point (x, y) collides with this obstacle at time t.
        
        :param x: X coordinate in meters
        :param y: Y coordinate in meters  
        :param t: Time in seconds (absolute game time)
        :returns: True if collision detected
        """
        ...
    
    def get_bounding_radius(self) -> float:
        """Get bounding radius for fast rejection tests.
        
        :returns: Radius in meters that bounds the obstacle at any time
        """
        ...
    
    def get_time_bounds(self) -> Tuple[float, float]:
        """Get the time interval during which this obstacle is active.
        
        :returns: (start_time, end_time) in seconds
        """
        ...


@dataclass
class TrajectoryObstacle:
    """A moving circular obstacle defined by a robot following a trajectory.
    
    The obstacle is a circle centered at the robot's position along the
    trajectory, with radius equal to the robot's bounding radius.
    
    This enables other robots to plan paths that cross this trajectory
    spatially, as long as they don't occupy the same space at the same time.
    """
    
    trajectory: 'SwerveTrajectory'
    """The trajectory this robot is following."""
    
    start_time: float
    """Absolute game time when the trajectory starts (seconds)."""
    
    robot_radius: float
    """Bounding radius of the robot (meters)."""
    
    query_robot_radius: float = 0.0
    """Bounding radius of the querying robot (meters). Added to collision check."""
    
    _trajectory_duration: float = None
    """Cached trajectory duration."""
    
    def __post_init__(self):
        """Cache trajectory duration for efficiency."""
        if self._trajectory_duration is None:
            from gamegine.utils.NCIM.Dimensions.temporal import Second
            self._trajectory_duration = self.trajectory.get_travel_time().to(Second)
    
    def contains_point_at_time(
        self, 
        x: float, 
        y: float, 
        t: float
    ) -> bool:
        """Check if point (x, y) collides with this trajectory obstacle at time t.
        
        The obstacle is modeled as a circle at the robot's trajectory position.
        Collision occurs if the distance from (x, y) to the robot's position
        is less than the combined radii of both robots.
        
        :param x: X coordinate in meters
        :param y: Y coordinate in meters
        :param t: Absolute game time in seconds
        :returns: True if collision detected
        """
        from gamegine.utils.NCIM.Dimensions.temporal import Second
        from gamegine.utils.NCIM.Dimensions.spatial import Meter
        
        # Time relative to trajectory start
        relative_t = t - self.start_time
        
        # Before trajectory starts - no collision
        if relative_t < 0:
            return False
        
        # After trajectory ends - robot is stationary at final position
        if relative_t > self._trajectory_duration:
            # Get final position
            state = self.trajectory.get_at_time(self.trajectory.get_travel_time())
        else:
            # Get position at this time
            state = self.trajectory.get_at_time(Second(relative_t))
        
        # Get obstacle position
        obs_x = state.x.to(Meter) if hasattr(state.x, 'to') else float(state.x)
        obs_y = state.y.to(Meter) if hasattr(state.y, 'to') else float(state.y)
        
        # Check distance against combined radii
        combined_radius = self.robot_radius + self.query_robot_radius
        distance_sq = (x - obs_x)**2 + (y - obs_y)**2
        
        return distance_sq < combined_radius**2
    
    def get_bounding_radius(self) -> float:
        """Get the robot's bounding radius.
        
        :returns: Radius in meters
        """
        return self.robot_radius
    
    def get_time_bounds(self) -> Tuple[float, float]:
        """Get the time interval during which this obstacle is active.
        
        :returns: (start_time, end_time) - note end_time can be extended
                  if you want stationary robot to remain an obstacle
        """
        return (self.start_time, self.start_time + self._trajectory_duration)
    
    def with_query_radius(self, query_radius: float) -> 'TrajectoryObstacle':
        """Create a copy with a different query robot radius.
        
        This allows the same trajectory obstacle to be used for different
        sized robots performing collision checks.
        
        :param query_radius: Bounding radius of the robot doing the query (meters)
        :returns: New TrajectoryObstacle with updated query_robot_radius
        """
        return TrajectoryObstacle(
            trajectory=self.trajectory,
            start_time=self.start_time,
            robot_radius=self.robot_radius,
            query_robot_radius=query_radius,
            _trajectory_duration=self._trajectory_duration,
        )


@dataclass
class StationaryCircleObstacle:
    """A stationary circular obstacle at a fixed position.
    
    Useful for representing robots that have completed their trajectories
    and are now stationary at their destination.
    """
    
    x: float
    """X coordinate in meters."""
    
    y: float
    """Y coordinate in meters."""
    
    radius: float
    """Radius of the obstacle in meters."""
    
    active_from: float = 0.0
    """Time from which this obstacle becomes active (seconds)."""
    
    active_until: float = float('inf')
    """Time until which this obstacle remains active (seconds)."""
    
    def contains_point_at_time(
        self, 
        x: float, 
        y: float, 
        t: float
    ) -> bool:
        """Check if point collides with this stationary obstacle at time t.
        
        :param x: X coordinate in meters
        :param y: Y coordinate in meters
        :param t: Time in seconds (used to check if obstacle is active)
        :returns: True if collision detected
        """
        if t < self.active_from or t > self.active_until:
            return False
        
        distance_sq = (x - self.x)**2 + (y - self.y)**2
        return distance_sq < self.radius**2
    
    def get_bounding_radius(self) -> float:
        """Get the obstacle's radius."""
        return self.radius
    
    def get_time_bounds(self) -> Tuple[float, float]:
        """Get the time interval during which this obstacle is active."""
        return (self.active_from, self.active_until)


def create_trajectory_obstacles(
    trajectories: List[Tuple['SwerveTrajectory', float, float]],
    query_robot_radius: float = 0.0,
) -> List[TrajectoryObstacle]:
    """Create TrajectoryObstacle instances from a list of trajectory info.
    
    :param trajectories: List of (trajectory, start_time, robot_radius) tuples
    :param query_robot_radius: Radius of the robot doing collision queries
    :returns: List of TrajectoryObstacle instances
    """
    return [
        TrajectoryObstacle(
            trajectory=traj,
            start_time=start_time,
            robot_radius=robot_radius,
            query_robot_radius=query_robot_radius,
        )
        for traj, start_time, robot_radius in trajectories
    ]
