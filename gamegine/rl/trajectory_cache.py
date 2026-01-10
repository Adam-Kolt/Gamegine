"""
Trajectory cache for RL training optimization.

Pre-computes trajectories between all interactable locations to avoid
expensive trajectory generation during training.
"""
from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple, Any
import hashlib

from gamegine.utils.NCIM.Dimensions.spatial import SpatialMeasurement, Meter
from gamegine.utils.NCIM.Dimensions.angular import AngularMeasurement


@dataclass
class CachedTrajectory:
    """A cached trajectory with its travel time."""
    travel_time: float  # seconds
    # We don't store the full trajectory for fast mode, just the time
    trajectory: Optional[Any] = None  # SwerveTrajectory if caching full trajectories


def _make_position_key(x: SpatialMeasurement, y: SpatialMeasurement, precision: float = 0.1) -> Tuple[float, float]:
    """Convert position to a hashable key with given precision (meters)."""
    x_m = x.to(Meter) if hasattr(x, 'to') else float(x)
    y_m = y.to(Meter) if hasattr(y, 'to') else float(y)
    # Round to precision
    return (round(x_m / precision) * precision, round(y_m / precision) * precision)


class TrajectoryCache:
    """
    Caches trajectories between positions and interactables.
    
    In fast_mode, only stores travel times (no actual trajectories).
    In full mode, stores complete trajectories for replay.
    """
    
    def __init__(self, fast_mode: bool = True, precision: float = 0.1):
        """
        Initialize the trajectory cache.
        
        :param fast_mode: If True, only cache travel times (not full trajectories).
        :param precision: Position rounding precision in meters (default 0.1m = 10cm).
        """
        self.fast_mode = fast_mode
        self.precision = precision
        self._cache: Dict[Tuple, CachedTrajectory] = {}
        self._hits = 0
        self._misses = 0
    
    def _make_key(
        self,
        start_x: SpatialMeasurement,
        start_y: SpatialMeasurement,
        interactable_name: str,
        interaction_name: str,
    ) -> Tuple:
        """Create a hashable cache key."""
        start_key = _make_position_key(start_x, start_y, self.precision)
        return (start_key[0], start_key[1], interactable_name, interaction_name)
    
    def get(
        self,
        start_x: SpatialMeasurement,
        start_y: SpatialMeasurement,
        interactable_name: str,
        interaction_name: str,
    ) -> Optional[CachedTrajectory]:
        """
        Get a cached trajectory if available.
        
        :return: CachedTrajectory if found, None otherwise.
        """
        key = self._make_key(start_x, start_y, interactable_name, interaction_name)
        result = self._cache.get(key)
        if result is not None:
            self._hits += 1
        else:
            self._misses += 1
        return result
    
    def put(
        self,
        start_x: SpatialMeasurement,
        start_y: SpatialMeasurement,
        interactable_name: str,
        interaction_name: str,
        travel_time: float,
        trajectory: Optional[Any] = None,
    ) -> None:
        """
        Store a trajectory in the cache.
        
        :param travel_time: Time to traverse in seconds.
        :param trajectory: Full trajectory (optional, only stored if not fast_mode).
        """
        key = self._make_key(start_x, start_y, interactable_name, interaction_name)
        self._cache[key] = CachedTrajectory(
            travel_time=travel_time,
            trajectory=trajectory if not self.fast_mode else None,
        )
    
    def precompute(
        self,
        server,
        robot_configs: List,
        interactables: List[Tuple[str, str]],
    ) -> int:
        """
        Pre-compute trajectories for all start positions and interactables.
        
        :param server: DiscreteGameServer instance.
        :param robot_configs: List of RobotConfig with start positions.
        :param interactables: List of (interactable_name, interaction_name) tuples.
        :return: Number of trajectories cached.
        """
        count = 0
        for robot_cfg in robot_configs:
            robot_name = robot_cfg.name
            start_x = robot_cfg.start_state.x if hasattr(robot_cfg.start_state, 'x') else robot_cfg.start_state.get('x', 0)
            start_y = robot_cfg.start_state.y if hasattr(robot_cfg.start_state, 'y') else robot_cfg.start_state.get('y', 0)
            
            for interactable_name, interaction_name in interactables:
                try:
                    nav_point = server.match.get_navigation_point(
                        interactable_name, interaction_name, robot_name
                    )
                    if nav_point is None:
                        continue
                    
                    # Compute trajectory (this is the expensive part)
                    result = server.drive_and_process_action(
                        interactable_name, interaction_name, robot_name, None
                    )
                    
                    if isinstance(result, tuple) and len(result) > 1 and result[1] is not None:
                        trajectory = result[1]
                        travel_time = trajectory.get_travel_time().to(1) if hasattr(trajectory.get_travel_time(), 'to') else float(trajectory.get_travel_time())
                        self.put(start_x, start_y, interactable_name, interaction_name, travel_time, trajectory)
                        count += 1
                except Exception:
                    pass  # Skip invalid combinations
        
        return count
    
    def get_stats(self) -> Dict[str, int]:
        """Get cache statistics."""
        return {
            "size": len(self._cache),
            "hits": self._hits,
            "misses": self._misses,
            "hit_rate": self._hits / max(1, self._hits + self._misses),
        }
    
    def clear(self) -> None:
        """Clear all cached trajectories."""
        self._cache.clear()
        self._hits = 0
        self._misses = 0
