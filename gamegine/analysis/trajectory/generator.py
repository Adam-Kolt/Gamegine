from abc import ABC, abstractmethod
from typing import Any, Tuple, List

from gamegine.analysis.trajectory.lib.TrajGen import SwerveTrajectory
from gamegine.analysis import pathfinding
from gamegine.representation.robot import SwerveRobot
from gamegine.utils.NCIM.Dimensions.spatial import SpatialMeasurement, Meter
from gamegine.utils.NCIM.Dimensions.angular import AngularMeasurement
from gamegine.utils.NCIM.ComplexDimensions.velocity import Velocity, MetersPerSecond
from gamegine.utils.NCIM.ComplexDimensions.acceleration import Acceleration, MeterPerSecondSquared

# Gravity constant for friction calculations
GRAVITY_M_S2 = 9.81


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


class SplineTrajectoryGenerator(OfflineTrajectoryGenerator):
    """Offline trajectory generator that uses cubic splines directly.
    
    This generator bypasses the complex optimization solver by:
    1. Fitting a smooth cubic spline through the A* path points
    2. Computing a time-optimal velocity profile with curvature limiting
    3. Generating kinematically-consistent trajectory states
    
    Centripetal acceleration is limited by wheel friction: a_centripetal <= µ * g
    
    Benefits:
    - No solver failures (factorization, line search, etc.)
    - Much faster computation
    - Inherits obstacle avoidance from A* path
    """
    
    def __init__(
        self,
        max_velocity: Velocity = MetersPerSecond(4.0),
        max_acceleration: Acceleration = MeterPerSecondSquared(3.0),
        min_curvature_radius: SpatialMeasurement = Meter(0.3),
        resolution: SpatialMeasurement = Meter(0.15),
    ):
        """
        :param max_velocity: Maximum linear velocity
        :param max_acceleration: Maximum linear acceleration (for tangential motion)
        :param min_curvature_radius: Minimum turning radius
        :param resolution: Distance between output trajectory points
        """
        self.max_velocity = max_velocity
        self.max_acceleration = max_acceleration
        self.min_curvature_radius = min_curvature_radius
        self.resolution = resolution
    
    def _validate_and_subdivide_spline(
        self,
        x_arr: 'np.ndarray',
        y_arr: 'np.ndarray', 
        t_arr: 'np.ndarray',
        expanded_obstacles,
        resolution_m: float,
        max_iterations: int = 10,
    ) -> 'Tuple[np.ndarray, np.ndarray, np.ndarray]':
        """Iteratively validate and subdivide spline until it avoids all obstacles.
        
        Algorithm:
        1. Fit a cubic spline through current knots
        2. Evaluate spline at dense intervals
        3. Check each point for obstacle collision
        4. If collision found, insert midpoint of the problematic segment and retry
        5. Repeat until no collisions or max_iterations reached
        
        :param x_arr: X coordinates of path points
        :param y_arr: Y coordinates of path points
        :param t_arr: Parameter values for each point (normalized arc length)
        :param expanded_obstacles: ExpandedObjectBounds to check collisions against
        :param resolution_m: Resolution for dense evaluation (meters)
        :param max_iterations: Maximum subdivision iterations
        :return: Tuple of (x_knots, y_knots, t_knots) for the validated spline
        """
        from scipy.interpolate import CubicSpline
        import numpy as np
        from gamegine.utils.NCIM.Dimensions.spatial import Meter
        
        x_knots = x_arr.copy()
        y_knots = y_arr.copy()
        t_knots = t_arr.copy()
        
        for iteration in range(max_iterations):
            # Fit spline with current knots
            if len(t_knots) < 2:
                break
            
            cs_x = CubicSpline(t_knots, x_knots, bc_type='natural')
            cs_y = CubicSpline(t_knots, y_knots, bc_type='natural')
            
            # Dense evaluation
            total_length = np.sum(np.sqrt(np.diff(x_knots)**2 + np.diff(y_knots)**2))
            n_eval = max(int(total_length / resolution_m) + 1, 10)
            t_dense = np.linspace(0, 1, n_eval)
            
            eval_x = cs_x(t_dense)
            eval_y = cs_y(t_dense)
            
            # Check for collisions
            collision_indices = []
            for i, (x, y) in enumerate(zip(eval_x, eval_y)):
                point = (Meter(float(x)), Meter(float(y)))
                # Check if point is inside any obstacle
                for obstacle in expanded_obstacles:
                    if obstacle.contains_point(*point):
                        collision_indices.append(i)
                        break
            
            if not collision_indices:
                # No collisions, we're done
                print(f"[SplineTrajectoryGenerator] Validated after {iteration} subdivision(s). {len(t_knots)} knots.")
                break
            
            # Find which segments need subdivision
            # Map collision indices back to knot segments
            segments_to_subdivide = set()
            for coll_idx in collision_indices:
                t_coll = t_dense[coll_idx]
                # Find which knot segment this falls into
                for seg_idx in range(len(t_knots) - 1):
                    if t_knots[seg_idx] <= t_coll <= t_knots[seg_idx + 1]:
                        segments_to_subdivide.add(seg_idx)
                        break
            
            if not segments_to_subdivide:
                break
            
            # Insert midpoints for colliding segments (in reverse order to preserve indices)
            new_x = list(x_knots)
            new_y = list(y_knots)
            new_t = list(t_knots)
            
            for seg_idx in sorted(segments_to_subdivide, reverse=True):
                mid_t = (t_knots[seg_idx] + t_knots[seg_idx + 1]) / 2
                mid_x = (x_knots[seg_idx] + x_knots[seg_idx + 1]) / 2
                mid_y = (y_knots[seg_idx] + y_knots[seg_idx + 1]) / 2
                
                new_t.insert(seg_idx + 1, mid_t)
                new_x.insert(seg_idx + 1, mid_x)
                new_y.insert(seg_idx + 1, mid_y)
            
            x_knots = np.array(new_x)
            y_knots = np.array(new_y)
            t_knots = np.array(new_t)
            
            print(f"[SplineTrajectoryGenerator] Iteration {iteration + 1}: Added {len(segments_to_subdivide)} midpoint(s). Now {len(t_knots)} knots.")
        
        return x_knots, y_knots, t_knots
    
    def generate(
        self,
        robot_name: str,
        robot: SwerveRobot,
        start_state: Tuple[SpatialMeasurement, SpatialMeasurement, AngularMeasurement],
        target_state: Tuple[SpatialMeasurement, SpatialMeasurement, AngularMeasurement],
        path: pathfinding.Path,
        traversal_space: Any,
        constraints: Any = None,
        no_safety_corridor: bool = False,
        robot_constraints: Any = None,
    ) -> SwerveTrajectory:
        """Generates a trajectory using cubic spline interpolation.
        
        :param robot_constraints: Optional SwerveRobotConstraints. If provided, enables
                                  full SwerveTrajectory rendering and friction-based limits.
        """
        from scipy.interpolate import CubicSpline
        import numpy as np
        from gamegine.analysis.trajectory.lib.trajectoryStates import TrajectoryState
        from gamegine.analysis.trajectory.lib.TrajGen import Trajectory, SwerveTrajectory, SwerveRobotConstraints
        from gamegine.utils.NCIM.Dimensions.spatial import Meter
        from gamegine.utils.NCIM.Dimensions.temporal import Second
        from gamegine.utils.NCIM.Dimensions.angular import Radian
        from gamegine.utils.NCIM.ComplexDimensions.velocity import VelocityUnit
        from gamegine.utils.NCIM.ComplexDimensions.acceleration import AccelerationUnit
        from gamegine.utils.NCIM.ComplexDimensions.omega import RadiansPerSecond
        from gamegine.utils.NCIM.ComplexDimensions.alpha import RadiansPerSecondSquared
        
        # Convert units to base SI for computation
        max_vel_mps = self.max_velocity.to(MetersPerSecond)
        max_accel_mps2 = self.max_acceleration.to(MeterPerSecondSquared)
        min_radius_m = self.min_curvature_radius.to(Meter)
        resolution_m = self.resolution.to(Meter)
        
        # Compute centripetal acceleration limit from friction
        # a_centripetal <= µ * g
        # Default to a conservative friction coefficient if not available
        friction_coeff = 1.0  # Default
        if robot_constraints is not None and hasattr(robot_constraints, 'swerve_config'):
            swerve_config = robot_constraints.swerve_config
            if swerve_config is not None and hasattr(swerve_config, 'module'):
                module = swerve_config.module
                if module is not None and hasattr(module, 'wheel'):
                    friction_coeff = module.wheel.grip()
        
        # Centripetal acceleration limit from friction: a_c = µ * g
        centripetal_limit = friction_coeff * GRAVITY_M_S2
        
        # Get path points and convert to numpy arrays
        path_points = path.get_points()
        if len(path_points) < 2:
            raise ValueError("Path must have at least 2 points")
        
        x_arr = np.array([p[0].to(Meter) for p in path_points])
        y_arr = np.array([p[1].to(Meter) for p in path_points])
        n_input = len(x_arr)
        
        # Compute arc length parameterization
        diffs = np.sqrt(np.diff(x_arr)**2 + np.diff(y_arr)**2)
        arc_lengths = np.concatenate([[0], np.cumsum(diffs)])
        total_length = arc_lengths[-1]
        
        if total_length < 0.01:
            raise ValueError("Path is too short for trajectory generation")
        
        # Normalize to [0, 1] for spline parameter
        t_input = arc_lengths / total_length
        
        # Use ALL path points as knots (no downsampling) to prevent corner-cutting
        # If expanded_obstacles is provided, validate and subdivide as needed
        if traversal_space is not None:
            # Extract expanded_obstacles from traversal_space or use directly
            expanded_obstacles = traversal_space
            x_knots, y_knots, t_knots = self._validate_and_subdivide_spline(
                x_arr, y_arr, t_input, expanded_obstacles, resolution_m, max_iterations=10
            )
        else:
            x_knots = x_arr
            y_knots = y_arr
            t_knots = t_input
        
        # Fit cubic splines using all knots
        cs_x = CubicSpline(t_knots, x_knots, bc_type='natural')
        cs_y = CubicSpline(t_knots, y_knots, bc_type='natural')
        
        # Generate dense output points at desired resolution
        n_output = max(int(total_length / resolution_m) + 1, 2)
        t_dense = np.linspace(0, 1, n_output)
        
        # Evaluate spline at dense points
        smooth_x = cs_x(t_dense)
        smooth_y = cs_y(t_dense)
        
        # Compute derivatives
        dx_dt = cs_x(t_dense, 1)
        dy_dt = cs_y(t_dense, 1)
        d2x_dt2 = cs_x(t_dense, 2)
        d2y_dt2 = cs_y(t_dense, 2)
        
        # Compute speed (magnitude of tangent) and curvature
        speed_sq = dx_dt**2 + dy_dt**2
        speed = np.sqrt(speed_sq)
        speed_cubed = speed_sq * speed
        speed_cubed = np.maximum(speed_cubed, 1e-6)
        
        curvature = np.abs(dx_dt * d2y_dt2 - dy_dt * d2x_dt2) / speed_cubed
        
        # Compute segment lengths in output space
        segment_lengths = np.sqrt(np.diff(smooth_x)**2 + np.diff(smooth_y)**2)
        
        # Compute curvature-limited max velocity
        # v_max = sqrt(a_centripetal / curvature) from a_c = v^2 / r = v^2 * curvature
        # Use a small epsilon to prevent division by zero, but don't artificially inflate curvature
        # which would slow down straight sections
        curvature_safe = np.maximum(curvature, 1e-6)  # Small epsilon to prevent div/0
        v_curvature_limit = np.sqrt(centripetal_limit / curvature_safe)
        # Clamp to max velocity (this handles near-zero curvature giving huge v_limit)
        v_curvature_limit = np.minimum(v_curvature_limit, max_vel_mps)
        v_curvature_limit[0] = 0  # Start at rest
        v_curvature_limit[-1] = 0  # End at rest
        
        # Forward pass: respect tangential acceleration limits
        forward_vel = np.zeros(n_output)
        for i in range(1, n_output):
            v_prev = forward_vel[i-1]
            ds = segment_lengths[i-1]
            # v^2 = v0^2 + 2*a*s
            v_max_from_accel = np.sqrt(v_prev**2 + 2 * max_accel_mps2 * ds)
            forward_vel[i] = min(v_curvature_limit[i], v_max_from_accel)
        
        # Backward pass: respect deceleration limits
        backward_vel = np.zeros(n_output)
        backward_vel[-1] = 0
        for i in range(n_output - 2, -1, -1):
            v_next = backward_vel[i+1]
            ds = segment_lengths[i]
            v_max_from_decel = np.sqrt(v_next**2 + 2 * max_accel_mps2 * ds)
            backward_vel[i] = min(forward_vel[i], v_max_from_decel)
        
        velocities = backward_vel
        
        # Compute velocity vectors (tangent direction × speed)
        tangent_x = dx_dt / np.maximum(speed, 1e-6)
        tangent_y = dy_dt / np.maximum(speed, 1e-6)
        vel_x = velocities * tangent_x
        vel_y = velocities * tangent_y
        
        # Compute time steps
        dt_values = np.zeros(n_output - 1)
        for i in range(n_output - 1):
            ds = segment_lengths[i]
            avg_v = (velocities[i] + velocities[i+1]) / 2
            if avg_v > 0.01:
                dt_values[i] = ds / avg_v
            else:
                dt_values[i] = ds / 0.5
            dt_values[i] = np.clip(dt_values[i], 0.01, 2.0)
        
        # Compute accelerations
        accel_x = np.zeros(n_output - 1)
        accel_y = np.zeros(n_output - 1)
        for i in range(n_output - 1):
            dt = dt_values[i]
            if dt > 0.001:
                accel_x[i] = (vel_x[i+1] - vel_x[i]) / dt
                accel_y[i] = (vel_y[i+1] - vel_y[i]) / dt
        
        # Compute heading (theta) - interpolate between start and end
        start_theta = start_state[2].to(Radian)
        end_theta = target_state[2].to(Radian)
        theta_values = np.linspace(start_theta, end_theta, n_output)
        
        # Build unit constructors
        VEL_UNIT = VelocityUnit(Meter, Second)
        ACCEL_UNIT = AccelerationUnit(Meter, Second)
        
        # Create trajectory states
        states = []
        for i in range(n_output - 1):
            states.append(TrajectoryState(
                x=Meter(float(smooth_x[i])),
                y=Meter(float(smooth_y[i])),
                theta=Radian(float(theta_values[i])),
                vel_x=VEL_UNIT(float(vel_x[i])),
                vel_y=VEL_UNIT(float(vel_y[i])),
                acc_x=ACCEL_UNIT(float(accel_x[i])),
                acc_y=ACCEL_UNIT(float(accel_y[i])),
                omega=RadiansPerSecond(0),
                alpha=RadiansPerSecondSquared(0),
                dt=Second(float(dt_values[i])),
            ))
        
        # Add final state
        states.append(TrajectoryState(
            x=Meter(float(smooth_x[-1])),
            y=Meter(float(smooth_y[-1])),
            theta=Radian(float(theta_values[-1])),
            vel_x=VEL_UNIT(0),
            vel_y=VEL_UNIT(0),
            acc_x=None,
            acc_y=None,
            omega=RadiansPerSecond(0),
            alpha=None,
            dt=None,
        ))
        
        # Use provided robot_constraints or create basic ones
        if robot_constraints is not None:
            final_constraints = robot_constraints
        else:
            from gamegine.analysis.trajectory.lib.TrajGen import TrajectoryRobotConstraints
            final_constraints = TrajectoryRobotConstraints(
                max_acceleration=self.max_acceleration,
                max_velocity=self.max_velocity,
            )
        
        return SwerveTrajectory(states, final_constraints)
