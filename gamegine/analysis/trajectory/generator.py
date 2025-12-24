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
    
    def _compute_motor_limited_acceleration(
        self,
        linear_velocity_mps: float,
        angular_velocity_rps: float,
        swerve_config,
        robot_mass_kg: float,
        robot_moi_kgm2: float,
        module_radius_m: float,
    ) -> tuple:
        """Compute maximum acceleration at given velocity, accounting for rotation.
        
        When the robot is both translating and rotating, motor torque is split between:
        - Translation force: F_trans = m * a_linear / 4
        - Rotation torque contribution: τ_rot = I * α / (4 * r_module)
        
        The available motor torque is shared, so max linear acceleration is reduced
        when angular velocity/acceleration is demanded.
        
        :param linear_velocity_mps: Current linear velocity in m/s
        :param angular_velocity_rps: Current angular velocity in rad/s
        :param swerve_config: SwerveConfig with module specifications
        :param robot_mass_kg: Robot mass in kg
        :param robot_moi_kgm2: Robot moment of inertia in kg*m^2
        :param module_radius_m: Distance from robot center to module in m
        :return: Tuple of (max_linear_accel_mps2, max_angular_accel_rps2)
        """
        from gamegine.utils.NCIM.ComplexDimensions.omega import RadiansPerSecond
        from gamegine.utils.NCIM.ComplexDimensions.torque import NewtonMeter
        from gamegine.utils.NCIM.Dimensions.spatial import Meter
        import math
        
        module = swerve_config.module
        wheel_radius_m = module.wheel.diameter.to(Meter) / 2.0
        gear_ratio = module.drive_gear_ratio.get_ratio()
        
        if wheel_radius_m <= 0 or module_radius_m <= 0:
            max_accel = self.max_acceleration.to(MeterPerSecondSquared)
            return (max_accel, max_accel / module_radius_m)
        
        # Each module sees a combination of translational and rotational velocity
        # For worst-case (modules at corners), module speed ≈ sqrt(v_linear² + (ω * r_module)²)
        rotational_linear_component = abs(angular_velocity_rps) * module_radius_m
        module_linear_speed = math.sqrt(linear_velocity_mps**2 + rotational_linear_component**2)
        
        # Convert module linear speed to wheel angular velocity
        wheel_omega_rad_s = module_linear_speed / wheel_radius_m
        
        # Convert wheel speed to motor speed (through gearing)
        # Motor runs faster than wheel by gear_ratio
        motor_omega_rad_s = wheel_omega_rad_s * gear_ratio
        
        # Query motor torque at motor-side speed
        motor_omega = RadiansPerSecond(motor_omega_rad_s)
        # Note: get_torque already applies gearing to give wheel-side torque
        available_wheel_torque = module.get_torque(RadiansPerSecond(wheel_omega_rad_s))
        torque_nm = available_wheel_torque.to(NewtonMeter)
        
        # Convert wheel torque to force at ground: F = τ / r_wheel
        force_per_module = torque_nm / wheel_radius_m
        
        # Total force from all 4 modules
        total_force = force_per_module * 4
        
        # Split force between translation and rotation based on current demands
        # Simple model: allocate proportionally to velocity components
        total_velocity = linear_velocity_mps + rotational_linear_component + 0.1  # epsilon
        translation_fraction = (linear_velocity_mps + 0.05) / total_velocity
        rotation_fraction = (rotational_linear_component + 0.05) / total_velocity
        
        # Force available for translation
        trans_force = total_force * translation_fraction
        # Torque available for rotation (force * module_radius)
        rot_force = total_force * rotation_fraction
        rot_torque = rot_force * module_radius_m
        
        # Compute accelerations
        # a_linear = F / m
        if robot_mass_kg > 0:
            max_linear_accel = trans_force / robot_mass_kg
        else:
            max_linear_accel = self.max_acceleration.to(MeterPerSecondSquared)
        
        # α = τ / I
        if robot_moi_kgm2 > 0:
            max_angular_accel = rot_torque / robot_moi_kgm2
        else:
            max_angular_accel = 10.0  # Default rad/s^2
        
        # Clamp to configured max
        max_accel = self.max_acceleration.to(MeterPerSecondSquared)
        return (min(max_linear_accel, max_accel), min(max_angular_accel, max_accel / module_radius_m))
    
    def _compute_module_states(
        self,
        state,
        prev_state,
        swerve_config,
        dt: float,
    ):
        """Compute swerve module states from robot chassis state using inverse kinematics.
        
        For each module, computes:
        - Wheel angle (steer direction to achieve desired velocity)
        - Wheel omega (angular velocity of wheel)
        - Wheel alpha (angular acceleration)
        
        :param state: Current TrajectoryState
        :param prev_state: Previous TrajectoryState (for alpha calculation)
        :param swerve_config: SwerveConfig with module offsets
        :param dt: Time step in seconds
        :return: List of 4 SwerveModuleState objects
        """
        from gamegine.analysis.trajectory.lib.trajectoryStates import SwerveModuleState
        from gamegine.utils.NCIM.Dimensions.angular import Radian
        from gamegine.utils.NCIM.ComplexDimensions.omega import RadiansPerSecond
        from gamegine.utils.NCIM.ComplexDimensions.alpha import RadiansPerSecondSquared
        from gamegine.utils.NCIM.Dimensions.spatial import Meter
        from gamegine.utils.NCIM.ComplexDimensions.velocity import MetersPerSecond
        import math
        
        module_states = []
        offsets = [
            swerve_config.top_left_offset,
            swerve_config.top_right_offset,
            swerve_config.bottom_left_offset,
            swerve_config.bottom_right_offset,
        ]
        
        wheel_radius_m = swerve_config.module.wheel.diameter.to(Meter) / 2.0
        
        # Get chassis velocities
        vx = state.vel_x.to(MetersPerSecond) if hasattr(state.vel_x, 'to') else float(state.vel_x)
        vy = state.vel_y.to(MetersPerSecond) if hasattr(state.vel_y, 'to') else float(state.vel_y)
        omega = state.omega.to(RadiansPerSecond) if hasattr(state.omega, 'to') else float(state.omega)
        
        # Previous wheel omegas for alpha calculation
        prev_wheel_omegas = []
        if prev_state is not None:
            prev_vx = prev_state.vel_x.to(MetersPerSecond) if hasattr(prev_state.vel_x, 'to') else float(prev_state.vel_x)
            prev_vy = prev_state.vel_y.to(MetersPerSecond) if hasattr(prev_state.vel_y, 'to') else float(prev_state.vel_y)
            prev_omega_chassis = prev_state.omega.to(RadiansPerSecond) if hasattr(prev_state.omega, 'to') else float(prev_state.omega)
            
            for offset in offsets:
                rx = offset[0].to(Meter)
                ry = offset[1].to(Meter)
                vx_mod = prev_vx - prev_omega_chassis * ry
                vy_mod = prev_vy + prev_omega_chassis * rx
                linear_speed = math.sqrt(vx_mod**2 + vy_mod**2)
                prev_wheel_omegas.append(linear_speed / wheel_radius_m if wheel_radius_m > 0 else 0.0)
        else:
            prev_wheel_omegas = [0.0, 0.0, 0.0, 0.0]
        
        for i, offset in enumerate(offsets):
            # Module offset from robot center (in meters)
            rx = offset[0].to(Meter)
            ry = offset[1].to(Meter)
            
            # Inverse kinematics: module velocity = chassis velocity + omega × offset
            # v_module = v_chassis + ω × r
            # In 2D: vx_mod = vx - ω*ry, vy_mod = vy + ω*rx
            vx_module = vx - omega * ry
            vy_module = vy + omega * rx
            
            # Wheel angle (steer direction)
            if abs(vx_module) < 1e-6 and abs(vy_module) < 1e-6:
                wheel_angle = 0.0
            else:
                wheel_angle = math.atan2(vy_module, vx_module)
            
            # Wheel speed (linear → angular via wheel radius)
            linear_speed = math.sqrt(vx_module**2 + vy_module**2)
            wheel_omega_val = linear_speed / wheel_radius_m if wheel_radius_m > 0 else 0.0
            
            # Alpha from change in omega
            if dt > 0.001:
                wheel_alpha_val = (wheel_omega_val - prev_wheel_omegas[i]) / dt
            else:
                wheel_alpha_val = 0.0
            
            module_states.append(SwerveModuleState(
                wheel_angle=Radian(wheel_angle),
                wheel_omega=RadiansPerSecond(wheel_omega_val),
                wheel_alpha=RadiansPerSecondSquared(wheel_alpha_val),
            ))
        
        return module_states
    
    def generate(
        self,
        robot_name: str,
        robot: SwerveRobot,
        start_state: Tuple[SpatialMeasurement, SpatialMeasurement, AngularMeasurement],
        target_state: Tuple[SpatialMeasurement, SpatialMeasurement, AngularMeasurement],
        path: pathfinding.Path,
        expanded_obstacles: Any = None,
        constraints: Any = None,
        no_safety_corridor: bool = False,
        robot_constraints: Any = None,  # Deprecated, use robot instead
    ) -> SwerveTrajectory:
        """Generates a trajectory using cubic spline interpolation.
        
        Uses the SwerveRobot's drivetrain, physics (mass, MOI), and wheel friction
        to compute a physically-accurate trajectory with motor curve integration.
        
        :param robot: SwerveRobot with drivetrain and physics. If None, falls back to robot_constraints.
        :param expanded_obstacles: ExpandedObjectBounds for collision validation.
        """
        from scipy.interpolate import CubicSpline
        import numpy as np
        from gamegine.analysis.trajectory.lib.trajectoryStates import TrajectoryState, SwerveTrajectoryState
        from gamegine.analysis.trajectory.lib.TrajGen import Trajectory, SwerveTrajectory, SwerveRobotConstraints
        from gamegine.utils.NCIM.Dimensions.spatial import Meter
        from gamegine.utils.NCIM.Dimensions.mass import Kilogram
        from gamegine.utils.NCIM.Dimensions.temporal import Second
        from gamegine.utils.NCIM.Dimensions.angular import Radian
        from gamegine.utils.NCIM.ComplexDimensions.velocity import VelocityUnit
        from gamegine.utils.NCIM.ComplexDimensions.acceleration import AccelerationUnit
        from gamegine.utils.NCIM.ComplexDimensions.MOI import KilogramMetersSquared
        from gamegine.utils.NCIM.ComplexDimensions.omega import RadiansPerSecond
        from gamegine.utils.NCIM.ComplexDimensions.alpha import RadiansPerSecondSquared
        
        # Convert units to base SI for computation
        max_vel_mps = self.max_velocity.to(MetersPerSecond)
        max_accel_mps2 = self.max_acceleration.to(MeterPerSecondSquared)
        min_radius_m = self.min_curvature_radius.to(Meter)
        resolution_m = self.resolution.to(Meter)
        
        # Extract parameters from SwerveRobot or fall back to robot_constraints
        swerve_config = None
        robot_mass_kg = 50.0  # Default
        robot_moi_kgm2 = 5.0  # Default kg*m^2
        friction_coeff = 1.0  # Default
        use_motor_curve = False
        module_radius_m = 0.4  # Default module distance from center
        
        # Prefer robot object if available
        if robot is not None:
            if hasattr(robot, 'drivetrain') and robot.drivetrain is not None:
                swerve_config = robot.drivetrain
                use_motor_curve = True
                # Get friction from wheel
                if hasattr(swerve_config.module, 'wheel'):
                    friction_coeff = swerve_config.module.wheel.grip()
                # Compute module radius from offsets
                offset = swerve_config.top_left_offset
                module_radius_m = (offset[0].to(Meter)**2 + offset[1].to(Meter)**2)**0.5
            if hasattr(robot, 'physics') and robot.physics is not None:
                robot_mass_kg = robot.physics.mass.to(Kilogram)
                robot_moi_kgm2 = robot.physics.moi.to(KilogramMetersSquared)
        # Fallback to robot_constraints for backward compatibility
        elif robot_constraints is not None:
            if hasattr(robot_constraints, 'swerve_config') and robot_constraints.swerve_config is not None:
                swerve_config = robot_constraints.swerve_config
                use_motor_curve = True
                if hasattr(swerve_config.module, 'wheel'):
                    friction_coeff = swerve_config.module.wheel.grip()
                offset = swerve_config.top_left_offset
                module_radius_m = (offset[0].to(Meter)**2 + offset[1].to(Meter)**2)**0.5
            if hasattr(robot_constraints, 'physical_parameters') and robot_constraints.physical_parameters is not None:
                robot_mass_kg = robot_constraints.physical_parameters.mass.to(Kilogram)
                if hasattr(robot_constraints.physical_parameters, 'moi'):
                    robot_moi_kgm2 = robot_constraints.physical_parameters.moi.to(KilogramMetersSquared)
        
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
        if expanded_obstacles is not None:
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
        # Compute heading change rate for angular velocity consideration
        start_theta = start_state[2].to(Radian)
        end_theta = target_state[2].to(Radian)
        total_heading_change = abs(end_theta - start_theta)
        
        # Ensure start and end at rest
        v_curvature_limit[0] = 0
        v_curvature_limit[-1] = 0
        
        # Forward pass: respect tangential acceleration limits (motor curve aware)
        forward_vel = np.zeros(n_output)
        forward_omega = np.zeros(n_output)  # Angular velocity profile
        
        for i in range(1, n_output):
            v_prev = forward_vel[i-1]
            ds = segment_lengths[i-1]
            
            # Estimate angular velocity based on heading change rate
            # Distribute heading change proportionally to distance traveled
            cumulative_dist = np.sum(segment_lengths[:i])
            progress = cumulative_dist / total_length if total_length > 0 else 0
            omega_estimate = total_heading_change / (sum(segment_lengths) / max(v_prev, 0.5)) if total_heading_change > 0.01 else 0
            forward_omega[i] = omega_estimate
            
            # Use motor curve for acceleration if swerve_config available
            if use_motor_curve and swerve_config is not None:
                accel_limit, _ = self._compute_motor_limited_acceleration(
                    v_prev, omega_estimate, swerve_config, robot_mass_kg, robot_moi_kgm2, module_radius_m
                )
            else:
                accel_limit = max_accel_mps2
            
            # v^2 = v0^2 + 2*a*s
            v_max_from_accel = np.sqrt(v_prev**2 + 2 * accel_limit * ds)
            forward_vel[i] = min(v_curvature_limit[i], v_max_from_accel)
        
        # Backward pass: respect deceleration limits (motor curve aware for regenerative braking)
        backward_vel = np.zeros(n_output)
        backward_vel[-1] = 0
        for i in range(n_output - 2, -1, -1):
            v_next = backward_vel[i+1]
            ds = segment_lengths[i]
            omega_estimate = forward_omega[i]
            
            # Use motor curve for deceleration
            if use_motor_curve and swerve_config is not None:
                decel_limit, _ = self._compute_motor_limited_acceleration(
                    v_next, omega_estimate, swerve_config, robot_mass_kg, robot_moi_kgm2, module_radius_m
                )
            else:
                decel_limit = max_accel_mps2
            
            v_max_from_decel = np.sqrt(v_next**2 + 2 * decel_limit * ds)
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
        
        # Compute omega (angular velocity) from heading change
        omega_values = np.zeros(n_output - 1)
        for i in range(n_output - 1):
            if dt_values[i] > 0.001:
                omega_values[i] = (theta_values[i+1] - theta_values[i]) / dt_values[i]
        
        # Create trajectory states (with module states if swerve_config available)
        states = []
        prev_state = None
        
        for i in range(n_output - 1):
            # Create base state first
            base_state = TrajectoryState(
                x=Meter(float(smooth_x[i])),
                y=Meter(float(smooth_y[i])),
                theta=Radian(float(theta_values[i])),
                vel_x=VEL_UNIT(float(vel_x[i])),
                vel_y=VEL_UNIT(float(vel_y[i])),
                acc_x=ACCEL_UNIT(float(accel_x[i])),
                acc_y=ACCEL_UNIT(float(accel_y[i])),
                omega=RadiansPerSecond(float(omega_values[i])),
                alpha=RadiansPerSecondSquared(0),
                dt=Second(float(dt_values[i])),
            )
            
            # Compute module states if swerve config available
            if use_motor_curve and swerve_config is not None:
                module_states = self._compute_module_states(
                    base_state, prev_state, swerve_config, float(dt_values[i])
                )
                state = SwerveTrajectoryState(
                    x=base_state.x,
                    y=base_state.y,
                    theta=base_state.theta,
                    vel_x=base_state.vel_x,
                    vel_y=base_state.vel_y,
                    acc_x=base_state.acc_x,
                    acc_y=base_state.acc_y,
                    omega=base_state.omega,
                    alpha=base_state.alpha,
                    dt=base_state.dt,
                    module_states=module_states,
                )
            else:
                state = base_state
            
            states.append(state)
            prev_state = base_state
        
        # Add final state
        final_base_state = TrajectoryState(
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
        )
        
        if use_motor_curve and swerve_config is not None:
            module_states = self._compute_module_states(
                final_base_state, prev_state, swerve_config, 0.02
            )
            final_state = SwerveTrajectoryState(
                x=final_base_state.x,
                y=final_base_state.y,
                theta=final_base_state.theta,
                vel_x=final_base_state.vel_x,
                vel_y=final_base_state.vel_y,
                acc_x=final_base_state.acc_x,
                acc_y=final_base_state.acc_y,
                omega=final_base_state.omega,
                alpha=final_base_state.alpha,
                dt=final_base_state.dt,
                module_states=module_states,
            )
        else:
            final_state = final_base_state
        
        states.append(final_state)
        
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
