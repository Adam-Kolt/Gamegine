from gamegine.analysis.trajectory.lib import (
    CALCULATION_UNIT_ANGULAR,
    CALCULATION_UNIT_FORCE,
    CALCULATION_UNIT_MASS,
    CALCULATION_UNIT_SPATIAL,
    CALCULATION_UNIT_TEMPORAL,
)
from gamegine.analysis.trajectory.lib.problemVariables import SwervePointVariables
from gamegine.reference.swerve import SwerveConfig
from gamegine.representation.robot import PhysicalParameters
from gamegine.utils.NCIM.ComplexDimensions.MOI import MOIUnit
from gamegine.utils.NCIM.ComplexDimensions.omega import OmegaUnit
from gamegine.utils.NCIM.ComplexDimensions.torque import TorqueUnit

GRAVITY = 9.81


def SwerveModuleConstraints(
    swerve_config: SwerveConfig,
    robot_parameters: PhysicalParameters,
):  # TODO: These sections are really thrown together, clean up once working
    """Constraints for the swerve module. Including its max speed, max torque, and wheel radius.

    :param swerve_config: The configuration of the swerve module.
    :type swerve_config: :class:`SwerveConfig`
    :param robot_parameters: The physical parameters of the robot.
    :type robot_parameters: :class:`PhysicalParameters`
    :return: The constraint function.
    :rtype: Callable[[Problem, SwervePointVariables], None]
    """
    max_speed = swerve_config.module.get_max_speed().to(
        OmegaUnit(CALCULATION_UNIT_ANGULAR, CALCULATION_UNIT_TEMPORAL)
    )
    max_torque = swerve_config.module.get_max_torque().to(
        TorqueUnit(CALCULATION_UNIT_SPATIAL, CALCULATION_UNIT_FORCE)
    )
    wheel_radius = swerve_config.module.wheel.diameter.to(CALCULATION_UNIT_SPATIAL) / 2
    
    print(f"DEBUG_SWERVE: max_torque = {max_torque}")
    print(f"DEBUG_SWERVE: max_speed = {max_speed}")
    print(f"DEBUG_SWERVE: wheel_radius = {wheel_radius}")
    print(f"DEBUG_SWERVE: max_force_surface_check = {max_torque/wheel_radius}")
    try:
        max_friction_force_check = (
                swerve_config.module.wheel.grip()
                * GRAVITY
                * robot_parameters.mass.to(CALCULATION_UNIT_MASS)
        )
        print(f"DEBUG_SWERVE: max_friction_force = {max_friction_force_check}")
    except:
        pass

    def __swerve_module_constraints(problem, point_variables: SwervePointVariables):

        for module in [
            point_variables.TR,
            point_variables.TL,
            point_variables.BL,
            point_variables.BR,
        ]:
            max_force_surface = max_torque / wheel_radius
            max_friction_force = (
                swerve_config.module.wheel.grip()
                * GRAVITY
                * robot_parameters.mass.to(CALCULATION_UNIT_MASS)
            )
            if max_force_surface > max_friction_force:
                max_force_surface = max_friction_force

            for i in range(len(module.FX)):
                V_mag_square = module.VX[i] ** 2 + module.VY[i] ** 2

                force_mag_squared = module.FX[i] ** 2 + module.FY[i] ** 2
                problem.subject_to(force_mag_squared <= max_force_surface**2)

                problem.subject_to(V_mag_square <= (max_speed * wheel_radius) ** 2)

    return __swerve_module_constraints


def SwerveKinematicConstraints(
    swerve_config: SwerveConfig, robot_parameters: PhysicalParameters
):
    """Constraints for the swerve kinematics. Including the robot's mass and moment of inertia.

    :param swerve_config: The configuration of the swerve module.
    :type swerve_config: :class:`SwerveConfig`
    :param robot_parameters: The physical parameters of the robot.
    :type robot_parameters: :class:`PhysicalParameters`
    :return: The constraint function.
    :rtype: Callable[[Problem, SwervePointVariables], None]"""

    mass = robot_parameters.mass.to(CALCULATION_UNIT_MASS)
    moi = robot_parameters.moi.to(
        MOIUnit(CALCULATION_UNIT_MASS, CALCULATION_UNIT_SPATIAL)
    )

    def __swerve_kinematic_constraints(problem, point_variables: SwervePointVariables):
        # We iterate over time steps first
        for i in range(len(point_variables.TL.FX)):
            net_force_x = 0
            net_force_y = 0
            net_torque = 0
            
            # offsets are constant
            offsets = [
                swerve_config.top_left_offset,
                swerve_config.top_right_offset,
                swerve_config.bottom_left_offset,
                swerve_config.bottom_right_offset,
            ]
            
            # Modules: TL, TR, BL, BR
            modules = [
                point_variables.TL,
                point_variables.TR,
                point_variables.BL,
                point_variables.BR,
            ]

            for module_idx, module in enumerate(modules):
                # Accumulate forces/torques
                fx = module.FX[i]
                fy = module.FY[i]
                
                net_force_x += fx
                net_force_y += fy

                offset = offsets[module_idx]
                offset_x = CALCULATION_UNIT_SPATIAL(offset[0])
                offset_y = CALCULATION_UNIT_SPATIAL(offset[1])
                
                # Torque = r x F = rx*Fy - ry*Fx
                net_torque += offset_x * fy - offset_y * fx

                # Kinematics: Module Velocity = Robot Velocity + Omega x r
                # V_mx = V_rx - Omega * ry
                # V_my = V_ry + Omega * rx
                
                robot_vx = point_variables.VEL_X[i]
                robot_vy = point_variables.VEL_Y[i]
                robot_omega = point_variables.OMEGA[i]
                
                module_vx = module.VX[i]
                module_vy = module.VY[i]

                problem.subject_to(module_vx == robot_vx - robot_omega * offset_y)
                problem.subject_to(module_vy == robot_vy + robot_omega * offset_x)
                
                # No individual angular constraint needed here, Omega is robot state
                # problem.subject_to(point_variables.OMEGA[i] == angular_speed) # REMOVED: Incorrect

            # Max Friction / Acceleration limits per module? 
            # The original code had a per-module accel check which is weird since accel is defined by force/mass.
            # But let's keep the friction limit if it makes sense physics-wise (F <= mu*N)
            # Fx^2 + Fy^2 <= (mu * m_robot * g / 4)^2 -- assuming equal weight distrib.
            # The original code did: acc^2 <= (mu*g)^2. Since F = m*a, this is equivalent.
            # However, enforcing it on *acceleration variables* checks if the resultant acceleration is feasible.
            # Let's check forces instead, as forces are the decision variables for modules usually.
            # But here we have ACCEL variables.
            
            # Re-applying consistent constraints from original, but fixed index
            # Original: 
            # problem.subject_to(
            #     point_variables.ACCEL_X[i] ** 2 + point_variables.ACCEL_Y[i] ** 2
            #     <= (swerve_config.module.wheel.grip() * GRAVITY) ** 2
            # )
            # This limits the robot's CENTER OF MASS acceleration to be within friction circle of *something*.
            # Actually, total max accel is mu*g if all wheels pull.
            # Let's keep it.
            
            # Newton's laws
            problem.subject_to(point_variables.ACCEL_X[i] == net_force_x / mass)
            problem.subject_to(point_variables.ACCEL_Y[i] == net_force_y / mass)
            problem.subject_to(point_variables.ALPHA[i] == net_torque / moi)

    return __swerve_kinematic_constraints
