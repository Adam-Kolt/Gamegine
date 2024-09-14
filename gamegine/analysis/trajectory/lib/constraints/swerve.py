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


def SwerveModuleConstraints(
    swerve_config: SwerveConfig,
):  # TODO: These sections are really thrown together, clean up once working
    max_speed = swerve_config.module.get_max_speed().to(
        OmegaUnit(CALCULATION_UNIT_ANGULAR, CALCULATION_UNIT_TEMPORAL)
    )
    max_torque = swerve_config.module.get_max_torque().to(
        TorqueUnit(CALCULATION_UNIT_SPATIAL, CALCULATION_UNIT_FORCE)
    )
    wheel_radius = swerve_config.module.wheel.diameter.to(CALCULATION_UNIT_SPATIAL) / 2

    def __swerve_module_constraints(problem, point_variables: SwervePointVariables):

        for module in [
            point_variables.TR,
            point_variables.TL,
            point_variables.BL,
            point_variables.BR,
        ]:

            for i in range(len(module.FX)):
                V_mag_square = module.VX[i] ** 2 + module.VY[i] ** 2
                max_force_surface = max_torque / wheel_radius
                force_mag_squared = module.FX[i] ** 2 + module.FY[i] ** 2
                problem.subject_to(force_mag_squared <= max_force_surface**2)

                problem.subject_to(V_mag_square <= (max_speed * wheel_radius) ** 2)

    return __swerve_module_constraints


def SwerveKinematicConstraints(
    swerve_config: SwerveConfig, robot_parameters: PhysicalParameters
):
    mass = robot_parameters.mass.to(CALCULATION_UNIT_MASS)
    moi = robot_parameters.moi.to(
        MOIUnit(CALCULATION_UNIT_MASS, CALCULATION_UNIT_SPATIAL)
    )

    def __swerve_kinematic_constraints(problem, point_variables: SwervePointVariables):
        for i in range(len(point_variables.TL.FX)):
            net_force_x = 0
            net_force_y = 0
            net_torque = 0

            net_speed_x = 0
            net_speed_y = 0
            net_angular_speed = 0

            offsets = [
                swerve_config.top_left_offset,
                swerve_config.top_right_offset,
                swerve_config.bottom_left_offset,
                swerve_config.bottom_right_offset,
            ]
            for i, module in enumerate(
                [
                    point_variables.TL,
                    point_variables.TR,
                    point_variables.BL,
                    point_variables.BR,
                ]
            ):
                net_force_x += module.FX[i]
                net_force_y += module.FY[i]

                net_speed_x += module.VX[i]
                net_speed_y += module.VY[i]

                offset = offsets[i]
                offset_x = CALCULATION_UNIT_SPATIAL(offset[0])
                offset_y = CALCULATION_UNIT_SPATIAL(offset[1])
                # Torque is the cross product of the force and the offset
                net_torque += module.FY[i] * offset_x - module.FX[i] * offset_y
                net_angular_speed += module.VY[i] * offset_x - module.VX[i] * offset_y

            problem.subject_to(point_variables.ACCEL_X[i] == net_force_x / mass)

            problem.subject_to(point_variables.ACCEL_Y[i] == net_force_y / mass)

            problem.subject_to(point_variables.ALPHA[i] == net_torque / moi)

            problem.subject_to(point_variables.VEL_X[i] == net_speed_x)
            problem.subject_to(point_variables.VEL_Y[i] == net_speed_y)

            problem.subject_to(point_variables.OMEGA[i] == net_angular_speed)

    return __swerve_kinematic_constraints
