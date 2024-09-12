from gamegine.analysis.trajectory.lib import (
    CALCULATION_UNIT_ANGULAR,
    CALCULATION_UNIT_SPATIAL,
    CALCULATION_UNIT_TEMPORAL,
)
from gamegine.analysis.trajectory.lib.constraints.base import (
    __VectorMagnitudeEqualityConstraint,
    get_magnitude_squared,
)
from gamegine.utils.NCIM.ComplexDimensions.acceleration import (
    Acceleration,
    AccelerationUnit,
)
from gamegine.utils.NCIM.ComplexDimensions.alpha import Alpha, AlphaUnit
from gamegine.utils.NCIM.ComplexDimensions.omega import Omega, OmegaUnit
from gamegine.utils.NCIM.ComplexDimensions.velocity import Velocity, VelocityUnit
from gamegine.utils.NCIM.Dimensions.spatial import SpatialMeasurement


def VelocityEquals(velocity: Velocity):
    vel = velocity.to(VelocityUnit(CALCULATION_UNIT_SPATIAL, CALCULATION_UNIT_TEMPORAL))

    def _velocity_equals(problem, point_variables, index: int = -1):
        __VectorMagnitudeEqualityConstraint(
            problem, point_variables.VEL_X, point_variables.VEL_Y, vel, index
        )

    return _velocity_equals


def VelocityLessThan(velocity: Velocity):
    vel = velocity.to(VelocityUnit(CALCULATION_UNIT_SPATIAL, CALCULATION_UNIT_TEMPORAL))

    def _velocity_less_than(problem, point_variables, index: int = -1):
        indices = range(len(point_variables.VEL_X)) if index == -1 else [index]

        for i in indices:
            problem.subject_to(
                get_magnitude_squared(
                    point_variables.VEL_X[i], point_variables.VEL_Y[i]
                )
                <= vel**2
            )

    return _velocity_less_than


def PositionEquals(x: SpatialMeasurement, y: SpatialMeasurement):
    def _position_equals(problem, point_variables, index: int):
        x = x.to(CALCULATION_UNIT_SPATIAL)
        y = y.to(CALCULATION_UNIT_SPATIAL)

        __VectorMagnitudeEqualityConstraint(
            problem, point_variables.POS_X, point_variables.POS_Y, x, index
        )

    return _position_equals


def AccelerationLessThan(acceleration: Acceleration):
    acc = acceleration.to(
        AccelerationUnit(CALCULATION_UNIT_SPATIAL, CALCULATION_UNIT_TEMPORAL)
    )

    def _acceleration_less_than(problem, point_variables, index: int = -1):
        indices = range(len(point_variables.ACCEL_X)) if index == -1 else [index]

        for i in indices:
            problem.subject_to(
                get_magnitude_squared(
                    point_variables.ACCEL_X[i], point_variables.ACCEL_Y[i]
                )
                <= acc**2
            )

    return _acceleration_less_than


def AngularVelocityLessThan(angular_velocity: Omega):
    omega = angular_velocity.to(
        OmegaUnit(CALCULATION_UNIT_ANGULAR, CALCULATION_UNIT_TEMPORAL)
    )

    def _angular_velocity_less_than(problem, point_variables, index: int = -1):
        indices = range(len(point_variables.OMEGA)) if index == -1 else [index]

        for i in indices:
            problem.subject_to(point_variables.OMEGA[i] <= omega)

    return _angular_velocity_less_than


def AngularAccelerationLessThan(angular_acceleration: Alpha):
    alpha = angular_acceleration.to(
        AlphaUnit(CALCULATION_UNIT_ANGULAR, CALCULATION_UNIT_TEMPORAL)
    )

    def _angular_acceleration_less_than(problem, point_variables, index: int = -1):
        indices = range(len(point_variables.ALPHA)) if index == -1 else [index]

        for i in indices:
            problem.subject_to(point_variables.ALPHA[i] <= alpha)

    return _angular_acceleration_less_than
