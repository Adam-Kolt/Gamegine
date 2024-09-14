from gamegine.analysis.trajectory.lib import CALCULATION_UNIT_SPATIAL
from gamegine.analysis.trajectory.lib.constraints.base import (
    __MaxVectorSpacingConstraint,
    __MinVectorSpacingConstraint,
)
from gamegine.utils.NCIM.Dimensions.spatial import SpatialMeasurement


def PositionMinSpacingConstraint(spacing: SpatialMeasurement):
    spacing_num = spacing.to(CALCULATION_UNIT_SPATIAL)

    def __min_spacing_constraint(problem, point_variables):
        __MinVectorSpacingConstraint(
            problem, point_variables.POS_X, point_variables.POS_Y, spacing_num
        )

    return __min_spacing_constraint


def PositionMaxSpacingConstraint(spacing: SpatialMeasurement):
    spacing_num = spacing.to(CALCULATION_UNIT_SPATIAL)

    def __max_spacing_constraint(problem, point_variables):
        __MaxVectorSpacingConstraint(
            problem, point_variables.POS_X, point_variables.POS_Y, spacing_num
        )

    return __max_spacing_constraint
