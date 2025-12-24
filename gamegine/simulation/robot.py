import math
from typing import Dict

# Rendering is handled by gamegine.render.handlers, not embedded here
from gamegine.representation.gamepiece import Gamepiece
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from gamegine.representation.interactable import RobotInteractionConfig
from gamegine.simulation.state import StateSpace, ValueEntry
from gamegine.utils.NCIM.Dimensions.angular import AngularMeasurement, Radian
from gamegine.utils.NCIM.Dimensions.spatial import Meter, SpatialMeasurement


from gamegine.first.alliance import Alliance


class RobotState(StateSpace):
    """Class for representing the state space of a robot, which includes the x and y coordinates of the robot on the field screen, the heading of the robot, the alliance of the robot, and the gamepieces the robot is holding.

    :param x: The x-coordinate of the robot.
    :type x: :class:`SpatialMeasurement`
    :param y: The y-coordinate of the robot.
    :type y: :class:`SpatialMeasurement`
    :param heading: The heading of the robot.
    :type heading: :class:`AngularMeasurement`
    :param alliance: The alliance of the robot.
    :type alliance: :class:`Alliance`
    :param gamepieces: A dictionary of gamepieces the robot is holding.
    :type gamepieces: Dict[:class:`Gamepiece`, int]
    """

    def __init__(
        self,
        x: SpatialMeasurement = Meter(0),
        y: SpatialMeasurement = Meter(0),
        heading: AngularMeasurement = Radian(0),
        alliance: Alliance = Alliance.RED,
        gamepieces: Dict[Gamepiece, int] = {},
        current_action: 'RobotInteractionConfig' = None,
        action_progress: float = 0,
    ):
        super().__init__()
        self.setValue("x", x)
        self.setValue("y", y)
        self.setValue("heading", heading)
        self.setValue("gamepieces", gamepieces)
        self.setValue("alliance", alliance)
        self.setValue("current_action", current_action)
        self.setValue("action_progress", action_progress)

    @property
    def x(self) -> ValueEntry[SpatialMeasurement]:
        return self.getValue("x")

    @property
    def y(self) -> ValueEntry[SpatialMeasurement]:
        return self.getValue("y")

    @property
    def heading(self) -> ValueEntry[AngularMeasurement]:
        return self.getValue("heading")

    @property
    def alliance(self) -> ValueEntry[Alliance]:
        return self.getValue("alliance")

    @property
    def gamepieces(self) -> ValueEntry[Dict[Gamepiece, int]]:
        return self.getValue("gamepieces")


    def distance_to(
        self, x: SpatialMeasurement, y: SpatialMeasurement
    ) -> SpatialMeasurement:
        dx = self.x.get() - x
        dy = self.y.get() - y
        return (dx**2 + dy**2) ** 0.5
