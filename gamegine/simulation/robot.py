import math
from typing import Dict

import pygame
from gamegine.first.alliance import Alliance
from gamegine.render.drawable import Drawable
from gamegine.representation.gamepiece import Gamepiece
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from gamegine.representation.interactable import RobotInteractionConfig
from gamegine.simulation.state import StateSpace, ValueEntry
from gamegine.utils.NCIM.Dimensions.angular import AngularMeasurement, Radian
from gamegine.utils.NCIM.Dimensions.spatial import Meter, SpatialMeasurement


class RobotState(StateSpace, Drawable):
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

    def draw(self, render_scale: SpatialMeasurement):
        # Rotated rectangle with heading line
        pygame.draw.rect(
            pygame.display.get_surface(),
            (255, 255, 255),
            (
                self.x.get() / render_scale,
                self.y.get() / render_scale,
                10,
                10,
            ),
        )
        pygame.draw.line(
            pygame.display.get_surface(),
            (0, 0, 0),
            (
                self.x.get() / render_scale + 5,
                self.y.get() / render_scale + 5,
            ),
            (
                self.x.get() / render_scale + 5 + 5 * self.heading.get().cos(),
                self.y.get() / render_scale + 5 + 5 * self.heading.get().sin(),
            ),
        )

    def draw_real(self, render_scale: SpatialMeasurement, robot):
        surface = pygame.display.get_surface()
        radius = robot.true_radius() / render_scale
        center_x = self.x.get() / render_scale
        center_y = self.y.get() / render_scale

        diagonal = 2 * radius
        half_side = diagonal / math.sqrt(2) / 2
        angle_offset = Radian(math.pi / 4)

        points = []
        for i in range(4):
            angle: AngularMeasurement = (
                self.heading.get() + angle_offset + Radian(math.pi / 2) * i
            )
            x = center_x + half_side * math.cos(angle.to(Radian))
            y = center_y + half_side * math.sin(angle.to(Radian))
            points.append((x, y))
        width = int(Meter(0.05) / render_scale)
        pygame.draw.polygon(surface, (0, 0, 255), points, width)

        heading_x = center_x + radius / 2 * math.cos(self.heading.get())
        heading_y = center_y + radius / 2 * math.sin(self.heading.get())
        pygame.draw.line(
            surface, (0, 0, 255), (center_x, center_y), (heading_x, heading_y), width
        )

        # Draw gamepieces
        for gamepiece, count in self.gamepieces.get().items():
            if count > 0:
                gamepiece.display(self.x.get(), self.y.get(), render_scale)

    def distance_to(
        self, x: SpatialMeasurement, y: SpatialMeasurement
    ) -> SpatialMeasurement:
        dx = self.x.get() - x
        dy = self.y.get() - y
        return (dx**2 + dy**2) ** 0.5
