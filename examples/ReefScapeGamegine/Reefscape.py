from dataclasses import dataclass
from enum import Enum
from typing import List
from gamegine.representation.game import Game
from gamegine.representation.bounds import (
    Circle,
    Cylinder,
    ExpandedObjectBounds,
    Point,
    Polygon,
    Rectangle,
    SymmetricalX,
    SymmetricalY,
    CircularPattern,
    Transform3D,
    RegularPolygon,
    Square,
)
from gamegine.first.alliance import Alliance

from gamegine.utils.NCIM.ncim import Inch, Degree
from gamegine.representation.obstacle import Obstacle
from pickup import CoralStation
from scoring import Barge, Processor, Reef

Reefscape = Game("FRC Reefscape 2025")

Reefscape.set_field_size(Inch(690.131072), Inch(314.848))
Reefscape.enable_field_border_obstacles()

field_obstacles = [
    Obstacle(
        "Center Reef",
        RegularPolygon((Inch(0), Inch(0)), Inch(37.620150), 6)  # Inch(176.746)
        .get_3d(Inch(0), Inch(80))
        .rotate(Degree(360) / 6 / 2)
        .translate(Inch(176.746), Reefscape.half_field_y()),
    ),
    *SymmetricalY(
        [
            Obstacle(
                "Corner",
                Polygon(
                    [
                        (Inch(0), Inch(0)),
                        (Inch(67.789), Inch(0)),
                        (Inch(0), Inch(48.924)),
                    ]
                ),
            )
        ],
        Reefscape.half_field_y(),
        "Top",
        "Bottom",
    ),
]

field_obstacles = SymmetricalX(field_obstacles, Reefscape.half_field_x(), "Blue", "Red")


class CageLevel(Enum):
    DEEP = 1
    SHALLOW = 2


def cage_obstacle(level: CageLevel, prefix: str):
    if level == CageLevel.DEEP:
        bound = Rectangle.from_center(
            (Inch(0), Inch(0)), Inch(6.749000), Inch(6.749000)
        ).get_3d(Inch(3.250), Inch(80))
    else:
        bound = Rectangle.from_center(
            (Inch(0), Inch(0)), Inch(6.749000), Inch(6.749000)
        ).get_3d(Inch(30.052), Inch(80))

    return Obstacle(
        f"{prefix} {level.name} Cage",
        bound,
    )


def add_cage_obstacles(config: List[CageLevel], alliance: Alliance):
    cages = []
    offset = Inch(0)
    x = Reefscape.half_field_x()
    if alliance == Alliance.RED:
        offset = Inch((175.172 + 162.328) / 2)

    y = Inch(31.178) + offset
    for i, level in enumerate(config):
        cages.append(cage_obstacle(level, f"{i} {alliance.name}").translate_ip(x, y))
        y += Inch(42.937)

    Reefscape.add_obstacles(cages)


field_obstacles.append(
    Obstacle(
        "Center Support",
        Rectangle.from_center(
            Reefscape.half_field_size(),
            Inch(14.000),
            Inch(14.000),
        ).get_3d(Inch(0), Inch(80)),
    )
)


def get_start_position(percentage: float, alliance: Alliance):
    if alliance == Alliance.BLUE:
        return (Inch(297.438046), Reefscape.full_field_y() * percentage)
    else:
        return (Inch(391.438030), Reefscape.full_field_y() * percentage)


class StartingPositionsBlue:
    A = get_start_position(0.25, Alliance.BLUE)
    B = get_start_position(0.5, Alliance.BLUE)
    C = get_start_position(0.75, Alliance.BLUE)


Reefscape.add_obstacles(field_obstacles)


class Names:
    TopCoralStation = "TopPickup"
    BottomCoralStation = "BottomPickup"
    Reef = "Red Reef"
    Processor = "Processor"
    Barge = "Barge"


Reefscape.add_interactable(
    Reef(
        (Inch(176.746), Reefscape.half_field_y()),
        (
            Inch(180.746) - Inch(34),
            Reefscape.half_field_y() - Inch(36.620150),
            Degree(60),
        ),
        "Red Reef",
    )
)

Reefscape.add_interactable(
    CoralStation(
        (Inch(0), Inch(0)),
        (Inch(45), Inch(45), Degree(45)),
        "TopPickup",
    )
)

Reefscape.add_interactable(
    CoralStation(
        (Inch(0), Reefscape.full_field_y()),
        (Inch(40), Reefscape.full_field_y() - Inch(40), Degree(-45)),
        "BottomPickup",
    )
)

Reefscape.add_interactable(
    Processor(
        (Inch(0), Reefscape.half_field_y()),
        (Inch(40), Reefscape.half_field_y(), Degree(0)),
        "Processor",
    )
)

Reefscape.add_interactable(
    Barge(
        (Reefscape.half_field_x(), Reefscape.half_field_y()),
        (Reefscape.half_field_x() - Inch(50), Reefscape.half_field_y(), Degree(0)),
        "Barge",
    )
)
