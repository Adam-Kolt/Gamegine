"""Rebuilt Robot Configuration
Shared robot definition for the Rebuilt game.
"""

from gamegine.representation.robot import SwerveRobot, PhysicalParameters
from gamegine.representation.bounds import Rectangle
from gamegine.reference import gearing, motors
from gamegine.reference.swerve import SwerveConfig, SwerveModule
from gamegine.utils.NCIM.Dimensions.spatial import Meter, Feet, Inch
from gamegine.utils.NCIM.Dimensions.mass import Pound
from gamegine.utils.NCIM.ComplexDimensions.acceleration import MeterPerSecondSquared
from gamegine.utils.NCIM.ncim import Ampere
from gamegine.first.alliance import Alliance

ROBOT_WIDTH = Inch(30)

def create_swerve_config():
    """Create standard swerve drivetrain config."""
    return SwerveConfig(
        SwerveModule(
            motors.MotorConfig(
                motors.KrakenX60,
                motors.PowerConfig(Ampere(60), Ampere(240), 1.0),
            ),
            gearing.MK4I.L3,
            motors.MotorConfig(
                motors.KrakenX60,
                motors.PowerConfig(Ampere(60), Ampere(240), 1.0),
            ),
            gearing.MK4I.L3,
        )
    )

def create_robot(name: str, alliance: Alliance, mass: Pound = Pound(125)) -> SwerveRobot:
    """Create a swerve robot for the Rebuilt game."""
    structure = [
        Rectangle.from_center((Inch(0), Inch(0)), ROBOT_WIDTH, ROBOT_WIDTH).get_3d(
            Inch(0), Feet(2.5)
        )
    ]
    
    robot = SwerveRobot(
        name=name,
        drivetrain=create_swerve_config(),
        structure=structure,
        physics=PhysicalParameters(
            mass=mass,
            moi=mass * Inch(15) ** 2,
            max_acceleration=MeterPerSecondSquared(4.0),
        ),
    )
    robot.override_bounding_radius(Inch(16))
    robot.alliance = alliance
    return robot

def setup_robot_interactions(robot: SwerveRobot, game):
    """Configure interactions for a robot based on game interactables."""
    from gamegine.representation.interactable import RobotInteractionConfig
    
    for interactable in game.get_interactables():
        nav_point = interactable.get_navigation_point()
        for interaction in interactable.get_interactions():
            # Create interaction config
            # Use sensible defaults
            config = RobotInteractionConfig(
                interactable_name=interactable.name,
                interaction_identifier=interaction.identifier,
                able_to_interact=lambda *args, **kwargs: True,  # Robot can try anything
                time_to_interact=lambda *args, **kwargs: 0.5,
                navigation_point=nav_point,
            )
            robot.add_interaction_config(config)
