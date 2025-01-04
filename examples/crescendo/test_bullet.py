from dataclasses import dataclass
from typing import List
from gamegine.reference import gearing, motors
from gamegine.reference.motors import MotorConfig
from gamegine.reference.swerve import SwerveConfig, SwerveModule
from gamegine.reference.wheels import TreadDB, Wheel
from gamegine.representation.bounds import (
    CircularPattern,
    Cylinder,
    SymmetricalX,
    Transform3D,
)
from gamegine.representation.game import Game
from gamegine.representation.obstacle import Obstacle
from gamegine.simulation.environment.object import Joint, ObjectNode
from gamegine.simulation.environment.shape import BulletBox, BulletPlane, BulletShape
from gamegine.simulation.environment.swerve import (
    BulletSwerveDrivetrain,
    construct_swerve,
)
from gamegine.utils.NCIM.ComplexDimensions.acceleration import MeterPerSecondSquared
from gamegine.utils.NCIM.Dimensions.mass import Kilogram, Pound
from gamegine.utils.NCIM.Dimensions.spatial import Inch, Feet, Meter
from gamegine.utils.NCIM.Dimensions.current import Ampere
from gamegine.simulation.environment import mybullet as mb
import pybullet as p
import pygame

from gamegine.analysis.meshing import TriangulatedGraph, VisibilityGraph


from gamegine.analysis.trajectory.lib.TrajGen import (
    SolverConfig,
    SwerveRobotConstraints,
    SwerveTrajectoryProblemBuilder,
    TrajectoryBuilderConfig,
    Waypoint,
)
from gamegine.analysis.trajectory.lib.constraints.avoidance import (
    SAFETY_CORRIDOR_DEBUG,
    SafetyCorridor,
)
from gamegine.analysis.trajectory.lib.constraints.constraints import (
    AngleEquals,
    VelocityEquals,
)
from gamegine.reference import gearing, motors
from gamegine.reference.swerve import SwerveConfig, SwerveModule
from gamegine.render.renderer import Renderer
from gamegine.representation.apriltag import AprilTag, AprilTagFamily
from gamegine.representation.bounds import (
    Circle,
    Cylinder,
    ExpandedObjectBounds,
    Point,
    Polygon,
    Rectangle,
    SymmetricalX,
    CircularPattern,
    Transform3D,
)
from gamegine.representation.game import Game
from gamegine.representation.obstacle import Circular, Obstacle, Polygonal, Rectangular
from gamegine.representation.robot import (
    PhysicalParameters,
)
from gamegine.utils.NCIM.ComplexDimensions.MOI import PoundsInchesSquared
from gamegine.utils.NCIM.ComplexDimensions.acceleration import MeterPerSecondSquared
from gamegine.utils.NCIM.ComplexDimensions.alpha import RadiansPerSecondSquared
from gamegine.utils.NCIM.ComplexDimensions.electricpot import Volt
from gamegine.utils.NCIM.ComplexDimensions.omega import (
    RadiansPerSecond,
    RotationsPerSecond,
)
from gamegine.utils.NCIM.Dimensions.angular import AngularMeasurement, Radian
from gamegine.utils.NCIM.Dimensions.current import Ampere
from gamegine.utils.NCIM.ncim import (
    Degree,
    Kilogram,
    KilogramMetersSquared,
    Meter,
    Centimeter,
    Feet,
    Inch,
    MetersPerSecond,
    Pound,
    SpatialMeasurement,
)
from gamegine.analysis import pathfinding
import time

from gamegine.utils.logging import Debug

import time

Crescendo = Game("FRC Crescendo 2024")

print("Name:", Crescendo.name)
Crescendo.set_field_size(Feet(54) + Inch(3.25), Feet(26) + Inch(11.25))
objs = SymmetricalX(
    [
        *CircularPattern(
            [
                Obstacle(
                    "Stage Leg",
                    Cylinder(
                        Inch(7),
                        Inch(74.5),
                        Transform3D((Inch(133), Inch(161.62), Inch(74.5) / 2)),
                    ),
                )
            ],  # Circular("Stage Leg", Inch(133), Inch(161.62), Inch(7))
            (Inch(133) + Inch(59.771), Inch(161.62)),
            Degree(360),
            3,
            lambda i: str(i),
        ),
        Obstacle(
            "Subwoofer",
            Polygon(
                [
                    (Inch(0), Inch(64.081)),
                    (Inch(0), Inch(64.081) + Inch(82.645)),
                    (Inch(35.695), Inch(64.081) + Inch(82.645) - Inch(20.825)),
                    (Inch(35.695), Inch(64.081) + Inch(20.825)),
                ]
            ).get_3d(z_end=Feet(2)),
        ),
        Obstacle(
            "Source",
            Polygon(
                [
                    (Inch(0), Inch(281.5)),
                    (Inch(0), Crescendo.full_field_y()),
                    (Inch(72.111), Crescendo.full_field_y()),
                ]
            ).get_3d(z_end=Feet(4)),
        ),
        Obstacle(
            "Stage Base",
            Polygon(
                [
                    (Inch(133), Inch(161.62)),
                    (
                        Inch(133) + Inch(29.855) + Inch(59.77),
                        Inch(161.62) + Inch(51.76),
                    ),
                    (
                        Inch(133) + Inch(29.855) + Inch(59.77),
                        Inch(161.62) - Inch(51.76),
                    ),
                ]
            ).get_3d(Inch(27.83), Inch(74.5)),
        ),
        # Circular("Note 1", Inch(114.010), Inch(47.638), Inch(7)),
        # Circular("Note 2", Inch(114.010), Inch(47.638) + Inch(43.000), Inch(7)),
        # Circular("Note 3", Inch(114.010), Inch(47.638) + Inch(43.000) * 2, Inch(7)),
    ],
    Crescendo.half_field_x(),
    "Red ",
    "Blue ",
)


Crescendo.add_obstacles(objs)
Crescendo.enable_field_border_obstacles()


mb.connect()

GRAVITY = MeterPerSecondSquared(-9.8)


p.resetSimulation()
p.setRealTimeSimulation(1)
mb.setGravity(GRAVITY)

# Parameters
mass = Pound(120)
width = Inch(30)
length = Inch(30)
base_thickness = Inch(2)
wheel_diameter = Inch(4)
ground_clearance = Inch(20)


ground_shape = BulletPlane(Feet(100), Feet(100), Feet(0.1))


Kraken_Config = MotorConfig(
    motors.KrakenX60, motors.PowerConfig(Ampere(60), Ampere(240))
)
swerve = SwerveConfig(
    module=SwerveModule(
        motors.MotorConfig(
            motors.KrakenX60,
            motors.PowerConfig(Ampere(60), Ampere(360), 1.0),
        ),
        gearing.MK4I.L3,
        motors.MotorConfig(
            motors.KrakenX60,
            motors.PowerConfig(Ampere(60), Ampere(360), 1.0),
        ),
        gearing.MK4I.L3,
        wheel=Wheel(Inch(4), TreadDB.am_blue_nitrile_roughtop_tread, Inch(3)),
    )
)


ground = ObjectNode(
    mass=Kilogram(0),
    collision_shape=ground_shape,
    visual_shape=ground_shape,
    position=[Feet(0), Feet(0), Feet(0)],
    orientation=[0, 0, 0, 1],
)

robot_base = ObjectNode(
    mass=mass,
    collision_shape=BulletBox(length, width, base_thickness),
    visual_shape=BulletBox(length, width, base_thickness),
    position=[Feet(3), Feet(3), ground_clearance],
    orientation=[0, 0, 0, 1],
)


obstacles = Crescendo.get_obstacles()

for obstacle in obstacles:
    bound = obstacle.bounds

    if not hasattr(bound, "get_bullet_shape"):
        continue
    shape: BulletShape = bound.get_bullet_shape()
    transform: Transform3D = bound.get_transform()

    bullet_object = ObjectNode(
        mass=Kilogram(0),
        collision_shape=shape,
        visual_shape=shape,
        position=transform.position,
        orientation=[0, 0, 0, 1],
    )

    # Make object static and not clippable
    bullet_object.get_dynamics().lateral_friction = 0.5

    ground.link(bullet_object, Joint(p.JOINT_FIXED, [0, 0, 0]))


ground = ground.generate_bullet_object()

swerve_drivetrain = BulletSwerveDrivetrain(robot_base, swerve)

targetVelocitySlider = p.addUserDebugParameter("wheelVelocity", -5, 5, 0)
targetAngleSlider = p.addUserDebugParameter("targetAngle", -1, 1, 0)
force = 30
while True:
    try:
        targetVelocity = p.readUserDebugParameter(targetVelocitySlider)
        targetAngle = p.readUserDebugParameter(targetAngleSlider)
    except p.error:
        targetVelocity = 0
        targetAngle = 0

    radian_velocity = targetVelocity / (wheel_diameter.to(Meter) / 2)

    p.setJointMotorControlArray(
        swerve_drivetrain.bullet_id,
        swerve_drivetrain.get_rotation_joints(),
        controlMode=p.POSITION_CONTROL,
        targetPositions=[targetAngle * 3.14] * 4,
        forces=[1000] * 4,
    )

    # Array Set
    p.setJointMotorControlArray(
        swerve_drivetrain.bullet_id,
        swerve_drivetrain.get_wheel_joints(),
        controlMode=p.VELOCITY_CONTROL,
        targetVelocities=[radian_velocity] * 4,
        forces=[force] * 4,
    )


p.disconnect()
