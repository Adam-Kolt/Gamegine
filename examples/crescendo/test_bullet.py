from dataclasses import dataclass
from typing import List
from gamegine.reference import gearing, motors
from gamegine.reference.motors import MotorConfig
from gamegine.reference.swerve import SwerveConfig, SwerveModule
from gamegine.reference.wheels import TreadDB, Wheel
from gamegine.representation.bounds import Transform3D
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
from examples.crescendo.crescendo import Crescendo

import time


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
