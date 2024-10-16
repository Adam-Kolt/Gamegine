from dataclasses import dataclass
from typing import List
from gamegine.reference import gearing, motors
from gamegine.reference.motors import MotorConfig
from gamegine.reference.swerve import SwerveConfig, SwerveModule
from gamegine.simulation.environment.object import ObjectNode
from gamegine.simulation.environment.shape import BulletBox, BulletPlane
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
base_thickness = Inch(1)
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
    position=[Feet(0), Feet(0), ground_clearance],
    orientation=[0, 0, 0, 1],
)

swerve_drivetrain = BulletSwerveDrivetrain(robot_base, swerve)


ground = ground.generate_bullet_object()

targetVelocitySlider = p.addUserDebugParameter("wheelVelocity", -10, 10, 0)
targetAngleSlider = p.addUserDebugParameter("targetAngle", -1, 1, 0)
force = 1000
while True:
    try:
        targetVelocity = p.readUserDebugParameter(targetVelocitySlider)
        targetAngle = p.readUserDebugParameter(targetAngleSlider)
    except p.error:
        targetVelocity = 0
        targetAngle = 0

    radian_velocity = targetVelocity / (wheel_diameter.to(Meter) / 2)
    for module in swerve_drivetrain.get_rotation_joints():

        p.setJointMotorControl2(
            swerve_drivetrain.bullet_id,
            module - 1,
            p.POSITION_CONTROL,
            targetPosition=targetAngle * 3.14,
            force=force,
        )

    for module in swerve_drivetrain.get_wheel_joints():
        p.setJointMotorControl2(
            swerve_drivetrain.bullet_id,
            module - 1,
            p.VELOCITY_CONTROL,
            targetVelocity=radian_velocity,
            force=force,
        )


p.disconnect()
