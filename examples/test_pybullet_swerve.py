from dataclasses import dataclass
from typing import List
from gamegine.utils.NCIM.ComplexDimensions.acceleration import MeterPerSecondSquared
from gamegine.utils.NCIM.Dimensions.mass import Kilogram, Pound
from gamegine.utils.NCIM.Dimensions.spatial import Inch, Meter
from gamegine.simulation.environment import mybullet as mb
import pybullet as p

import time


mb.connect()

GRAVITY = MeterPerSecondSquared(-9.8)


p.resetSimulation()
mb.setGravity(GRAVITY)

# Parameters
mass = Pound(120)
width = Inch(30)
length = Inch(30)
base_thickness = Inch(1)
wheel_diameter = Inch(4)
ground_clearance = Inch(20)


ground = p.createCollisionShape(
    p.GEOM_PLANE,
    halfExtents=[100, 100, 0.1],
)

ground_visual = p.createVisualShape(
    p.GEOM_PLANE,
    halfExtents=[100, 100, 0.1],
)

p.createMultiBody(
    0,
    ground,
    ground_visual,
    [0, 0, 0],
    [0, 0, 0, 1],
    useMaximalCoordinates=True,
)


@dataclass
class Link:
    mass: float
    collision_shape: int
    visual_shape: int
    position: List[float]
    orientation: List[float]
    intertialFramePosition: List[float]
    intertialFrameOrientation: List[float]
    joint_type: int
    parent_index: int
    joint_axis: List[float]


def create_module(name, x, y, z, index) -> List[Link]:
    TopPiece = Link(
        mass=Kilogram(1).get_unit_magnitude(),
        collision_shape=p.createCollisionShape(
            p.GEOM_BOX,
            halfExtents=[
                0.1,
                0.1,
                0.05,
            ],
        ),
        visual_shape=p.createVisualShape(
            p.GEOM_BOX,
            halfExtents=[
                0.1,
                0.1,
                0.05,
            ],
        ),
        position=[x, y, z],
        orientation=[0, 0, 0, 1],
        intertialFramePosition=[0, 0, 0],
        intertialFrameOrientation=[0, 0, 0, 1],
        joint_type=p.JOINT_REVOLUTE,
        parent_index=0,
        joint_axis=[0, 0, 1],
    )

    Wheel = Link(
        mass=Kilogram(0.1).get_unit_magnitude(),
        collision_shape=p.createCollisionShape(
            p.GEOM_CYLINDER, radius=wheel_diameter.to(Meter) / 2, height=0.05
        ),
        visual_shape=p.createVisualShape(
            p.GEOM_CYLINDER, radius=wheel_diameter.to(Meter) / 2, length=0.05
        ),
        position=[0, 0, -0.05 - wheel_diameter.to(Meter) / 2],
        orientation=[0, 0.707, 0, 0.707],
        intertialFramePosition=[0, 0, 0],
        intertialFrameOrientation=[0, 0, 0, 1],
        joint_type=p.JOINT_REVOLUTE,
        parent_index=index,
        joint_axis=[0, 0, 1],
    )

    return [TopPiece, Wheel]


modules: List[Link] = []
width_m = width.to(Meter)
length_m = length.to(Meter)

modules.extend(
    create_module(
        "FrontLeft", -0.15, 0.15, -base_thickness.to(Meter) / 2, len(modules) + 1
    )
)
modules.extend(
    create_module(
        "FrontRight", 0.15, 0.15, -base_thickness.to(Meter) / 2, len(modules) + 1
    )
)
modules.extend(
    create_module(
        "BackLeft", -0.15, -0.15, -base_thickness.to(Meter) / 2, len(modules) + 1
    )
)
modules.extend(
    create_module(
        "BackRight", 0.15, -0.15, -base_thickness.to(Meter) / 2, len(modules) + 1
    )
)

print(modules)


robot = p.createMultiBody(
    mass.to(Kilogram),
    p.createCollisionShape(
        p.GEOM_BOX,
        halfExtents=[
            width.to(Meter) / 2,
            length.to(Meter) / 2,
            base_thickness.to(Meter) / 2,
        ],
    ),
    p.createVisualShape(
        p.GEOM_BOX,
        halfExtents=[
            width.to(Meter) / 2,
            length.to(Meter) / 2,
            base_thickness.to(Meter) / 2,
        ],
    ),
    [0, 0, ground_clearance.to(Meter)],
    [0, 0, 0, 1],
    [0, 0, 0],
    [0, 0, 0, 1],
    [module.mass for module in modules],
    [module.collision_shape for module in modules],
    [module.visual_shape for module in modules],
    [module.position for module in modules],
    [module.orientation for module in modules],
    [module.intertialFramePosition for module in modules],
    [module.intertialFrameOrientation for module in modules],
    [module.parent_index for module in modules],
    [module.joint_type for module in modules],
    [module.joint_axis for module in modules],
)

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
    for module in range(4):
        p.setJointMotorControl2(
            robot,
            module * 2 + 1,
            p.VELOCITY_CONTROL,
            targetVelocity=radian_velocity,
            force=force,
        )
        p.setJointMotorControl2(
            robot,
            module * 2,
            p.POSITION_CONTROL,
            targetPosition=targetAngle * 3.14,
            force=force,
        )

    p.stepSimulation()
    time.sleep(1.0 / 2400)

p.disconnect()
