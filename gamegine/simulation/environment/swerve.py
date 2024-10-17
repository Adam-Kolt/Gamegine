from typing import List
from gamegine.reference.swerve import SwerveConfig, SwerveModule
from gamegine.simulation.environment.object import Joint, ObjectNode, GeneratedBody
from gamegine.simulation.environment.shape import BulletCylinder
from gamegine.utils.NCIM.Dimensions.spatial import SpatialMeasurement, Inch
from gamegine.utils.NCIM.Dimensions.mass import Kilogram
import pybullet as p


class BulletSwerveDrivetrain(GeneratedBody):
    def __init__(self, base: ObjectNode, config: SwerveConfig):
        self.module_objects = construct_swerve(base, config)
        super().__init__(base, base.createMultiBody())

    def get_rotation_joints(self) -> List[int]:
        return [module.link_index - 1 for module in self.module_objects]

    def get_wheel_joints(self) -> List[int]:
        return [module.link_index for module in self.module_objects]

    def set_module_states(self, states):
        pass

    def get_module_states(self):
        pass


def create_module(
    x: SpatialMeasurement, y: SpatialMeasurement, module: SwerveModule
) -> ObjectNode:
    plate_geometry = BulletCylinder(module.wheel.diameter / 2, Inch(1))
    rotation_plate = ObjectNode(
        mass=Kilogram(0.5),
        collision_shape=plate_geometry,
        visual_shape=plate_geometry,
        position=[x, y, Inch(0)],
        orientation=[0, 0, 0, 1],
    )

    wheel_geometry = BulletCylinder(module.wheel.diameter / 2, module.wheel.width)
    wheel = ObjectNode(
        mass=Kilogram(0.5),
        collision_shape=wheel_geometry,
        visual_shape=wheel_geometry,
        position=[Inch(0), Inch(0), -module.wheel.diameter / 2 - Inch(0.5)],
        orientation=[0, 0.707, 0, 0.707],
    )
    wheel.get_dynamics().set_lateral_friction(module.wheel.grip() * 10)

    rotation_plate.link(wheel, Joint(p.JOINT_REVOLUTE, [0, 0, 1]))

    return rotation_plate


def construct_swerve(base: ObjectNode, config: SwerveConfig) -> List[ObjectNode]:
    offsets = [
        config.top_left_offset,
        config.top_right_offset,
        config.bottom_left_offset,
        config.bottom_right_offset,
    ]
    modules = []
    for offset in offsets:
        module = create_module(*offset, config.module)
        base.link(module, Joint(p.JOINT_REVOLUTE, [0, 0, 1]))
        modules.append(module)

    return modules
