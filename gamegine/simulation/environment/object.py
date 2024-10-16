from dataclasses import dataclass, field
from typing import List, Union

from gamegine.representation.bounds import Boundary3D
from gamegine.simulation.environment import PYBULLET_UNITS
from gamegine.simulation.environment.shape import BulletShape, ShapeBuilder
from gamegine.utils.NCIM.Dimensions.mass import Kilogram, MassMeasurement
from gamegine.utils.NCIM.Dimensions.spatial import Meter, SpatialMeasurement
import pybullet as p
from enum import Enum


@dataclass
class Joint:
    type: int
    axis: list[float]


@dataclass
class Link:
    object: "ObjectNode"
    joint: Joint
    parent_index: int = -1


@dataclass
class GeneratedBody:
    object: "ObjectNode"
    bullet_id: int

    def __recursive_compute_dynamics(self, object: "ObjectNode"):
        object.get_dynamics().apply(self.bullet_id, object.link_index)
        for child in object.children:
            self.__recursive_compute_dynamics(child.object)

    def compute_dynamics(self):
        self.__recursive_compute_dynamics(self.object)

    def __post_init__(self):
        self.compute_dynamics()


@dataclass
class ObjectDynamics:
    lateral_friction: float = 0.5
    spinning_friction: float = 0.0
    rolling_friction: float = 0.0
    restitution: float = 0.0
    linear_damping: float = 0.0
    angular_damping: float = 0.0
    contact_damping: float = -1.0
    contact_stiffness: float = -1.0
    friction_anchor: bool = False
    local_inertia_diagonal: List[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])
    local_inertia_frame_position: List[float] = field(
        default_factory=lambda: [0.0, 0.0, 0.0]
    )
    local_inertia_frame_orientation: List[float] = field(
        default_factory=lambda: [0.0, 0.0, 0.0, 1.0]
    )
    collision_margin: float = 0.0
    anisotropic_friction: List[float] = field(default_factory=lambda: [1.0, 1.0, 1.0])
    max_joint_velocity: float = 1000.0
    coulomb_friction: float = 0.5
    friction_ce: float = 0.0
    friction_erp: float = 0.2
    rolling_friction_erp: float = 0.1
    restitution_threshold: float = 0.5
    disable_deactivation: bool = False
    disable_collision_response: bool = False
    enable_gyro: bool = False

    # Getters
    def get_lateral_friction(self):
        return self.lateral_friction

    def get_spinning_friction(self):
        return self.spinning_friction

    def get_rolling_friction(self):
        return self.rolling_friction

    def get_restitution(self):
        return self.restitution

    def get_linear_damping(self):
        return self.linear_damping

    def get_angular_damping(self):
        return self.angular_damping

    def get_contact_damping(self):
        return self.contact_damping

    def get_contact_stiffness(self):
        return self.contact_stiffness

    def get_friction_anchor(self):
        return self.friction_anchor

    def get_local_inertia_diagonal(self):
        return self.local_inertia_diagonal

    def get_local_inertia_frame_position(self):
        return self.local_inertia_frame_position

    def get_local_inertia_frame_orientation(self):
        return self.local_inertia_frame_orientation

    def get_collision_margin(self):
        return self.collision_margin

    def get_anisotropic_friction(self):
        return self.anisotropic_friction

    def get_max_joint_velocity(self):
        return self.max_joint_velocity

    def get_coulomb_friction(self):
        return self.coulomb_friction

    def get_friction_ce(self):
        return self.friction_ce

    def get_friction_erp(self):
        return self.friction_erp

    def get_rolling_friction_erp(self):
        return self.rolling_friction_erp

    def get_restitution_threshold(self):
        return self.restitution_threshold

    def get_disable_deactivation(self):
        return self.disable_deactivation

    def get_disable_collision_response(self):
        return self.disable_collision_response

    def get_enable_gyro(self):
        return self.enable_gyro

    def set_lateral_friction(self, value: float) -> "ObjectDynamics":
        self.lateral_friction = value
        return self

    def set_spinning_friction(self, value: float) -> "ObjectDynamics":
        self.spinning_friction = value
        return self

    def set_rolling_friction(self, value: float) -> "ObjectDynamics":
        self.rolling_friction = value
        return self

    def set_restitution(self, value: float) -> "ObjectDynamics":
        self.restitution = value
        return self

    def set_linear_damping(self, value: float) -> "ObjectDynamics":
        self.linear_damping = value
        return self

    def set_angular_damping(self, value: float) -> "ObjectDynamics":
        self.angular_damping = value
        return self

    def set_contact_damping(self, value: float) -> "ObjectDynamics":
        self.contact_damping = value
        return self

    def set_contact_stiffness(self, value: float) -> "ObjectDynamics":
        self.contact_stiffness = value
        return self

    def set_friction_anchor(self, value: bool) -> "ObjectDynamics":
        self.friction_anchor = value
        return self

    def set_local_inertia_diagonal(self, value: List[float]) -> "ObjectDynamics":
        self.local_inertia_diagonal = value
        return self

    def set_local_inertia_frame_position(self, value: List[float]) -> "ObjectDynamics":
        self.local_inertia_frame_position = value
        return self

    def set_local_inertia_frame_orientation(
        self, value: List[float]
    ) -> "ObjectDynamics":
        self.local_inertia_frame_orientation = value
        return self

    def set_collision_margin(self, value: float) -> "ObjectDynamics":
        self.collision_margin = value
        return self

    def set_anisotropic_friction(self, value: List[float]) -> "ObjectDynamics":
        self.anisotropic_friction = value
        return self

    def set_max_joint_velocity(self, value: float) -> "ObjectDynamics":
        self.max_joint_velocity = value
        return self

    def set_coulomb_friction(self, value: float) -> "ObjectDynamics":
        self.coulomb_friction = value
        return self

    def set_friction_ce(self, value: float) -> "ObjectDynamics":
        self.friction_ce = value
        return self

    def set_friction_erp(self, value: float) -> "ObjectDynamics":
        self.friction_erp = value
        return self

    def set_rolling_friction_erp(self, value: float) -> "ObjectDynamics":
        self.rolling_friction_erp = value
        return self

    def set_restitution_threshold(self, value: float) -> "ObjectDynamics":
        self.restitution_threshold = value
        return self

    def set_disable_deactivation(self, value: bool) -> "ObjectDynamics":
        self.disable_deactivation = value
        return self

    def set_disable_collision_response(self, value: bool) -> "ObjectDynamics":
        self.disable_collision_response = value
        return self

    def set_enable_gyro(self, value: bool) -> "ObjectDynamics":
        self.enable_gyro = value
        return self

    def apply(self, body_id: int, link_index: int = -1):
        p.changeDynamics(
            bodyUniqueId=body_id,
            linkIndex=link_index,
            lateralFriction=self.lateral_friction,
            spinningFriction=self.spinning_friction,
            rollingFriction=self.rolling_friction,
            restitution=self.restitution,
            linearDamping=self.linear_damping,
            angularDamping=self.angular_damping,
            contactDamping=self.contact_damping,
            contactStiffness=self.contact_stiffness,
            frictionAnchor=self.friction_anchor,
            collisionMargin=self.collision_margin,
            anisotropicFriction=self.anisotropic_friction,
            maxJointVelocity=self.max_joint_velocity,
        )


@dataclass
class ObjectNode:
    mass: MassMeasurement
    collision_shape: BulletShape
    visual_shape: BulletShape
    position: list[SpatialMeasurement]
    orientation: list[float]
    inertialFramePosition: list[float] = field(default_factory=lambda: [0, 0, 0])
    inertialFrameOrientation: list[float] = field(default_factory=lambda: [0, 0, 0, 1])
    children: list[Link] = field(default_factory=list)
    link_index: int = -1
    dynamics: ObjectDynamics = field(default_factory=ObjectDynamics)

    def get_bullet_mass(self) -> float:
        return self.mass.to(PYBULLET_UNITS.MASS)

    def get_bullet_position(self) -> list[float]:
        return [pos.to(PYBULLET_UNITS.SPATIAL) for pos in self.position]

    def link(self, object: "ObjectNode", joint: Joint):
        object.parent = self
        self.children.append(Link(object, joint))

    def get_collision_bullet_shape(self) -> int:
        return ShapeBuilder.build(self.collision_shape)

    def get_visual_bullet_shape(self) -> int:
        return ShapeBuilder.build(self.visual_shape)

    def get_children(self, curr_index: int) -> List[Link]:
        self.link_index = curr_index
        out = []
        for child in self.children:
            child.parent_index = curr_index
            out.append(child)
            out += child.object.get_children(curr_index + len(out))
        return out

    def get_dynamics(self) -> ObjectDynamics:
        return self.dynamics

    def set_dynamics(self, dynamics: ObjectDynamics):
        self.dynamics = dynamics

    def createMultiBody(self) -> int:
        children = self.get_children(0)  # Recursion is cool :Thumbs up:

        id = p.createMultiBody(
            self.get_bullet_mass(),
            self.get_collision_bullet_shape(),
            self.get_visual_bullet_shape(),
            self.get_bullet_position(),
            self.orientation,
            self.inertialFramePosition,
            self.inertialFrameOrientation,
            [child.object.get_bullet_mass() for child in children],
            [child.object.get_collision_bullet_shape() for child in children],
            [child.object.get_visual_bullet_shape() for child in children],
            [child.object.get_bullet_position() for child in children],
            [child.object.orientation for child in children],
            [child.object.inertialFramePosition for child in children],
            [child.object.inertialFrameOrientation for child in children],
            [child.parent_index for child in children],
            [child.joint.type for child in children],
            [child.joint.axis for child in children],
        )
        return id

    def generate_bullet_object(self) -> GeneratedBody:
        return GeneratedBody(self, self.createMultiBody())
