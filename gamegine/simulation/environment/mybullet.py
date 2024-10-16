from dataclasses import dataclass
from typing import List, Union

from gamegine.representation.bounds import Boundary3D
from gamegine.simulation.environment import PYBULLET_UNITS
from gamegine.simulation.environment.shape import BulletShape, ShapeBuilder
from gamegine.utils.NCIM.ComplexDimensions.acceleration import (
    Acceleration,
    AccelerationUnit,
)
from gamegine.utils.NCIM.Dimensions.mass import Kilogram, MassMeasurement
from gamegine.utils.NCIM.Dimensions.spatial import Meter, SpatialMeasurement
import pybullet as p
from enum import Enum


def connect():
    cid = p.connect(p.SHARED_MEMORY)
    if cid < 0:
        p.connect(p.GUI)


def resetSimulation():
    p.resetSimulation()


def setGravity(gravity: Acceleration):
    p.setGravity(
        0, 0, gravity.to(AccelerationUnit(PYBULLET_UNITS.SPATIAL, PYBULLET_UNITS.TIME))
    )


def addUserDebugParameter(
    name: str, min_value: float, max_value: float, default_value: float
):
    return p.addUserDebugParameter(name, min_value, max_value, default_value)


def readUserDebugParameter(param: int):
    try:
        return p.readUserDebugParameter(param)
    except Exception:
        return 0.0


def stepSimulation():
    p.stepSimulation()


def disconnect():
    p.disconnect()
