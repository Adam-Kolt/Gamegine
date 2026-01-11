"""Camera specifications for FRC vision cameras.

This module provides standardized camera specifications following the same
pattern as motor specifications. Includes presets for commonly used FRC cameras.
"""

from dataclasses import dataclass
from typing import Tuple

from gamegine.utils.NCIM.Dimensions.angular import AngularMeasurement, Degree
from gamegine.utils.NCIM.Dimensions.spatial import SpatialMeasurement, Meter, Inch


@dataclass
class CameraSpecification:
    """Stores specifications of a vision camera.

    :param name: Human-readable name of the camera.
    :type name: str
    :param fov_horizontal: Horizontal field of view.
    :type fov_horizontal: :class:`AngularMeasurement`
    :param fov_vertical: Vertical field of view.
    :type fov_vertical: :class:`AngularMeasurement`
    :param resolution: Pixel resolution as (width, height).
    :type resolution: Tuple[int, int]
    :param max_detection_distance: Maximum reliable detection distance.
    :type max_detection_distance: :class:`SpatialMeasurement`
    :param min_detection_distance: Minimum focus distance.
    :type min_detection_distance: :class:`SpatialMeasurement`
    """

    name: str
    fov_horizontal: AngularMeasurement
    fov_vertical: AngularMeasurement
    resolution: Tuple[int, int]
    max_detection_distance: SpatialMeasurement
    min_detection_distance: SpatialMeasurement

    def get_pixels_per_degree_h(self) -> float:
        """Returns horizontal pixels per degree of FOV."""
        return self.resolution[0] / float(self.fov_horizontal.to(Degree))

    def get_pixels_per_degree_v(self) -> float:
        """Returns vertical pixels per degree of FOV."""
        return self.resolution[1] / float(self.fov_vertical.to(Degree))


# =============================================================================
# Limelight Camera Family
# =============================================================================

Limelight2 = CameraSpecification(
    name="Limelight 2",
    fov_horizontal=Degree(59.6),
    fov_vertical=Degree(49.7),
    resolution=(320, 240),
    max_detection_distance=Meter(5),
    min_detection_distance=Meter(0.3),
)

Limelight2Plus = CameraSpecification(
    name="Limelight 2+",
    fov_horizontal=Degree(59.6),
    fov_vertical=Degree(49.7),
    resolution=(320, 240),
    max_detection_distance=Meter(5.5),
    min_detection_distance=Meter(0.3),
)

Limelight3 = CameraSpecification(
    name="Limelight 3",
    fov_horizontal=Degree(63.3),
    fov_vertical=Degree(49.7),
    resolution=(1280, 960),
    max_detection_distance=Meter(6),
    min_detection_distance=Meter(0.3),
)

Limelight3G = CameraSpecification(
    name="Limelight 3G",
    fov_horizontal=Degree(80),
    fov_vertical=Degree(64),
    resolution=(1280, 800),
    max_detection_distance=Meter(6),
    min_detection_distance=Meter(0.2),
)

# =============================================================================
# ArduCam Camera Family (commonly used with PhotonVision)
# =============================================================================

ArduCam_OV9281 = CameraSpecification(
    name="ArduCam OV9281",
    fov_horizontal=Degree(80),
    fov_vertical=Degree(64),
    resolution=(1280, 800),
    max_detection_distance=Meter(5),
    min_detection_distance=Meter(0.2),
)

ArduCam_OV2311 = CameraSpecification(
    name="ArduCam OV2311",
    fov_horizontal=Degree(80),
    fov_vertical=Degree(64),
    resolution=(1600, 1200),
    max_detection_distance=Meter(6),
    min_detection_distance=Meter(0.2),
)

ArduCam_OV9782 = CameraSpecification(
    name="ArduCam OV9782",
    fov_horizontal=Degree(120),
    fov_vertical=Degree(90),
    resolution=(1280, 800),
    max_detection_distance=Meter(4),
    min_detection_distance=Meter(0.15),
)

# =============================================================================
# Microsoft LifeCam (common budget option)
# =============================================================================

LifeCam_HD3000 = CameraSpecification(
    name="Microsoft LifeCam HD-3000",
    fov_horizontal=Degree(68.5),
    fov_vertical=Degree(54.4),
    resolution=(1280, 720),
    max_detection_distance=Meter(4),
    min_detection_distance=Meter(0.3),
)

# =============================================================================
# Generic/Custom Camera Template
# =============================================================================

def create_camera(
    name: str,
    fov_h: AngularMeasurement,
    fov_v: AngularMeasurement,
    resolution: Tuple[int, int],
    max_range: SpatialMeasurement = Meter(5),
    min_range: SpatialMeasurement = Meter(0.2),
) -> CameraSpecification:
    """Factory function to create custom camera specifications.
    
    :param name: Camera name.
    :param fov_h: Horizontal field of view.
    :param fov_v: Vertical field of view.
    :param resolution: Resolution as (width, height).
    :param max_range: Maximum detection range.
    :param min_range: Minimum detection range.
    :return: CameraSpecification instance.
    """
    return CameraSpecification(
        name=name,
        fov_horizontal=fov_h,
        fov_vertical=fov_v,
        resolution=resolution,
        max_detection_distance=max_range,
        min_detection_distance=min_range,
    )
