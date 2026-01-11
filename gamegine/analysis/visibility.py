"""AprilTag visibility analysis for camera placement optimization.

This module provides tools to analyze AprilTag visibility from different
camera positions, accounting for obstacles, tag orientation, and detection
quality factors.
"""

from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple
import math

from gamegine.representation.apriltag import AprilTag
from gamegine.representation.bounds import Point, Boundary3D
from gamegine.representation.game import Game
from gamegine.representation.obstacle import Obstacle, Obstacle3D
from gamegine.representation.camera import CameraMount
from gamegine.reference.cameras import CameraSpecification
from gamegine.utils.NCIM.Dimensions.angular import AngularMeasurement, Degree, Radian
from gamegine.utils.NCIM.Dimensions.spatial import SpatialMeasurement, Meter, Inch


# =============================================================================
# Ray Casting Utilities
# =============================================================================

@dataclass
class RayHit:
    """Result of a ray cast that hit an obstacle.
    
    :param distance: Distance from ray origin to hit point.
    :param hit_point: The 3D point where the ray hit.
    :param obstacle: The obstacle that was hit.
    """
    distance: SpatialMeasurement
    hit_point: Point
    obstacle: Obstacle


def _line_segment_intersects_rectangle(
    ray_start: Tuple[float, float],
    ray_end: Tuple[float, float],
    rect_min: Tuple[float, float],
    rect_max: Tuple[float, float],
) -> bool:
    """Check if a 2D line segment intersects an axis-aligned rectangle.
    
    Uses the Liang-Barsky algorithm for efficiency.
    """
    x1, y1 = ray_start
    x2, y2 = ray_end
    dx = x2 - x1
    dy = y2 - y1
    
    t_min = 0.0
    t_max = 1.0
    
    # Check each edge
    for p, q in [
        (-dx, x1 - rect_min[0]),  # Left edge
        (dx, rect_max[0] - x1),   # Right edge
        (-dy, y1 - rect_min[1]),  # Bottom edge
        (dy, rect_max[1] - y1),   # Top edge
    ]:
        if p == 0:
            if q < 0:
                return False
        else:
            t = q / p
            if p < 0:
                t_min = max(t_min, t)
            else:
                t_max = min(t_max, t)
    
    return t_min <= t_max


def cast_ray_2d(
    origin_x: SpatialMeasurement,
    origin_y: SpatialMeasurement,
    target_x: SpatialMeasurement,
    target_y: SpatialMeasurement,
    obstacles: List[Obstacle],
) -> Optional[RayHit]:
    """Cast a 2D ray and return the first obstacle hit.
    
    This simplified version projects 3D obstacles to 2D for intersection testing.
    
    :param origin_x: X coordinate of ray origin.
    :param origin_y: Y coordinate of ray origin.
    :param target_x: X coordinate of ray target.
    :param target_y: Y coordinate of ray target.
    :param obstacles: List of obstacles to test against.
    :return: RayHit if an obstacle is hit, None otherwise.
    """
    ox = float(origin_x.to(Meter))
    oy = float(origin_y.to(Meter))
    tx = float(target_x.to(Meter))
    ty = float(target_y.to(Meter))
    
    # Calculate ray direction and length
    dx = tx - ox
    dy = ty - oy
    ray_length = math.sqrt(dx * dx + dy * dy)
    
    if ray_length < 0.001:
        return None
    
    closest_hit: Optional[RayHit] = None
    closest_dist = ray_length
    
    for obstacle in obstacles:
        if not obstacle.isVisible():
            continue
        
        # Get obstacle bounds
        bounds = obstacle.bounds
        if hasattr(bounds, 'get_bounded_rectangle'):
            rect = bounds.get_bounded_rectangle()
            rect_min = (float(rect.x.to(Meter)), float(rect.y.to(Meter)))
            rect_max = (
                rect_min[0] + float(rect.width.to(Meter)),
                rect_min[1] + float(rect.height.to(Meter)),
            )
            
            if _line_segment_intersects_rectangle(
                (ox, oy), (tx, ty), rect_min, rect_max
            ):
                # Simple distance approximation to rectangle center
                center_x = (rect_min[0] + rect_max[0]) / 2
                center_y = (rect_min[1] + rect_max[1]) / 2
                dist = math.sqrt((center_x - ox) ** 2 + (center_y - oy) ** 2)
                
                if dist < closest_dist:
                    closest_dist = dist
                    closest_hit = RayHit(
                        distance=Meter(dist),
                        hit_point=Point(Meter(center_x), Meter(center_y), Meter(0)),
                        obstacle=obstacle,
                    )
    
    return closest_hit


def is_line_of_sight_clear(
    from_pos: Point,
    to_pos: Point,
    obstacles: List[Obstacle],
    z_tolerance: SpatialMeasurement = Inch(6),
) -> Tuple[bool, Optional[Obstacle]]:
    """Check if there's a clear line of sight between two 3D points.
    
    :param from_pos: Starting position.
    :param to_pos: Target position.
    :param obstacles: List of obstacles to check.
    :param z_tolerance: Height tolerance for intersection (default 6 inches).
    :return: Tuple of (is_clear, blocking_obstacle).
    """
    # Filter obstacles by height relevance
    from_z = float(from_pos.z.to(Meter))
    to_z = float(to_pos.z.to(Meter))
    min_z = min(from_z, to_z) - float(z_tolerance.to(Meter))
    max_z = max(from_z, to_z) + float(z_tolerance.to(Meter))
    
    relevant_obstacles = []
    for obs in obstacles:
        if isinstance(obs, Obstacle3D):
            z_min, z_max = obs.get_z_interval()
            obs_z_min = float(z_min.to(Meter))
            obs_z_max = float(z_max.to(Meter))
            # Check if obstacle overlaps with ray's Z range
            if obs_z_max >= min_z and obs_z_min <= max_z:
                relevant_obstacles.append(obs)
        else:
            # Assume ground-level obstacles
            relevant_obstacles.append(obs)
    
    hit = cast_ray_2d(from_pos.x, from_pos.y, to_pos.x, to_pos.y, relevant_obstacles)
    
    if hit:
        # Check if hit is closer than target
        target_dist = math.sqrt(
            (float(to_pos.x.to(Meter)) - float(from_pos.x.to(Meter))) ** 2
            + (float(to_pos.y.to(Meter)) - float(from_pos.y.to(Meter))) ** 2
        )
        if float(hit.distance.to(Meter)) < target_dist * 0.95:  # 5% margin
            return (False, hit.obstacle)
    
    return (True, None)


# =============================================================================
# Detection Quality
# =============================================================================

def calculate_detection_quality(
    distance: SpatialMeasurement,
    viewing_angle_offset: AngularMeasurement,
    tag_size: SpatialMeasurement,
    camera: CameraSpecification,
) -> float:
    """Calculate detection quality score (0.0 to 1.0).
    
    Factors considered:
    - Distance falloff (inverse relationship)
    - Angular penalty (worse at glancing angles)
    - Tag size vs camera resolution (pixel coverage)
    
    :param distance: Distance from camera to tag.
    :param viewing_angle_offset: Angle offset from tag normal.
    :param tag_size: Physical size of the tag.
    :param camera: Camera specification.
    :return: Quality score between 0.0 and 1.0.
    """
    dist_m = float(distance.to(Meter))
    max_dist = float(camera.max_detection_distance.to(Meter))
    min_dist = float(camera.min_detection_distance.to(Meter))
    
    # Out of range
    if dist_m > max_dist or dist_m < min_dist:
        return 0.0
    
    # Distance factor: linear falloff from optimal range
    # Optimal is at 30% of max range
    optimal_dist = min_dist + (max_dist - min_dist) * 0.3
    if dist_m <= optimal_dist:
        # Close range: slight penalty for being too close
        dist_factor = 0.9 + 0.1 * (dist_m - min_dist) / (optimal_dist - min_dist)
    else:
        # Far range: linear falloff
        dist_factor = 1.0 - 0.7 * (dist_m - optimal_dist) / (max_dist - optimal_dist)
    
    # Angular factor: cosine falloff
    angle_rad = abs(float(viewing_angle_offset.to(Radian)))
    angle_factor = max(0.0, math.cos(angle_rad))
    
    # Pixel coverage factor
    tag_size_m = float(tag_size.to(Meter))
    # Approximate pixels on target
    fov_h_rad = float(camera.fov_horizontal.to(Radian))
    pixels_per_meter = camera.resolution[0] / (2 * dist_m * math.tan(fov_h_rad / 2))
    tag_pixels = tag_size_m * pixels_per_meter
    
    # Minimum ~10 pixels for detection, optimal ~50 pixels
    if tag_pixels < 10:
        pixel_factor = 0.0
    elif tag_pixels < 50:
        pixel_factor = (tag_pixels - 10) / 40
    else:
        pixel_factor = 1.0
    
    return dist_factor * angle_factor * pixel_factor


# =============================================================================
# Visibility Analyzer
# =============================================================================

@dataclass
class TagVisibilityResult:
    """Result of visibility analysis for a single AprilTag.
    
    :param tag: The AprilTag being analyzed.
    :param is_visible: Whether the tag is visible (orientation + no occlusion).
    :param distance: Distance from camera to tag.
    :param viewing_angle_offset: Angle offset from tag's normal vector.
    :param detection_quality: Quality score from 0.0 to 1.0.
    :param blocking_obstacle: Obstacle blocking the view, if any.
    """
    tag: AprilTag
    is_visible: bool
    distance: SpatialMeasurement
    viewing_angle_offset: AngularMeasurement
    detection_quality: float
    blocking_obstacle: Optional[Obstacle] = None


@dataclass
class VisibilityGridCell:
    """Single cell in a visibility grid/heatmap.
    
    :param x: X coordinate of cell center.
    :param y: Y coordinate of cell center.
    :param heading: Robot heading for this sample.
    :param visible_tags: Number of tags visible from this position.
    :param total_quality: Sum of detection quality scores.
    :param best_tag_id: ID of the best-detected tag, if any.
    """
    x: SpatialMeasurement
    y: SpatialMeasurement
    heading: AngularMeasurement
    visible_tags: int
    total_quality: float
    best_tag_id: Optional[int] = None


class VisibilityAnalyzer:
    """Analyzes AprilTag visibility from camera positions.
    
    This class provides methods to evaluate tag visibility from specific
    positions and to generate coverage heatmaps across the field.
    
    :param game: The Game containing AprilTags and obstacles.
    :param camera: The camera specification to use for analysis.
    :param camera_mount: How the camera is mounted on the robot.
    """
    
    def __init__(
        self,
        game: Game,
        camera: CameraSpecification,
        camera_mount: Optional[CameraMount] = None,
    ):
        self.game = game
        self.camera = camera
        self.camera_mount = camera_mount or CameraMount(
            camera=camera,
            offset=Point(Inch(0), Inch(0), Inch(24)),  # 2ft high centered
            heading=Degree(0),
        )
        self._obstacles = list(game.get_obstacles())
        self._tags = game.get_apriltags()
    
    def analyze_from_position(
        self,
        robot_x: SpatialMeasurement,
        robot_y: SpatialMeasurement,
        robot_heading: AngularMeasurement,
    ) -> List[TagVisibilityResult]:
        """Analyze all tag visibility from a single robot position.
        
        :param robot_x: Robot X position.
        :param robot_y: Robot Y position.
        :param robot_heading: Robot heading.
        :return: List of visibility results for each tag.
        """
        # Get camera world position
        camera_pos = self.camera_mount.get_world_position(
            robot_x, robot_y, robot_heading
        )
        camera_heading = self.camera_mount.get_world_heading(robot_heading)
        
        results = []
        for tag in self._tags:
            # Calculate distance
            dx = float(tag.position.x.to(Meter)) - float(camera_pos.x.to(Meter))
            dy = float(tag.position.y.to(Meter)) - float(camera_pos.y.to(Meter))
            dz = float(tag.position.z.to(Meter)) - float(camera_pos.z.to(Meter))
            distance = Meter(math.sqrt(dx * dx + dy * dy + dz * dz))
            
            # Check angular visibility (is camera within tag's visible arc)
            is_angle_visible = tag.is_visible_from_position(camera_pos.x, camera_pos.y)
            
            # Calculate viewing angle offset from tag normal
            view_to_tag = math.atan2(dy, dx)
            tag_normal_angle = float(tag.heading.to(Radian))
            # Offset is difference between camera's view and tag's facing direction
            angle_offset = Radian(abs(view_to_tag + math.pi - tag_normal_angle))
            while float(angle_offset.to(Radian)) > math.pi:
                angle_offset = Radian(abs(float(angle_offset.to(Radian)) - 2 * math.pi))
            
            # Check if within camera FOV
            angle_to_tag = Radian(view_to_tag)
            cam_heading_rad = float(camera_heading.to(Radian))
            fov_offset = abs(view_to_tag - cam_heading_rad)
            while fov_offset > math.pi:
                fov_offset = abs(fov_offset - 2 * math.pi)
            
            is_in_fov = fov_offset <= float(self.camera.fov_horizontal.to(Radian)) / 2
            
            # Check line of sight
            is_clear, blocking_obs = is_line_of_sight_clear(
                camera_pos, tag.position, self._obstacles
            )
            
            is_visible = is_angle_visible and is_in_fov and is_clear
            
            # Calculate quality
            if is_visible:
                quality = calculate_detection_quality(
                    distance, angle_offset, tag.size, self.camera
                )
            else:
                quality = 0.0
            
            results.append(TagVisibilityResult(
                tag=tag,
                is_visible=is_visible,
                distance=distance,
                viewing_angle_offset=angle_offset,
                detection_quality=quality,
                blocking_obstacle=blocking_obs,
            ))
        
        return results
    
    def get_visible_tags(
        self,
        robot_x: SpatialMeasurement,
        robot_y: SpatialMeasurement,
        robot_heading: AngularMeasurement,
    ) -> List[AprilTag]:
        """Get list of visible tags from a position.
        
        :param robot_x: Robot X position.
        :param robot_y: Robot Y position.
        :param robot_heading: Robot heading.
        :return: List of visible AprilTags.
        """
        results = self.analyze_from_position(robot_x, robot_y, robot_heading)
        return [r.tag for r in results if r.is_visible]
    
    def get_total_quality(
        self,
        robot_x: SpatialMeasurement,
        robot_y: SpatialMeasurement,
        robot_heading: AngularMeasurement,
    ) -> float:
        """Get total detection quality score from a position.
        
        :param robot_x: Robot X position.
        :param robot_y: Robot Y position.
        :param robot_heading: Robot heading.
        :return: Sum of quality scores for all visible tags.
        """
        results = self.analyze_from_position(robot_x, robot_y, robot_heading)
        return sum(r.detection_quality for r in results)
    
    def _precompute_geometry(self):
        """Precompute float-based geometry for fast analysis."""
        # 0. Cache mount geometry
        self._fast_mount = {
            'ox': float(self.camera_mount.offset.x.to(Meter)),
            'oy': float(self.camera_mount.offset.y.to(Meter)),
            'oz': float(self.camera_mount.offset.z.to(Meter)),
            'heading_offset': float(self.camera_mount.heading.to(Radian)),
            'fov': float(self.camera.fov_horizontal.to(Radian)),
            'max_dist': float(self.camera.max_detection_distance.to(Meter)),
            'min_dist': float(self.camera.min_detection_distance.to(Meter)),
            'res_x': self.camera.resolution[0]
        }
        
        # 1. Obstacles (floats: min_x, min_y, max_x, max_y, min_z, max_z)
        self._fast_obstacles = []
        for obs in self._obstacles:
            if not obs.isVisible():
                continue
            
            # Get 2D bounds
            bounds = obs.bounds
            if hasattr(bounds, 'get_bounded_rectangle'):
                rect = bounds.get_bounded_rectangle()
                min_x = float(rect.x.to(Meter))
                min_y = float(rect.y.to(Meter))
                max_x = min_x + float(rect.width.to(Meter))
                max_y = min_y + float(rect.height.to(Meter))
            else:
                continue
                
            # Get Z range
            if isinstance(obs, Obstacle3D):
                z_min_val, z_max_val = obs.get_z_interval()
                min_z = float(z_min_val.to(Meter))
                max_z = float(z_max_val.to(Meter))
            else:
                min_z = 0.0
                max_z = 2.0
                
            self._fast_obstacles.append({
                'bounds': (min_x, min_y, max_x, max_y),
                'z_range': (min_z, max_z),
                'obj': obs
            })
            
        # 2. Tags (floats: x, y, z, normal_angle, size, id)
        self._fast_tags = []
        for tag in self._tags:
            self._fast_tags.append({
                'pos': (
                    float(tag.position.x.to(Meter)),
                    float(tag.position.y.to(Meter)),
                    float(tag.position.z.to(Meter))
                ),
                'normal_angle': float(tag.heading.to(Radian)),
                'size': float(tag.size.to(Meter)),
                'id': tag.id,
                'obj': tag
            })

    def analyze_batch_fast(
        self,
        robot_poses: List[Tuple[float, float, float]],
    ) -> List[List[Dict[str, float]]]:
        """Analyze visibility for a batch of robot poses (x, y, heading_rad).
        
        :return: List (one per pose) of Lists (one per visible tag) of dicts {'id': int, 'quality': float}
        """
        if not hasattr(self, '_fast_mount'):
            self._precompute_geometry()
            
        mount = self._fast_mount
        mx, my, mz = mount['ox'], mount['oy'], mount['oz']
        h_offset = mount['heading_offset']
        fov = mount['fov']
        max_d = mount['max_dist']
        min_d = mount['min_dist']
        res_x = mount['res_x']
        
        # Pre-calc quality constants
        dist_range = max_d - min_d
        optimal_d = min_d + dist_range * 0.3
        
        batch_results = []
        
        for rx, ry, rh in robot_poses:
            # Calculate camera world pose
            cos_h = math.cos(rh)
            sin_h = math.sin(rh)
            
            cx = rx + mx * cos_h - my * sin_h
            cy = ry + mx * sin_h + my * cos_h
            cz = mz # Robot is at z=0 usually, so cam z is mount z
            ch = rh + h_offset
            
            # Normalize camera heading
            ch = math.atan2(math.sin(ch), math.cos(ch))
            
            visible_tags = []
            
            for tag in self._fast_tags:
                tx, ty, tz = tag['pos']
                
                # Distance
                dx, dy, dz = tx - cx, ty - cy, tz - cz
                dist_sq = dx*dx + dy*dy + dz*dz
                
                if dist_sq > max_d*max_d or dist_sq < min_d*min_d:
                    continue
                
                dist = math.sqrt(dist_sq)
                
                # Angle to tag
                angle_to_tag = math.atan2(dy, dx)
                
                # 1. FOV Check
                diff = abs(angle_to_tag - ch)
                while diff > math.pi: diff = abs(diff - 2*math.pi)
                if diff > fov / 2:
                    continue
                    
                # 2. Tag Orientation
                tag_normal = tag['normal_angle']
                view_angle_diff = abs((angle_to_tag + math.pi) - tag_normal)
                while view_angle_diff > math.pi: view_angle_diff = abs(view_angle_diff - 2*math.pi)
                if view_angle_diff > math.pi / 2:
                    continue
                    
                # 3. Ray Cast
                if self._cast_ray_fast(cx, cy, cz, tx, ty, tz):
                    continue
                    
                # 4. Quality
                # Distance factor
                if dist <= optimal_d:
                    d_factor = 0.9 + 0.1 * (dist - min_d) / (optimal_d - min_d)
                else:
                    d_factor = 1.0 - 0.7 * (dist - optimal_d) / (max_d - optimal_d)
                    
                # Angle factor
                a_factor = max(0.0, math.cos(view_angle_diff))
                
                # Pixel factor
                pix_per_m = res_x / (2 * dist * math.tan(fov/2))
                tag_pix = tag['size'] * pix_per_m
                if tag_pix < 10: p_factor = 0.0
                elif tag_pix < 50: p_factor = (tag_pix - 10)/40
                else: p_factor = 1.0
                
                qual = d_factor * a_factor * p_factor
                
                if qual > 0:
                    visible_tags.append({'id': tag['id'], 'quality': qual})
            
            batch_results.append(visible_tags)
            
        return batch_results


    def _cast_ray_fast(
        self, 
        ox: float, oy: float, oz: float,
        tx: float, ty: float, tz: float,
        z_tolerance: float = 0.1524  # 6 inches
    ) -> Optional[Obstacle]:
        """Fast ray cast using precomputed float geometry."""
        # Ray properties
        dx = tx - ox
        dy = ty - oy
        dist_sq = dx*dx + dy*dy
        dist = math.sqrt(dist_sq)
        
        if dist < 0.001:
            return None
            
        # Check Z range relevance
        ray_min_z = min(oz, tz) - z_tolerance
        ray_max_z = max(oz, tz) + z_tolerance
        
        closest_dist = dist * 0.95  # Only care if hit is closer than target (minus margin)
        hit_obstacle = None
        
        for obs in self._fast_obstacles:
            # 1. Z-check
            z_min, z_max = obs['z_range']
            if z_max < ray_min_z or z_min > ray_max_z:
                continue
                
            # 2. Rectangle intersection
            rect_min_x, rect_min_y, rect_max_x, rect_max_y = obs['bounds']
            
            # Liang-Barsky line-rect intersection
            t0, t1 = 0.0, 1.0
            p = [-dx, dx, -dy, dy]
            q = [
                ox - rect_min_x,
                rect_max_x - ox,
                oy - rect_min_y,
                rect_max_y - oy
            ]
            
            intersect = True
            for i in range(4):
                if p[i] == 0:
                    if q[i] < 0:
                        intersect = False
                        break
                else:
                    t = q[i] / p[i]
                    if p[i] < 0:
                        if t > t1: intersect = False; break
                        if t > t0: t0 = t
                    else:
                        if t < t0: intersect = False; break
                        if t < t1: t1 = t
            
            if intersect and t0 <= t1:
                # Approximate hit distance (center of rect) or simply t0 * length?
                # Using t0 * length is better for front-face hit
                hit_dist = t0 * dist
                if hit_dist < closest_dist:
                    closest_dist = hit_dist
                    hit_obstacle = obs['obj']
                    return hit_obstacle # Optimization: return first blocking hit? 
                    # If we just need boolean visibility, first hit is enough!
                    
        return hit_obstacle

    def generate_visibility_grid(
        self,
        resolution: SpatialMeasurement = Inch(12),
        heading_samples: int = 8,
    ) -> List[VisibilityGridCell]:
        """Generate visibility heatmap with optimization."""
        # Ensure cache exists
        self._precompute_geometry()
        
        field_width, field_height = self.game.get_field_size()
        w_m = float(field_width.to(Meter))
        h_m = float(field_height.to(Meter))
        cell_size = float(resolution.to(Meter))
        
        # Camera params
        cam_z = float(self.camera_mount.offset.z.to(Meter)) # simplified for fixed height
        cam_fov = float(self.camera.fov_horizontal.to(Radian))
        max_dist = float(self.camera.max_detection_distance.to(Meter))
        min_dist = float(self.camera.min_detection_distance.to(Meter))
        resolution_x = self.camera.resolution[0]
        
        cells = []
        
        x = cell_size / 2
        while x < w_m:
            y = cell_size / 2
            while y < h_m:
                # For each cell
                for h_idx in range(heading_samples):
                    heading_rad = h_idx * 2 * math.pi / heading_samples
                    
                    # Camera pose (simplified: robot center + heading rotation)
                    # Ideally allow full mount offset logic, but for speed:
                    # cam_x = x, cam_y = y (assuming offset is small or handled)
                    # Let's do simple offset rotation if needed, but standard mount is just Z offset usually
                    cam_x = x
                    cam_y = y
                    
                    visible_count = 0
                    total_quality = 0.0
                    best_id = None
                    best_qual = -1.0
                    
                    for tag in self._fast_tags:
                        tx, ty, tz = tag['pos']
                        
                        # Distance check
                        dx, dy, dz = tx - cam_x, ty - cam_y, tz - cam_z
                        dist = math.sqrt(dx*dx + dy*dy + dz*dz)
                        
                        if dist > max_dist or dist < min_dist:
                            continue
                            
                        # Angle to tag
                        angle_to_tag = math.atan2(dy, dx)
                        
                        # 1. FOV Check
                        # angle diff
                        diff = abs(angle_to_tag - heading_rad)
                        while diff > math.pi: diff = abs(diff - 2*math.pi)
                        if diff > cam_fov / 2:
                            continue
                            
                        # 2. Tag Orientation Check
                        tag_normal = tag['normal_angle']
                        # angle from tag to camera = angle_to_tag + pi
                        view_angle_diff = abs((angle_to_tag + math.pi) - tag_normal)
                        while view_angle_diff > math.pi: view_angle_diff = abs(view_angle_diff - 2*math.pi)
                        if view_angle_diff > math.pi / 2: # Visible from front 180 (90 each side) roughly
                             # Or use tag.visible_arc logic if stored
                             continue
                             
                        # 3. Ray Cast (expensive part)
                        if self._cast_ray_fast(cam_x, cam_y, cam_z, tx, ty, tz):
                            continue # Blocked
                            
                        # 4. Quality Calc
                        # Inline quality calculation for speed
                        
                        # Distance factor
                        optimal_dist = min_dist + (max_dist - min_dist) * 0.3
                        if dist <= optimal_dist:
                            d_factor = 0.9 + 0.1 * (dist - min_dist) / (optimal_dist - min_dist)
                        else:
                            d_factor = 1.0 - 0.7 * (dist - optimal_dist) / (max_dist - optimal_dist)
                            
                        # Angle factor
                        a_factor = max(0.0, math.cos(view_angle_diff))
                        
                        # Pixel factor
                        pix_per_m = resolution_x / (2 * dist * math.tan(cam_fov/2))
                        tag_pix = tag['size'] * pix_per_m
                        if tag_pix < 10: p_factor = 0.0
                        elif tag_pix < 50: p_factor = (tag_pix - 10)/40
                        else: p_factor = 1.0
                        
                        qual = d_factor * a_factor * p_factor
                        
                        if qual > 0:
                            visible_count += 1
                            total_quality += qual
                            if qual > best_qual:
                                best_qual = qual
                                best_id = tag['obj'].id
                                
                    cells.append(VisibilityGridCell(
                        x=Meter(x),
                        y=Meter(y),
                        heading=Radian(heading_rad),
                        visible_tags=visible_count,
                        total_quality=total_quality,
                        best_tag_id=best_id
                    ))
                
                y += cell_size
            x += cell_size
            
        return cells
    
    def find_blind_spots(
        self,
        resolution: SpatialMeasurement = Inch(24),
        min_tags_required: int = 1,
    ) -> List[Tuple[SpatialMeasurement, SpatialMeasurement]]:
        """Find field positions where no tags are visible from any heading.
        
        :param resolution: Grid resolution for sampling.
        :param min_tags_required: Minimum tags that must be visible to not be a blind spot.
        :return: List of (x, y) positions that are blind spots.
        """
        grid = self.generate_visibility_grid(resolution, heading_samples=8)
        
        # Group by position
        positions: Dict[Tuple[float, float], int] = {}
        for cell in grid:
            key = (float(cell.x.to(Meter)), float(cell.y.to(Meter)))
            if key not in positions:
                positions[key] = 0
            positions[key] = max(positions[key], cell.visible_tags)
        
        blind_spots = []
        for (x, y), max_visible in positions.items():
            if max_visible < min_tags_required:
                blind_spots.append((Meter(x), Meter(y)))
        
        return blind_spots
