"""Camera position optimization for AprilTag visibility.

This module provides tools to find optimal camera mounting positions
on a robot to maximize AprilTag visibility across the field.
"""

from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple
import math
import random

from gamegine.representation.bounds import Point, Boundary3D
from gamegine.representation.game import Game
from gamegine.representation.camera import CameraMount, MountConstraints
from gamegine.reference.cameras import CameraSpecification
from gamegine.analysis.visibility import VisibilityAnalyzer, VisibilityGridCell
from gamegine.utils.NCIM.Dimensions.angular import AngularMeasurement, Degree
from gamegine.utils.NCIM.Dimensions.spatial import SpatialMeasurement, Meter, Inch, Feet


@dataclass
class OptimizationObjective:
    """Configurable optimization targets with custom weighting.
    
    All weights should be non-negative. Higher weight means more importance.
    
    :param maximize_tags_visible: Weight for maximizing number of visible tags.
    :param minimize_blind_spots: Weight for minimizing blind spot regions.
    :param strategic_tag_weights: Dict of tag ID to importance weight (optional).
    """
    maximize_tags_visible: float = 1.0
    minimize_blind_spots: float = 1.0
    strategic_tag_weights: Optional[Dict[int, float]] = None
    
    def get_tag_weight(self, tag_id: int) -> float:
        """Get the importance weight for a specific tag.
        
        :param tag_id: The tag ID to get weight for.
        :return: Weight value (default 1.0 if not specified).
        """
        if self.strategic_tag_weights and tag_id in self.strategic_tag_weights:
            return self.strategic_tag_weights[tag_id]
        return 1.0


@dataclass
class CameraPlacementScore:
    """Scoring result for a camera placement.
    
    :param mount: The camera mount configuration being scored.
    :param total_score: Weighted total score (higher is better).
    :param tags_visible_avg: Average number of tags visible across field.
    :param coverage_percent: Percentage of field positions with tag visibility.
    :param weighted_quality: Quality score weighted by tag importance.
    :param per_tag_visibility: Dict of tag ID to visibility percentage.
    """
    mount: CameraMount
    total_score: float
    tags_visible_avg: float
    coverage_percent: float
    weighted_quality: float
    per_tag_visibility: Dict[int, float] = field(default_factory=dict)


class CameraPositionOptimizer:
    """Optimizes camera mounting positions with customizable objectives.
    
    This class searches the valid mounting region for camera positions
    that maximize visibility according to the specified objectives.
    
    :param game: The Game containing AprilTags and obstacles.
    :param camera: The camera specification to optimize for.
    :param mount_constraints: Constraints on valid mounting positions.
    :param objective: Optimization objectives (default: equal weight to all).
    """
    
    def __init__(
        self,
        game: Game,
        camera: CameraSpecification,
        mount_constraints: Optional[MountConstraints] = None,
        objective: Optional[OptimizationObjective] = None,
    ):
        self.game = game
        self.camera = camera
        self.constraints = mount_constraints or MountConstraints()
        self.objective = objective or OptimizationObjective()
        
        # Default robot dimensions for sampling if no regions specified
        self._default_robot_half_width = Inch(15)
        self._default_robot_half_length = Inch(15)
    
    def _generate_candidate_mounts(
        self,
        num_candidates: int = 50,
    ) -> List[CameraMount]:
        """Generate candidate camera mount positions to evaluate.
        
        :param num_candidates: Number of random candidates to generate.
        :return: List of candidate CameraMount configurations.
        """
        candidates = []
        
        # Define search bounds
        min_height = float(self.constraints.min_height.to(Meter))
        max_height = float(self.constraints.max_height.to(Meter))
        
        # Sample positions on robot perimeter + some interior positions
        half_w = float(self._default_robot_half_width.to(Meter))
        half_l = float(self._default_robot_half_length.to(Meter))
        
        # Strategic positions: corners and edges
        strategic_offsets = [
            # Front
            (half_l, 0),
            (half_l, half_w * 0.5),
            (half_l, -half_w * 0.5),
            # Rear
            (-half_l, 0),
            (-half_l, half_w * 0.5),
            (-half_l, -half_w * 0.5),
            # Sides
            (0, half_w),
            (0, -half_w),
            # Center (turret position)
            (0, 0),
        ]
        
        # Strategic headings for each position
        headings = [0, 45, 90, 135, 180, -45, -90, -135]
        
        # Heights to try
        heights = [
            min_height + (max_height - min_height) * 0.25,
            min_height + (max_height - min_height) * 0.5,
            min_height + (max_height - min_height) * 0.75,
        ]
        
        for ox, oy in strategic_offsets:
            for h_deg in headings:
                for z in heights:
                    offset = Point(Meter(ox), Meter(oy), Meter(z))
                    if self.constraints.is_valid_mount(offset):
                        candidates.append(CameraMount(
                            camera=self.camera,
                            offset=offset,
                            heading=Degree(h_deg),
                        ))
        
        # Add random candidates to fill
        while len(candidates) < num_candidates:
            ox = random.uniform(-half_l, half_l)
            oy = random.uniform(-half_w, half_w)
            z = random.uniform(min_height, max_height)
            h = random.uniform(-180, 180)
            
            offset = Point(Meter(ox), Meter(oy), Meter(z))
            if self.constraints.is_valid_mount(offset):
                candidates.append(CameraMount(
                    camera=self.camera,
                    offset=offset,
                    heading=Degree(h),
                ))
        
        return candidates[:num_candidates]
    
    def evaluate_placement(
        self,
        mount: CameraMount,
        sample_positions: Optional[List[Tuple[SpatialMeasurement, SpatialMeasurement]]] = None,
        heading_samples: int = 8,
    ) -> CameraPlacementScore:
        """Score a single camera placement across sample field positions.
        
        :param mount: The camera mount to evaluate.
        :param sample_positions: List of (x, y) positions to sample. If None, uses a grid.
        :param heading_samples: Number of headings to sample per position.
        :return: CameraPlacementScore with results.
        """
        analyzer = VisibilityAnalyzer(self.game, self.camera, mount)
        
        # Determine query poses (x, y, heading) in floats
        poses: List[Tuple[float, float, float]] = []
        
        # 1. Get X/Y positions
        if sample_positions is None:
            field_w, field_h = self.game.get_field_size()
            fw_m = float(field_w.to(Meter))
            fh_m = float(field_h.to(Meter))
            step_m = 1.5
            
            x_range = []
            x = step_m
            while x < fw_m:
                x_range.append(x)
                x += step_m
                
            y_range = []
            y = step_m
            while y < fh_m:
                y_range.append(y)
                y += step_m
                
            xy_positions = [(x, y) for x in x_range for y in y_range]
        else:
            xy_positions = [
                (float(p[0].to(Meter)), float(p[1].to(Meter))) 
                for p in sample_positions
            ]
            
        # 2. Add headings for each position
        headings = [
            i * 2 * math.pi / heading_samples 
            for i in range(heading_samples)
        ]
        
        poses = [
            (x, y, h) 
            for x, y in xy_positions 
            for h in headings
        ]
        
        # 3. Fast Batch Analysis
        batch_results = analyzer.analyze_batch_fast(poses)
        
        # 4. Aggregate Scores
        tag_visible_count: Dict[int, int] = {}
        for tag in self.game.get_apriltags():
            tag_visible_count[tag.id] = 0
            
        total_visible_tags = 0
        total_quality = 0.0
        positions_with_visibility = 0
        total_samples = len(poses)
        
        # batch_results corresponds 1:1 with poses
        for visible_tags in batch_results:
            if visible_tags:
                positions_with_visibility += 1
                total_visible_tags += len(visible_tags)
                
                for r in visible_tags:
                    tag_id = r['id']
                    qual = r['quality']
                    
                    w_qual = qual * self.objective.get_tag_weight(tag_id)
                    total_quality += w_qual
                    tag_visible_count[tag_id] += 1
        
        # Compute scores
        avg_visible = total_visible_tags / total_samples if total_samples > 0 else 0
        coverage = positions_with_visibility / total_samples if total_samples > 0 else 0
        
        per_tag_visibility = {
            tag_id: count / total_samples
            for tag_id, count in tag_visible_count.items()
        }
        
        # Combined score
        total_score = (
            self.objective.maximize_tags_visible * avg_visible +
            self.objective.minimize_blind_spots * coverage * 10 +  # Scale coverage
            total_quality / max(total_samples, 1)
        )
        
        return CameraPlacementScore(
            mount=mount,
            total_score=total_score,
            tags_visible_avg=avg_visible,
            coverage_percent=coverage * 100,
            weighted_quality=total_quality / max(total_samples, 1),
            per_tag_visibility=per_tag_visibility,
        )
    
    def optimize(
        self,
        sample_positions: Optional[List[Tuple[SpatialMeasurement, SpatialMeasurement]]] = None,
        num_results: int = 5,
        num_candidates: int = 50,
    ) -> List[CameraPlacementScore]:
        """Find optimal camera mounting positions.
        
        :param sample_positions: Field positions to sample (None for default grid).
        :param num_results: Number of top results to return.
        :param num_candidates: Number of candidate positions to evaluate.
        :return: List of top CameraPlacementScore sorted by score (best first).
        """
        candidates = self._generate_candidate_mounts(num_candidates)
        
        scores = []
        for mount in candidates:
            score = self.evaluate_placement(mount, sample_positions)
            scores.append(score)
        
        # Sort by total score descending
        scores.sort(key=lambda s: s.total_score, reverse=True)
        
        return scores[:num_results]
    
    def optimize_multi_camera(
        self,
        num_cameras: int = 2,
        sample_positions: Optional[List[Tuple[SpatialMeasurement, SpatialMeasurement]]] = None,
        num_candidates_per: int = 30,
    ) -> List[Tuple[List[CameraMount], float]]:
        """Optimize placement for multiple cameras.
        
        Uses a greedy approach: optimize first camera, then find complementary
        positions for additional cameras.
        
        :param num_cameras: Number of cameras to place.
        :param sample_positions: Field positions to sample.
        :param num_candidates_per: Candidates to evaluate per camera.
        :return: List of (camera_list, combined_score) tuples.
        """
        # Get top candidates for first camera
        first_results = self.optimize(sample_positions, num_results=3, num_candidates=num_candidates_per)
        
        multi_results = []
        
        for first_score in first_results:
            cameras = [first_score.mount]
            combined_score = first_score.total_score
            
            for cam_idx in range(1, num_cameras):
                # For additional cameras, look for complementary positions
                # This is simplified - a full implementation would track combined coverage
                candidates = self._generate_candidate_mounts(num_candidates_per)
                
                best_additional = None
                best_additional_score = 0
                
                for mount in candidates:
                    # Skip if too close to existing cameras
                    too_close = False
                    for existing in cameras:
                        dist = math.sqrt(
                            (float(mount.offset.x.to(Meter)) - float(existing.offset.x.to(Meter))) ** 2 +
                            (float(mount.offset.y.to(Meter)) - float(existing.offset.y.to(Meter))) ** 2
                        )
                        if dist < 0.1:  # 10cm minimum separation
                            too_close = True
                            break
                    
                    if too_close:
                        continue
                    
                    score = self.evaluate_placement(mount, sample_positions)
                    if score.total_score > best_additional_score:
                        best_additional = mount
                        best_additional_score = score.total_score
                
                if best_additional:
                    cameras.append(best_additional)
                    combined_score += best_additional_score
            
            multi_results.append((cameras, combined_score))
        
        # Sort by combined score
        multi_results.sort(key=lambda x: x[1], reverse=True)
        
        return multi_results
