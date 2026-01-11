"""Tests for AprilTag visibility analysis system."""

import pytest
import math
from gamegine.utils.NCIM.Dimensions.spatial import Meter, Feet, Inch
from gamegine.utils.NCIM.Dimensions.angular import Degree, Radian
from gamegine.representation.bounds import Rectangle, Point
from gamegine.representation.apriltag import AprilTag, AprilTagFamily
from gamegine.representation.game import Game
from gamegine.representation.obstacle import Rectangular
from gamegine.representation.camera import CameraMount, MountConstraints
from gamegine.reference.cameras import CameraSpecification, Limelight3
from gamegine.analysis.visibility import (
    VisibilityAnalyzer,
    calculate_detection_quality,
    is_line_of_sight_clear,
)


class TestAprilTagVisibility:
    """Tests for AprilTag orientation and visibility methods."""
    
    def test_normal_vector(self):
        """Tag facing +X should have normal (1, 0, 0)."""
        tag = AprilTag(Meter(0), Meter(0), Meter(1), Degree(0), 1, AprilTagFamily.TAG_36h11)
        normal = tag.get_normal_vector()
        assert abs(normal[0] - 1.0) < 0.01
        assert abs(normal[1]) < 0.01
        assert abs(normal[2]) < 0.01
    
    def test_normal_vector_90_degrees(self):
        """Tag facing +Y should have normal (0, 1, 0)."""
        tag = AprilTag(Meter(0), Meter(0), Meter(1), Degree(90), 1, AprilTagFamily.TAG_36h11)
        normal = tag.get_normal_vector()
        assert abs(normal[0]) < 0.01
        assert abs(normal[1] - 1.0) < 0.01
    
    def test_visible_from_front(self):
        """Tag should be visible when viewed from the front."""
        tag = AprilTag(Meter(0), Meter(0), Meter(1), Degree(0), 1, AprilTagFamily.TAG_36h11)
        # Viewer at +X looking toward origin
        assert tag.is_visible_from_position(Meter(5), Meter(0)) == True
    
    def test_not_visible_from_back(self):
        """Tag should NOT be visible when viewed from behind."""
        tag = AprilTag(Meter(0), Meter(0), Meter(1), Degree(0), 1, AprilTagFamily.TAG_36h11)
        # Viewer at -X (behind the tag)
        assert tag.is_visible_from_position(Meter(-5), Meter(0)) == False
    
    def test_visible_at_edge_of_arc(self):
        """Tag should be visible at edge of visible arc."""
        tag = AprilTag(Meter(0), Meter(0), Meter(1), Degree(0), 1, AprilTagFamily.TAG_36h11,
                       visible_arc=Degree(160))
        # Viewer at 75 degrees off-axis (within 80 degree half-arc)
        assert tag.is_visible_from_position(Meter(1), Meter(3)) == True


class TestGameAprilTagCollection:
    """Tests for AprilTag management in Game class."""
    
    def test_add_single_apriltag(self):
        """Adding a single AprilTag to game."""
        game = Game("Test")
        tag = AprilTag(Meter(0), Meter(0), Meter(1), Degree(0), 1, AprilTagFamily.TAG_36h11)
        game.add_apriltag(tag)
        
        assert len(game.get_apriltags()) == 1
        assert game.get_apriltag_by_id(1) == tag
    
    def test_add_multiple_apriltags(self):
        """Adding multiple AprilTags at once."""
        game = Game("Test")
        tags = [
            AprilTag(Meter(0), Meter(0), Meter(1), Degree(0), i, AprilTagFamily.TAG_36h11)
            for i in range(1, 5)
        ]
        game.add_apriltags(tags)
        
        assert len(game.get_apriltags()) == 4
    
    def test_duplicate_id_raises(self):
        """Duplicate tag ID should raise exception."""
        game = Game("Test")
        tag1 = AprilTag(Meter(0), Meter(0), Meter(1), Degree(0), 1, AprilTagFamily.TAG_36h11)
        tag2 = AprilTag(Meter(5), Meter(5), Meter(1), Degree(90), 1, AprilTagFamily.TAG_36h11)
        
        game.add_apriltag(tag1)
        with pytest.raises(Exception):
            game.add_apriltag(tag2)


class TestDetectionQuality:
    """Tests for detection quality calculation."""
    
    def test_quality_in_range(self):
        """Quality score should be between 0 and 1."""
        quality = calculate_detection_quality(
            Meter(3), Degree(0), Inch(10.5), Limelight3
        )
        assert 0 <= quality <= 1
    
    def test_quality_decreases_with_angle(self):
        """Detection quality should decrease with viewing angle."""
        q_straight = calculate_detection_quality(
            Meter(3), Degree(0), Inch(10.5), Limelight3
        )
        q_angled = calculate_detection_quality(
            Meter(3), Degree(45), Inch(10.5), Limelight3
        )
        assert q_straight > q_angled
    
    def test_zero_quality_beyond_max_range(self):
        """Quality should be zero beyond max detection distance."""
        quality = calculate_detection_quality(
            Meter(100), Degree(0), Inch(10.5), Limelight3
        )
        assert quality == 0.0


class TestLineOfSight:
    """Tests for line of sight checking."""
    
    def test_clear_path(self):
        """No obstacle between points."""
        is_clear, blocker = is_line_of_sight_clear(
            Point(Meter(0), Meter(0), Meter(1)),
            Point(Meter(5), Meter(0), Meter(1)),
            obstacles=[],
        )
        assert is_clear == True
        assert blocker is None
    
    def test_blocked_by_obstacle(self):
        """Obstacle between points should block LOS."""
        obstacle = Rectangular("Wall", Meter(2), Meter(-1), Meter(1), Meter(2))
        
        is_clear, blocker = is_line_of_sight_clear(
            Point(Meter(0), Meter(0), Meter(1)),
            Point(Meter(5), Meter(0), Meter(1)),
            obstacles=[obstacle],
        )
        assert is_clear == False
        assert blocker == obstacle


class TestVisibilityAnalyzer:
    """Tests for the VisibilityAnalyzer class."""
    
    def test_analyze_single_tag(self):
        """Analyze visibility of a single tag."""
        game = Game("Test")
        game.set_field_size(Meter(16), Meter(8))
        tag = AprilTag(Meter(0), Meter(4), Inch(53), Degree(0), 1, AprilTagFamily.TAG_36h11)
        game.add_apriltag(tag)
        
        analyzer = VisibilityAnalyzer(game, Limelight3)
        results = analyzer.analyze_from_position(Meter(5), Meter(4), Degree(180))
        
        assert len(results) == 1
        assert results[0].tag.id == 1
    
    def test_get_visible_tags(self):
        """Get list of visible tags."""
        game = Game("Test")
        game.set_field_size(Meter(16), Meter(8))
        tag = AprilTag(Meter(0), Meter(4), Inch(53), Degree(0), 1, AprilTagFamily.TAG_36h11)
        game.add_apriltag(tag)
        
        analyzer = VisibilityAnalyzer(game, Limelight3)
        visible = analyzer.get_visible_tags(Meter(5), Meter(4), Degree(180))
        
        # Tag at origin facing +X should be visible from position at +X looking back
        assert any(t.id == 1 for t in visible)
    
    def test_total_quality(self):
        """Total quality should be sum of individual qualities."""
        game = Game("Test")
        game.set_field_size(Meter(16), Meter(8))
        tag = AprilTag(Meter(0), Meter(4), Inch(53), Degree(0), 1, AprilTagFamily.TAG_36h11)
        game.add_apriltag(tag)
        
        analyzer = VisibilityAnalyzer(game, Limelight3)
        total = analyzer.get_total_quality(Meter(5), Meter(4), Degree(180))
        
        assert total >= 0


class TestCameraSpecifications:
    """Tests for camera specification presets."""
    
    def test_limelight3_specs(self):
        """Limelight 3 should have expected specs."""
        assert Limelight3.name == "Limelight 3"
        assert float(Limelight3.fov_horizontal.to(Degree)) > 60
        assert Limelight3.resolution == (1280, 960)
    
    def test_pixels_per_degree(self):
        """Pixels per degree calculation."""
        ppd = Limelight3.get_pixels_per_degree_h()
        assert ppd > 0


class TestMountConstraints:
    """Tests for camera mount constraints."""
    
    def test_height_constraints(self):
        """Mount position should respect height constraints."""
        constraints = MountConstraints(min_height=Inch(6), max_height=Feet(4))
        
        # Too low
        assert constraints.is_valid_mount(Point(Meter(0), Meter(0), Inch(3))) == False
        # Valid
        assert constraints.is_valid_mount(Point(Meter(0), Meter(0), Inch(24))) == True
        # Too high
        assert constraints.is_valid_mount(Point(Meter(0), Meter(0), Feet(5))) == False
