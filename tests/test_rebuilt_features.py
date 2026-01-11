"""Tests for Rebuilt game integration features."""

import pytest
from gamegine.utils.NCIM.Dimensions.spatial import Meter, Feet, Inch
from gamegine.representation.bounds import Rectangle


class TestTraversalZone:
    """Tests for TraversalZone speed modifiers."""
    
    def test_zone_creation(self):
        """Test basic zone creation."""
        from gamegine.representation.zone import TraversalZone
        
        boundary = Rectangle(Meter(0), Meter(0), Meter(5), Meter(5))
        zone = TraversalZone(
            name="ramp",
            boundary=boundary,
            speed_multiplier=0.5
        )
        
        assert zone.name == "ramp"
        assert zone.speed_multiplier == 0.5
    
    def test_zone_contains_point(self):
        """Test point containment check."""
        from gamegine.representation.zone import TraversalZone
        
        boundary = Rectangle(Meter(0), Meter(0), Meter(5), Meter(5))
        zone = TraversalZone(name="test", boundary=boundary, speed_multiplier=0.5)
        
        # Point inside
        assert zone.contains_point(Meter(2.5), Meter(2.5)) == True
        # Point outside  
        assert zone.contains_point(Meter(10), Meter(10)) == False
    
    def test_weight_multiplier_calculation(self):
        """Test pathfinding weight multiplier is inverse of speed."""
        from gamegine.representation.zone import TraversalZone
        
        boundary = Rectangle(Meter(0), Meter(0), Meter(1), Meter(1))
        
        # 0.5x speed = 2x weight (takes twice as long)
        zone_slow = TraversalZone(name="slow", boundary=boundary, speed_multiplier=0.5)
        assert zone_slow.get_weight_multiplier() == pytest.approx(2.0)
        
        # 1.0x speed = 1x weight (normal)
        zone_normal = TraversalZone(name="normal", boundary=boundary, speed_multiplier=1.0)
        assert zone_normal.get_weight_multiplier() == pytest.approx(1.0)


class TestObstacle3D:
    """Tests for height-based obstacle filtering."""
    
    def test_obstacle3d_creation(self):
        """Test Obstacle3D with height interval."""
        from gamegine.representation.obstacle import Obstacle3D
        
        bounds = Rectangle(Meter(0), Meter(0), Meter(2), Meter(1))
        obstacle = Obstacle3D(
            name="bar",
            bounds_2d=bounds,
            z_min=Feet(3),
            z_max=Feet(4)
        )
        
        assert obstacle.name == "bar"
        z_min, z_max = obstacle.get_z_interval()
        assert z_min == Feet(3)
        assert z_max == Feet(4)
    
    def test_applies_to_tall_robot(self):
        """Tall robots should be blocked by low obstacles."""
        from gamegine.representation.obstacle import Obstacle3D
        
        bounds = Rectangle(Meter(0), Meter(0), Meter(2), Meter(1))
        # Bar with clearance at 3 feet
        bar = Obstacle3D(name="bar", bounds_2d=bounds, z_min=Feet(3), z_max=Feet(4))
        
        # Robot taller than clearance - blocked
        assert bar.applies_to_robot(Feet(4)) == True
        
    def test_does_not_apply_to_short_robot(self):
        """Short robots should pass under obstacles."""
        from gamegine.representation.obstacle import Obstacle3D
        
        bounds = Rectangle(Meter(0), Meter(0), Meter(2), Meter(1))
        bar = Obstacle3D(name="bar", bounds_2d=bounds, z_min=Feet(3), z_max=Feet(4))
        
        # Robot shorter than clearance - passes under
        assert bar.applies_to_robot(Feet(2)) == False


class TestGamepieceManager:
    """Tests for gamepiece management and trajectory pickup."""
    
    def test_spawn_and_pickup(self):
        """Test spawning and picking up pieces."""
        from gamegine.simulation.gamepiece import GamepieceManager
        
        manager = GamepieceManager()
        piece_id = manager.spawn_piece(Meter(5), Meter(5))
        
        assert piece_id in manager.get_available_pieces()
        
        # Pickup should succeed
        assert manager.pickup_piece(piece_id, "robot_1") == True
        
        # Piece no longer on field
        assert piece_id not in manager.get_available_pieces()
        
        # Piece is held by robot
        assert piece_id in manager.get_robot_pieces("robot_1")
    
    def test_cannot_pickup_held_piece(self):
        """Cannot pick up a piece already held by another robot."""
        from gamegine.simulation.gamepiece import GamepieceManager
        
        manager = GamepieceManager()
        piece_id = manager.spawn_piece(Meter(5), Meter(5))
        
        manager.pickup_piece(piece_id, "robot_1")
        
        # Second robot cannot pick up same piece
        assert manager.pickup_piece(piece_id, "robot_2") == False
    
    def test_drop_piece(self):
        """Test dropping a piece back on field."""
        from gamegine.simulation.gamepiece import GamepieceManager
        
        manager = GamepieceManager()
        piece_id = manager.spawn_piece(Meter(5), Meter(5))
        manager.pickup_piece(piece_id, "robot_1")
        
        # Drop the piece
        assert manager.drop_piece(piece_id, Meter(10), Meter(10)) == True
        
        # Piece is back on field
        assert piece_id in manager.get_available_pieces()
    
    def test_pieces_near_point(self):
        """Test finding pieces within radius."""
        from gamegine.simulation.gamepiece import GamepieceManager
        
        manager = GamepieceManager()
        # Spawn pieces at various locations
        p1 = manager.spawn_piece(Meter(0), Meter(0))  # At origin
        p2 = manager.spawn_piece(Meter(1), Meter(0))  # 1m away
        p3 = manager.spawn_piece(Meter(5), Meter(0))  # 5m away
        
        # Search with 2m radius - should find p1 and p2
        nearby = manager.get_pieces_near_point(Meter(0), Meter(0), Meter(2))
        assert p1 in nearby
        assert p2 in nearby
        assert p3 not in nearby


class TestEnhancedGamepiecePhysics:
    """Tests for enhanced intake, pushing, and shooting mechanics."""
    
    def test_intake_state_tracking(self):
        """Test intake active state."""
        from gamegine.simulation.gamepiece import GamepieceManager
        
        manager = GamepieceManager()
        
        # Default: intake is not active
        assert manager.is_intake_active("robot_1") == False
        
        # Activate intake
        manager.set_intake_active("robot_1", True)
        assert manager.is_intake_active("robot_1") == True
        
        # Deactivate
        manager.set_intake_active("robot_1", False)
        assert manager.is_intake_active("robot_1") == False
    
    def test_intake_side_detection_front(self):
        """Test that front intake only picks up pieces in front."""
        from gamegine.simulation.gamepiece import GamepieceManager, IntakeConfig
        from gamegine.utils.NCIM.Dimensions.angular import Degree
        
        manager = GamepieceManager()
        intake = IntakeConfig(
            sides=["front"],
            pickup_radius=Meter(1),
            capacity=3
        )
        
        # Robot at origin, facing +X (heading = 0)
        robot_x, robot_y = Meter(0), Meter(0)
        robot_heading = Degree(0)
        
        # Piece in front (along +X)
        assert manager.is_piece_on_intake_side(
            Meter(1), Meter(0), robot_x, robot_y, robot_heading, intake
        ) == True
        
        # Piece behind (along -X)
        assert manager.is_piece_on_intake_side(
            Meter(-1), Meter(0), robot_x, robot_y, robot_heading, intake
        ) == False
        
        # Piece to the side
        assert manager.is_piece_on_intake_side(
            Meter(0), Meter(1), robot_x, robot_y, robot_heading, intake
        ) == False
    
    def test_push_pieces(self):
        """Test pieces get pushed away from robot."""
        from gamegine.simulation.gamepiece import GamepieceManager
        
        manager = GamepieceManager()
        pid = manager.spawn_piece(Meter(0.2), Meter(0))  # Very close to origin
        
        # Push from robot at origin with 0.5m radius
        pushed = manager.push_pieces_from_robot(Meter(0), Meter(0), Meter(0.5))
        
        assert pid in pushed
        
        # Piece should now be outside robot radius
        state = manager.get_piece(pid)
        new_x = float(state.x.value.to(Meter))
        assert new_x > 0.5  # Should be pushed out
        
        # Should also have velocity
        assert state.vel_x > 0
    
    def test_shoot_piece(self):
        """Test shooting piece to target."""
        from gamegine.simulation.gamepiece import GamepieceManager
        
        manager = GamepieceManager()
        pid = manager.spawn_piece(Meter(0), Meter(0))
        manager.pickup_piece(pid, "robot_1")
        
        # Shoot at target
        result = manager.shoot_piece(
            pid,
            target_x=Meter(5),
            target_y=Meter(5),
            shooter_x=Meter(0),
            shooter_y=Meter(0),
            deterministic=True
        )
        
        # At close range with good accuracy, should succeed
        assert result.success == True
        
        # Piece should now be on field at target
        state = manager.get_piece(pid)
        assert state.is_on_field() == True
    
    def test_piece_velocity_friction(self):
        """Test piece velocity decays over time."""
        from gamegine.simulation.gamepiece import GamepieceState
        
        state = GamepieceState(Meter(0), Meter(0), vel_x=5.0, vel_y=0.0)
        
        # Update physics for 1 second
        state.update_physics(dt=1.0, friction=3.0)
        
        # Velocity should have decayed
        assert state.vel_x < 5.0
        assert state.vel_x > 0.0  # Not fully stopped yet
        
        # Position should have changed
        new_x = float(state.x.value.to(Meter))
        assert new_x > 0


class TestShootingAccuracy:
    """Tests for shooting probability model."""
    
    def test_probability_at_optimal_range(self):
        """Probability equals base accuracy at optimal range."""
        from gamegine.simulation.shooting import ShootingParameters, calculate_shot_probability
        
        params = ShootingParameters(
            base_accuracy=0.9,
            optimal_range=Meter(3),
            accuracy_falloff_per_meter=0.1,
            max_range=Meter(10)
        )
        
        shooter = (Meter(0), Meter(0))
        target = (Meter(3), Meter(0))  # Exactly at optimal range
        
        prob = calculate_shot_probability(shooter, target, params)
        assert prob == pytest.approx(0.9)
    
    def test_probability_decreases_with_distance(self):
        """Probability decreases as distance from optimal increases."""
        from gamegine.simulation.shooting import ShootingParameters, calculate_shot_probability
        
        params = ShootingParameters(
            base_accuracy=0.9,
            optimal_range=Meter(3),
            accuracy_falloff_per_meter=0.1,
            max_range=Meter(10)
        )
        
        shooter = (Meter(0), Meter(0))
        
        # At optimal range
        prob_optimal = calculate_shot_probability(shooter, (Meter(3), Meter(0)), params)
        
        # 2m further from optimal
        prob_far = calculate_shot_probability(shooter, (Meter(5), Meter(0)), params)
        
        assert prob_far < prob_optimal
        assert prob_far == pytest.approx(0.7)  # 0.9 - (2m * 0.1)
    
    def test_probability_zero_beyond_max_range(self):
        """Probability is zero beyond max range."""
        from gamegine.simulation.shooting import ShootingParameters, calculate_shot_probability
        
        params = ShootingParameters(
            base_accuracy=0.9,
            optimal_range=Meter(3),
            accuracy_falloff_per_meter=0.1,
            max_range=Meter(10)
        )
        
        shooter = (Meter(0), Meter(0))
        target = (Meter(15), Meter(0))  # Beyond max range
        
        prob = calculate_shot_probability(shooter, target, params)
        assert prob == 0.0
    
    def test_deterministic_mode(self):
        """Deterministic mode: >50% = success, <=50% = miss."""
        from gamegine.simulation.shooting import (
            ShootingParameters, attempt_shot, ShotOutcome
        )
        
        params = ShootingParameters(
            base_accuracy=0.9,
            optimal_range=Meter(3),
            accuracy_falloff_per_meter=0.1,
            max_range=Meter(10)
        )
        
        shooter = (Meter(0), Meter(0))
        
        # High probability (90%) - should succeed
        result = attempt_shot(shooter, (Meter(3), Meter(0)), params, deterministic=True)
        assert result == ShotOutcome.SUCCESS
        
        # Low probability (at 7m: 0.9 - 0.4 = 0.5, exactly 50%) - should miss
        result = attempt_shot(shooter, (Meter(7), Meter(0)), params, deterministic=True)
        assert result == ShotOutcome.MISS


class TestRobotHeight:
    """Tests for robot height calculation."""
    
    def test_default_height(self):
        """Robot without structure returns default height."""
        from gamegine.representation.robot import Robot
        
        robot = Robot(name="test")
        height = robot.get_height()
        
        # Default is 4 feet
        assert height == Feet(4)


class TestGameZones:
    """Tests for zone integration with Game class."""
    
    def test_add_zone_to_game(self):
        """Test adding zones to a game."""
        from gamegine.representation.game import Game
        from gamegine.representation.zone import TraversalZone
        
        game = Game("TestGame")
        
        boundary = Rectangle(Meter(0), Meter(0), Meter(5), Meter(5))
        zone = TraversalZone(name="ramp", boundary=boundary, speed_multiplier=0.5)
        
        game.add_zone(zone)
        
        zones = game.get_zones()
        assert len(zones) == 1
        assert zones[0].name == "ramp"
