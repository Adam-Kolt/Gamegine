"""Tests for REBUILT 2026 scoring and match logic."""

import pytest
from gamegine.simulation.state import StateSpace
from gamegine.simulation.game import GameState
from gamegine.simulation.robot import RobotState
from gamegine.first.alliance import Alliance
from gamegine.utils.NCIM.Dimensions.spatial import Meter, Feet, Inch


class TestHubScoring:
    """Tests for Hub scoring mechanics."""
    
    def test_hub_state_initialization(self):
        """Test HubState initializes with correct defaults."""
        from examples.Rebuilt.scoring import HubState
        
        state = HubState()
        assert state.fuel_scored_active.get() == 0
        assert state.fuel_scored_inactive.get() == 0
        assert state.is_active.get() == True
        assert state.total_fuel_scored == 0
    
    def test_hub_active_scoring(self):
        """Test FUEL scored when Hub is active awards points."""
        from examples.Rebuilt.scoring import Hub, Fuel, HubState
        
        # Create game state with Hub registered
        game_state = GameState()
        game_state.createSpace("interactables")
        
        hub = Hub(
            center=(Feet(10), Feet(10)),
            navigation_point=(Feet(12), Feet(10), Inch(0)),
            alliance=Alliance.BLUE,
            name="Blue Hub",
        )
        
        # Register hub state
        hub_state = HubState()
        game_state.get("interactables").registerSpace("Blue Hub", hub_state)
        
        # Create robot state with FUEL
        game_state.createSpace("robots")
        robot_state = RobotState(Feet(10), Feet(8), Inch(0))
        robot_state.setValue("gamepieces", {Fuel: 1})
        robot_state.name = "TestBot"
        game_state.get("robots").registerSpace("TestBot", robot_state)
        
        # Get interaction and score
        interaction = hub.get_interactions()[0]  # score_fuel
        assert interaction.ableToInteract(hub_state, robot_state, game_state)
        
        # Execute scoring
        changes = interaction.interact(hub_state, robot_state, game_state)
        for change in changes:
            change.apply()
        
        # Verify points awarded
        assert hub_state.fuel_scored_active.get() == 1
        assert hub_state.fuel_scored_inactive.get() == 0
        assert game_state.blue_score.get() == 1
    
    def test_hub_inactive_tracking(self):
        """Test FUEL scored when Hub inactive tracks for RP but no points."""
        from examples.Rebuilt.scoring import Hub, Fuel, HubState
        
        game_state = GameState()
        game_state.createSpace("interactables")
        
        hub = Hub(
            center=(Feet(10), Feet(10)),
            navigation_point=(Feet(12), Feet(10), Inch(0)),
            alliance=Alliance.BLUE,
            name="Blue Hub",
        )
        
        hub_state = HubState()
        hub_state.set_active(False)  # Hub is INACTIVE
        game_state.get("interactables").registerSpace("Blue Hub", hub_state)
        
        game_state.createSpace("robots")
        robot_state = RobotState(Feet(10), Feet(8), Inch(0))
        robot_state.setValue("gamepieces", {Fuel: 1})
        robot_state.name = "TestBot"
        game_state.get("robots").registerSpace("TestBot", robot_state)
        
        interaction = hub.get_interactions()[0]
        changes = interaction.interact(hub_state, robot_state, game_state)
        for change in changes:
            change.apply()
        
        # Verify: tracked for RP, but no points
        assert hub_state.fuel_scored_active.get() == 0
        assert hub_state.fuel_scored_inactive.get() == 1
        assert hub_state.total_fuel_scored == 1  # Still counts
        assert game_state.blue_score.get() == 0  # No points


class TestTowerClimbing:
    """Tests for Tower climbing mechanics."""
    
    def test_tower_level_1_auto(self):
        """Test Level 1 climb awards 15 pts in AUTO."""
        from examples.Rebuilt.scoring import Tower, TowerState
        
        game_state = GameState()
        game_state.auto_time.set(20)
        game_state.current_time.set(10)  # In AUTO
        game_state.createSpace("interactables")
        
        tower = Tower(
            center=(Feet(5), Feet(10)),
            navigation_point=(Feet(7), Feet(10), Inch(0)),
            alliance=Alliance.BLUE,
            name="Blue Tower",
        )
        
        tower_state = TowerState()
        game_state.get("interactables").registerSpace("Blue Tower", tower_state)
        
        game_state.createSpace("robots")
        robot_state = RobotState(Feet(5), Feet(8), Inch(0))
        robot_state.name = "TestBot"
        game_state.get("robots").registerSpace("TestBot", robot_state)
        
        # Climb level 1
        interaction = tower.get_interactions()[0]  # climb_level_1
        assert interaction.ableToInteract(tower_state, robot_state, game_state)
        
        changes = interaction.interact(tower_state, robot_state, game_state)
        for change in changes:
            change.apply()
        
        assert tower_state.tower_score.get() == 15
        assert tower_state.level_1_count.get() == 1
        assert game_state.blue_score.get() == 15
    
    def test_tower_level_3_teleop(self):
        """Test Level 3 climb awards 30 pts in TELEOP."""
        from examples.Rebuilt.scoring import Tower, TowerState
        
        game_state = GameState()
        game_state.auto_time.set(20)
        game_state.current_time.set(100)  # In TELEOP
        game_state.createSpace("interactables")
        
        tower = Tower(
            center=(Feet(5), Feet(10)),
            navigation_point=(Feet(7), Feet(10), Inch(0)),
            alliance=Alliance.RED,
            name="Red Tower",
        )
        
        tower_state = TowerState()
        game_state.get("interactables").registerSpace("Red Tower", tower_state)
        
        game_state.createSpace("robots")
        robot_state = RobotState(Feet(5), Feet(8), Inch(0))
        robot_state.name = "TestBot"
        game_state.get("robots").registerSpace("TestBot", robot_state)
        
        # Climb level 3
        interaction = tower.get_interactions()[2]  # climb_level_3
        changes = interaction.interact(tower_state, robot_state, game_state)
        for change in changes:
            change.apply()
        
        assert tower_state.tower_score.get() == 30
        assert tower_state.level_3_count.get() == 1
        assert game_state.red_score.get() == 30


class TestMatchPeriods:
    """Tests for match period transitions."""
    
    def test_get_match_period(self):
        """Test period detection based on time."""
        from examples.Rebuilt.match_logic import get_match_period, MatchPeriod
        
        assert get_match_period(10) == MatchPeriod.AUTO
        assert get_match_period(25) == MatchPeriod.TRANSITION
        assert get_match_period(40) == MatchPeriod.SHIFT_1
        assert get_match_period(70) == MatchPeriod.SHIFT_2
        assert get_match_period(95) == MatchPeriod.SHIFT_3
        assert get_match_period(120) == MatchPeriod.SHIFT_4
        assert get_match_period(150) == MatchPeriod.ENDGAME
    
    def test_hub_activation_shift_1(self):
        """Test Hub activation states in SHIFT 1."""
        from examples.Rebuilt.match_logic import get_hub_active_states, MatchPeriod
        
        # Red won AUTO -> Red inactive in SHIFT 1
        red_active, blue_active = get_hub_active_states(MatchPeriod.SHIFT_1, "red")
        assert red_active == False
        assert blue_active == True
        
        # Blue won AUTO -> Blue inactive in SHIFT 1
        red_active2, blue_active2 = get_hub_active_states(MatchPeriod.SHIFT_1, "blue")
        assert red_active2 == True
        assert blue_active2 == False
    
    def test_hub_activation_alternates(self):
        """Test Hub states alternate through shifts."""
        from examples.Rebuilt.match_logic import get_hub_active_states, MatchPeriod
        
        # Red won AUTO
        s1_red, s1_blue = get_hub_active_states(MatchPeriod.SHIFT_1, "red")
        s2_red, s2_blue = get_hub_active_states(MatchPeriod.SHIFT_2, "red")
        s3_red, s3_blue = get_hub_active_states(MatchPeriod.SHIFT_3, "red")
        s4_red, s4_blue = get_hub_active_states(MatchPeriod.SHIFT_4, "red")
        
        # States should alternate
        assert (s1_red, s1_blue) == (False, True)  # SHIFT 1
        assert (s2_red, s2_blue) == (True, False)  # SHIFT 2
        assert (s3_red, s3_blue) == (False, True)  # SHIFT 3
        assert (s4_red, s4_blue) == (True, False)  # SHIFT 4


class TestRankingPoints:
    """Tests for ranking point thresholds."""
    
    def test_energized_rp_threshold(self):
        """Test ENERGIZED RP at 100 FUEL."""
        from examples.Rebuilt.match_logic import check_energized_rp, RPThresholds
        from examples.Rebuilt.scoring import HubState
        
        game_state = GameState()
        game_state.createSpace("interactables")
        
        hub_state = HubState()
        hub_state.setValue("fuel_scored_active", 80)
        hub_state.setValue("fuel_scored_inactive", 19)
        game_state.get("interactables").registerSpace("Blue Hub", hub_state)
        
        # 99 FUEL - not enough
        assert check_energized_rp(game_state, Alliance.BLUE) == False
        
        # Add 1 more
        hub_state.setValue("fuel_scored_inactive", 20)
        assert check_energized_rp(game_state, Alliance.BLUE) == True
    
    def test_supercharged_rp_threshold(self):
        """Test SUPERCHARGED RP at 360 FUEL."""
        from examples.Rebuilt.match_logic import check_supercharged_rp
        from examples.Rebuilt.scoring import HubState
        
        game_state = GameState()
        game_state.createSpace("interactables")
        
        hub_state = HubState()
        hub_state.setValue("fuel_scored_active", 300)
        hub_state.setValue("fuel_scored_inactive", 59)
        game_state.get("interactables").registerSpace("Red Hub", hub_state)
        
        assert check_supercharged_rp(game_state, Alliance.RED) == False
        
        hub_state.setValue("fuel_scored_inactive", 60)
        assert check_supercharged_rp(game_state, Alliance.RED) == True
    
    def test_traversal_rp_threshold(self):
        """Test TRAVERSAL RP at 50 TOWER points."""
        from examples.Rebuilt.match_logic import check_traversal_rp
        from examples.Rebuilt.scoring import TowerState
        
        game_state = GameState()
        game_state.createSpace("interactables")
        
        tower_state = TowerState()
        tower_state.setValue("tower_score", 49)
        game_state.get("interactables").registerSpace("Blue Tower", tower_state)
        
        assert check_traversal_rp(game_state, Alliance.BLUE) == False
        
        tower_state.setValue("tower_score", 50)
        assert check_traversal_rp(game_state, Alliance.BLUE) == True


class TestGameCreation:
    """Tests for game creation and field setup."""
    
    def test_create_rebuilt_game(self):
        """Test full game creation."""
        from examples.Rebuilt.Rebuilt import create_rebuilt_game, ALL_PIECES
        
        game = create_rebuilt_game()
        
        assert game.name == "Rebuilt 2026"
        
        # Check interactables
        interactables = list(game.get_interactables())
        assert len(interactables) == 6  # 2 Hubs, 2 Towers, 2 Depots
        
        names = [i.name for i in interactables]
        assert "Blue Hub" in names
        assert "Red Hub" in names
        assert "Blue Tower" in names
        assert "Red Tower" in names
        
    def test_fuel_positions(self):
        """Test FUEL spawn positions are generated."""
        from examples.Rebuilt.Rebuilt import ALL_PIECES, NEUTRAL_ZONE_PIECES
        
        assert len(ALL_PIECES) > 200  # Should have plenty of pieces
        assert len(NEUTRAL_ZONE_PIECES) > 100  # Neutral zone has most
