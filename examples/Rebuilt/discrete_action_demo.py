"""Discrete Action Demo - Visual demonstration of the new zone-based action system

Showcases:
- Discrete pickup actions (1/5/10 balls) from Depot, NeutralZone, AllianceZone
- Shooting from ShootingLocations with accuracy-based outcomes
- Shuttling balls to Alliance Zone
- Defense mechanics
- Full game state visualization with scores + ball counts

Controls:
- SPACE: Execute next action in sequence
- R: Reset demo
- ESC: Exit
"""

import sys
sys.path.insert(0, '.')

from typing import List, Tuple, Optional, Dict, Any
from dataclasses import dataclass

from examples.Rebuilt.Rebuilt import create_rebuilt_game, HALF_LENGTH, HALF_WIDTH
from examples.Rebuilt.scoring import Fuel, NeutralZone, AllianceZone, Depot

from gamegine.representation.robot import SwerveRobot, PhysicalParameters
from gamegine.reference import gearing, motors
from gamegine.reference.swerve import SwerveConfig, SwerveModule
from gamegine.simulation.GameServer import DiscreteGameServer, ServerConfig
from gamegine.simulation.robot import RobotState
from gamegine.simulation.game import GameState
from gamegine.representation.interactable import RobotInteractionConfig
from gamegine.representation.bounds import Rectangle
from gamegine.utils.NCIM.Dimensions.spatial import Meter, Feet, Inch
from gamegine.utils.NCIM.Dimensions.angular import Degree
from gamegine.utils.NCIM.Dimensions.mass import Pound
from gamegine.utils.NCIM.Dimensions.temporal import Second
from gamegine.utils.NCIM.ComplexDimensions.acceleration import MeterPerSecondSquared
from gamegine.utils.NCIM.ncim import Ampere
from gamegine.first.alliance import Alliance

from gamegine.render import Renderer, DisplayLevel, AlertType
import arcade


# =============================================================================
# ROBOT SETUP
# =============================================================================

ROBOT_WIDTH = Inch(30)


def create_swerve_config():
    """Create standard swerve drivetrain config."""
    return SwerveConfig(
        SwerveModule(
            motors.MotorConfig(
                motors.KrakenX60,
                motors.PowerConfig(Ampere(60), Ampere(240), 1.0),
            ),
            gearing.MK4I.L3,
            motors.MotorConfig(
                motors.KrakenX60,
                motors.PowerConfig(Ampere(60), Ampere(240), 1.0),
            ),
            gearing.MK4I.L3,
        )
    )


def create_robot(name: str, alliance: Alliance) -> SwerveRobot:
    """Create a swerve robot for the demo."""
    structure = [
        Rectangle.from_center((Inch(0), Inch(0)), ROBOT_WIDTH, ROBOT_WIDTH).get_3d(
            Inch(0), Feet(2.5)
        )
    ]
    
    robot = SwerveRobot(
        name=name,
        drivetrain=create_swerve_config(),
        structure=structure,
        physics=PhysicalParameters(
            mass=Pound(125),
            moi=Pound(125) * Inch(15) ** 2,
            max_acceleration=MeterPerSecondSquared(4.0),
        ),
    )
    robot.override_bounding_radius(Inch(16))
    robot.alliance = alliance
    return robot


# =============================================================================
# ACTION SEQUENCE
# =============================================================================

@dataclass
class DemoAction:
    """A single action in the demo sequence."""
    interactable: str
    interaction: str
    description: str


# Predefined action routine showcasing all available actions
DEMO_ACTIONS: List[DemoAction] = [
    DemoAction("Blue Depot", "pickup_5", "Pick up 5 balls from Depot"),
    DemoAction("BLUE Near Top", "shoot", "Shoot from NEAR position (95% accuracy)"),
    DemoAction("BLUE Near Top", "shoot", "Shoot again from NEAR position"),
    DemoAction("BLUE Near Top", "shoot", "Shoot from NEAR (3rd shot)"),
    DemoAction("Blue Depot", "pickup_10", "Pick up 10 balls from Depot"),
    DemoAction("BLUE Mid Top", "shuttle", "Shuttle balls to Alliance Zone"),
    DemoAction("Blue Alliance Zone", "pickup_5", "Pick up 5 from Alliance Zone"),
    DemoAction("BLUE Far Top", "shoot", "Shoot from FAR position (60% accuracy)"),
    DemoAction("BLUE Far Top", "shoot", "Shoot from FAR (2nd shot)"),
    DemoAction("Neutral Zone", "pickup_1", "Pick up 1 from Neutral Zone (recycled balls)"),
    DemoAction("BLUE Near Bot", "shoot", "Shoot from NEAR BOT position"),
    DemoAction("Blue Depot", "pickup_1", "Pick up remaining Depot balls"),
    DemoAction("BLUE Mid Bot", "shoot", "Final shot from MID position"),
    DemoAction("Blue Tower", "climb_level_3", "Climb to Level 3 for 30 points!"),
]


# =============================================================================
# GAME STATE HUD
# =============================================================================

class GameStateHUD:
    """Draws game state information on screen."""
    
    def __init__(self, game_state: GameState, demo: "DiscreteActionDemo"):
        self.game_state = game_state
        self.demo = demo
    
    def draw(self, canvas, theme, display_level, renderer=None):
        """Draw game state HUD."""
        # Use renderer dimensions if available (renderer is the Window)
        if renderer:
            width = renderer.width
            height = renderer.height
        else:
            width = 800
            height = 600
            
        x = 20
        y = height - 30
        line_height = 22
        
        # Background panel
        panel_width = 280
        panel_height = 320
        arcade.draw_lbwh_rectangle_filled(
            10, height - 10 - panel_height, panel_width, panel_height,
            (0, 0, 0, 180)
        )
        arcade.draw_lbwh_rectangle_outline(
            10, height - 10 - panel_height, panel_width, panel_height,
            (100, 150, 255, 255), 2
        )
        
        # Title
        arcade.draw_text(
            "GAME STATE",
            x, y, (100, 200, 255), 16, bold=True
        )
        y -= line_height + 8
        
        # Score
        blue_score = self.game_state.blue_score.get() if hasattr(self.game_state, 'blue_score') else 0
        arcade.draw_text(
            f"⭐ Blue Score: {blue_score}",
            x, y, (100, 150, 255), 14, bold=True
        )
        y -= line_height
        
        # Robot inventory
        robot_state = self._get_robot_state()
        balls_held = 0
        if robot_state:
            inventory = robot_state.gamepieces.get() if hasattr(robot_state, 'gamepieces') else {}
            balls_held = inventory.get(Fuel, 0) if inventory else 0
        
        arcade.draw_text(
            f"🤖 Robot Inventory: {balls_held} balls",
            x, y, (255, 220, 100), 14
        )
        y -= line_height + 8
        
        # Zone states
        arcade.draw_text("── Zone Ball Counts ──", x, y, (150, 150, 150), 12)
        y -= line_height
        
        zone_data = [
            ("Blue Depot", "🏭"),
            ("Blue Alliance Zone", "🔵"),
            ("Neutral Zone", "⚪"),
        ]
        
        for zone_name, emoji in zone_data:
            count = self._get_zone_balls(zone_name)
            color = (200, 200, 200) if count > 0 else (100, 100, 100)
            arcade.draw_text(
                f"{emoji} {zone_name}: {count}",
                x, y, color, 13
            )
            y -= line_height
        
        y -= 8
        
        # Hub state
        arcade.draw_text("── Hub Stats ──", x, y, (150, 150, 150), 12)
        y -= line_height
        
        hub_active, hub_inactive = self._get_hub_stats()
        arcade.draw_text(
            f"🎯 Active Scores: {hub_active}",
            x, y, (100, 255, 100), 13
        )
        y -= line_height
        arcade.draw_text(
            f"🎯 Inactive Scores: {hub_inactive}",
            x, y, (255, 200, 100), 13
        )
        y -= line_height + 8
        
        # Current action
        arcade.draw_text("── Current Action ──", x, y, (150, 150, 150), 12)
        y -= line_height
        
        if self.demo.current_action_idx < len(DEMO_ACTIONS):
            action = DEMO_ACTIONS[self.demo.current_action_idx]
            arcade.draw_text(
                f"#{self.demo.current_action_idx + 1}: {action.description}",
                x, y, (255, 255, 255), 12
            )
        else:
            arcade.draw_text(
                "Demo complete!",
                x, y, (100, 255, 100), 12
            )
    
    def _get_robot_state(self):
        """Get robot state from game state."""
        try:
            return self.game_state.get("robots").get(self.demo.robot_base_name)
        except:
            return None
    
    def _get_zone_balls(self, zone_name: str) -> int:
        """Get ball count for a zone."""
        try:
            zone_state = self.game_state.get("interactables").get(zone_name)
            if zone_state:
                return zone_state.fuel_available.get()
        except:
            pass
        return 0
    
    def _get_hub_stats(self) -> Tuple[int, int]:
        """Get Hub scoring stats."""
        try:
            hub_state = self.game_state.get("interactables").get("Blue Hub")
            if hub_state:
                return (
                    hub_state.fuel_scored_active.get(),
                    hub_state.fuel_scored_inactive.get()
                )
        except:
            pass
        return (0, 0)


# =============================================================================
# MAIN DEMO
# =============================================================================

class DiscreteActionDemo:
    """Animated demonstration of discrete action system."""
    
    def __init__(self):
        # Create game
        self.game = create_rebuilt_game()
        
        # Create robot
        self.robot = create_robot("BlueBot", Alliance.BLUE)
        self.robot_base_name = "BlueBot"  # Use this for server calls
        
        # Game server
        self.server = DiscreteGameServer(ServerConfig())
        self.server.load_from_game(self.game)
        self.server.add_robot(self.robot)
        
        # Starting position near Blue Depot
        start_x = Feet(6)
        start_y = HALF_WIDTH
        self.server.init_robot(
            self.robot_base_name,
            RobotState(start_x, start_y, Degree(0))
        )
        
        # Configure robot interactions for all interactables
        self._configure_robot_interactions()
        
        # Animation state
        self.current_trajectory = None
        self.anim_time = 0.0
        self.current_robot_state = RobotState(start_x, start_y, Degree(0))
        self.is_animating = False
        self.is_waiting = False
        self.wait_time = 0.0
        
        # Demo state
        self.current_action_idx = 0
        self.last_result = None
        
        # Setup renderer
        self.renderer = Renderer.create(game=self.game)
        self.renderer.display_level = DisplayLevel.SHOWCASE
        
        # Add obstacles
        for obs in self.game.get_obstacles():
            self.renderer.add(obs)
        
        # Track robot
        self.renderer.track_robot(lambda: self.current_robot_state)
        
        # Add game state HUD
        self.hud = GameStateHUD(self.server.match.game_state, self)
        
        # Register HUD render handler
        from gamegine.render.renderer import ObjectRendererRegistry
        ObjectRendererRegistry.register_handler(GameStateHUD, lambda obj, canvas, theme, display_level, renderer=None: obj.draw(canvas, theme, display_level, renderer))
        
        self.renderer.add_dynamic(lambda: self.hud, "ui")
        
        # Register callbacks
        self.renderer.on_update_callback(self.update)
        self.renderer.on_key_press_callback(self.on_key_press)
        
        # Show initial alert
        self.renderer.show_alert(
            f"DISCRETE ACTION DEMO | {len(DEMO_ACTIONS)} actions to perform | Press SPACE to start",
            AlertType.INFO, 5.0
        )
    
    def _configure_robot_interactions(self):
        """Configure robot interaction configs for all interactables."""
        # Get all interactables and create interaction configs
        for interactable in self.game.get_interactables():
            nav_point = interactable.get_navigation_point()
            for interaction in interactable.get_interactions():
                # Create interaction config with navigation point and timing
                config = RobotInteractionConfig(
                    interactable_name=interactable.name,
                    interaction_identifier=interaction.identifier,
                    able_to_interact=lambda *args, **kwargs: True,  # Delegate to interactable
                    time_to_interact=lambda *args, **kwargs: 0.5,  # Base time
                    navigation_point=nav_point,
                )
                self.robot.add_interaction_config(config)
    
    def on_key_press(self, key, modifiers):
        """Handle key presses."""
        if key == arcade.key.SPACE:
            if not self.is_animating and not self.is_waiting:
                self.execute_next_action()
        elif key == arcade.key.R:
            self.reset_demo()
        elif key == arcade.key.ESCAPE:
            arcade.exit()
    
    def reset_demo(self):
        """Reset the entire demo."""
        self.current_action_idx = 0
        self.is_animating = False
        self.is_waiting = False
        
        # Reset robot position
        # Start at a neutral position to demonstrate driving to first interactable
        start_x = Feet(2)
        start_y = Feet(2)
        self.current_robot_state = RobotState(start_x, start_y, Degree(0))
        self.server.init_robot(
            self.robot_base_name,
            RobotState(start_x, start_y, Degree(0))
        )
        
        # Reset game state
        self.server.match.reset()
        self._configure_robot_interactions()
        
        self.renderer.show_alert("Demo reset! Press SPACE to start", AlertType.INFO)
    
    def execute_next_action(self):
        """Execute the next action in the sequence."""
        if self.current_action_idx >= len(DEMO_ACTIONS):
            self.renderer.show_alert("🎉 Demo complete! Press R to reset", AlertType.SUCCESS)
            return
        
        action = DEMO_ACTIONS[self.current_action_idx]
        
        self.renderer.show_alert(
            f"Action {self.current_action_idx + 1}/{len(DEMO_ACTIONS)}: {action.description}",
            AlertType.INFO, 2.0
        )
        
        # Drive to interactable and process action
        try:
            success, trajectory = self.server.drive_and_process_action(
                action.interactable,
                action.interaction,
                self.robot_base_name,
            )
            print(f"DEBUG: Action={action.interactable}:{action.interaction}")
            print(f"DEBUG: Success={success}")
            
            if trajectory:
                print(f"DEBUG: Trajectory duration={trajectory.get_travel_time()}")
                self.current_trajectory = trajectory
                self.anim_time = 0.0
                self.is_animating = True
                self.last_result = success
            else:
                print("DEBUG: Already at location")
                self.renderer.show_alert("Already at location - Executing...", AlertType.INFO)
                self.current_trajectory = None
                self.is_animating = False
                self.is_waiting = True
                self.wait_time = 1.0 # Simulate action time
            
        except Exception as e:
            self.renderer.show_alert(f"❌ Error: {str(e)}", AlertType.ERROR, 3.0)
            print(f"ERROR: {e}")
            self.current_action_idx += 1
    
    def update(self, dt):
        """Update animation each frame."""
        if self.is_waiting:
            self.wait_time -= dt
            if self.wait_time <= 0:
                self.is_waiting = False
                self.current_action_idx += 1
                if self.current_action_idx < len(DEMO_ACTIONS):
                    self.renderer.show_alert(
                        f"Press SPACE for next action ({self.current_action_idx + 1}/{len(DEMO_ACTIONS)})",
                        AlertType.INFO
                    )
            return
        
        if not self.is_animating or self.current_trajectory is None:
            return
        
        self.anim_time += dt
        travel_time = float(self.current_trajectory.get_travel_time().to(Second))
        
        if self.anim_time >= travel_time:
            # Animation complete
            state = self.current_trajectory.get_at_time(self.current_trajectory.get_travel_time())
            self.current_robot_state = RobotState(state.x, state.y, state.theta)
            self.is_animating = False
            
            # Show result
            action = DEMO_ACTIONS[self.current_action_idx]
            if self.last_result:
                self.renderer.show_alert(
                    f"✅ {action.interaction} successful!",
                    AlertType.SUCCESS, 1.5
                )
            else:
                self.renderer.show_alert(
                    f"❌ {action.interaction} failed (conditions not met)",
                    AlertType.WARNING, 1.5
                )
            
            # Brief wait before allowing next action
            self.is_waiting = True
            self.wait_time = 1.0
        else:
            # Continue animation
            state = self.current_trajectory.get_at_time(Second(self.anim_time))
            self.current_robot_state = RobotState(state.x, state.y, state.theta)
    
    def run(self):
        """Run the demo."""
        arcade.run()


# =============================================================================
# ENTRY POINT
# =============================================================================

def main():
    print("=" * 70)
    print("REBUILT DISCRETE ACTION DEMO")
    print("=" * 70)
    print("\nThis demo showcases the new discrete action system with:")
    print("  • Zone-based ball storage (Depot, Neutral Zone, Alliance Zone)")
    print("  • Discrete pickup amounts (1, 5, or 10 balls at a time)")
    print("  • Shooting locations with distance-based accuracy")
    print("  • Shuttling to pass balls to teammates")
    print("  • Tower climbing for endgame points")
    print()
    print("Controls:")
    print("  SPACE - Execute next action in sequence")
    print("  R     - Reset demo")
    print("  ESC   - Exit")
    print()
    print(f"Sequence has {len(DEMO_ACTIONS)} predefined actions to demonstrate.")
    print()
    
    demo = DiscreteActionDemo()
    demo.run()


if __name__ == "__main__":
    main()
