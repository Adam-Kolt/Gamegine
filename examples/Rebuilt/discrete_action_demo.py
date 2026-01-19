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
import math
import arcade
import random
from dataclasses import dataclass

from examples.Rebuilt.Rebuilt import create_rebuilt_game, HALF_LENGTH, HALF_WIDTH
from examples.Rebuilt.scoring import Fuel, NeutralZone, AllianceZone, Depot, Hub, Tower, MatchPhaseManager
from examples.Rebuilt.shooting_locations import ShootingLocation

from gamegine.representation.robot import SwerveRobot, PhysicalParameters
from gamegine.render.renderer import Renderer, DisplayLevel, AlertType, ObjectRendererRegistry
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
# DEMO ACTIONS
# =============================================================================

@dataclass
class DemoAction:
    """A single action in the demo sequence."""
    interactable: str
    interaction: str
    description: str


# Legacy single-robot demo actions (kept for reference)
LEGACY_DEMO_ACTIONS: List[DemoAction] = [
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
# MULTI-ROBOT CONFIGURATION
# =============================================================================

@dataclass
class RobotConfig:
    """Configuration for a single robot in the demo."""
    name: str
    start_x: Meter
    start_y: Meter
    policy: str  # "cycle", "zone", "defender"
    color: Tuple[int, int, int]


# 3 Blue alliance robots with distinct starting positions and policies
ROBOT_CONFIGS = [
    RobotConfig("BlueBot1", Feet(6), HALF_WIDTH - Feet(6), "cycle", (52, 152, 219)),   # Blue - Cycle bot
    RobotConfig("BlueBot2", Feet(6), HALF_WIDTH, "zone", (46, 204, 113)),              # Green - Zone bot
    RobotConfig("BlueBot3", Feet(6), HALF_WIDTH + Feet(6), "defender", (155, 89, 182)), # Purple - Defender
]


@dataclass 
class RobotAnimState:
    """Animation state for a single robot."""
    robot_name: str
    config: RobotConfig
    current_state: RobotState
    current_trajectory: Optional[Any] = None
    anim_time: float = 0.0
    is_animating: bool = False
    is_waiting: bool = False
    wait_time: float = 0.0
    action_queue: List = None  # Projectile animations queue
    spawn_timer: float = 0.0
    actions_performed: int = 0
    balls_scored: int = 0
    has_climbed: bool = False
    
    def __post_init__(self):
        if self.action_queue is None:
            self.action_queue = []


# =============================================================================
# AI POLICY FUNCTIONS
# =============================================================================

def select_next_action(robot_name: str, robot_state: RobotState, 
                        game_state: GameState, policy: str,
                        actions_performed: int, has_climbed: bool) -> Optional[DemoAction]:
    """Select next action based on robot's policy and current state.
    
    Returns None if no more actions (e.g., robot has climbed and is done).
    """
    # Get current ball count
    inventory = robot_state.gamepieces.get() if hasattr(robot_state, 'gamepieces') else {}
    ball_count = inventory.get(Fuel, 0)
    
    # Get match time
    current_time = game_state.current_time.get() if hasattr(game_state, 'current_time') else 0
    
    # Check if should climb (last 10 seconds and enough actions done)
    if current_time > 130 and not has_climbed and actions_performed >= 5:
        return DemoAction("Blue Tower", "climb_level_2", "Climb for endgame!")
    
    if has_climbed:
        return None  # Done for the match
    
    if policy == "cycle":
        # Cycle bot: Depot -> Near shooting location -> repeat
        if ball_count < 5:
            return DemoAction("Blue Depot", "pickup_5", "Restock from Depot")
        else:
            # Rotate between shooting locations
            locations = ["BLUE Near Top", "BLUE Near Bot", "BLUE Mid Top"]
            loc = locations[actions_performed % len(locations)]
            return DemoAction(loc, "shoot", f"Shoot from {loc}")
    
    elif policy == "zone":
        # Zone bot: Neutral Zone / Alliance Zone -> Far shooting -> repeat
        if ball_count < 3:
            # Alternate between zones
            if actions_performed % 3 == 0:
                return DemoAction("Neutral Zone", "pickup_1", "Get from Neutral Zone")
            else:
                return DemoAction("Blue Alliance Zone", "pickup_1", "Get from Alliance Zone")
        else:
            locations = ["BLUE Mid Top", "BLUE Mid Bot", "BLUE Far Top"]
            loc = locations[actions_performed % len(locations)]
            return DemoAction(loc, "shoot", f"Shoot from {loc}")
    
    elif policy == "defender":
        # Defender: Collect some, shoot a bit, then climb early
        if actions_performed < 3:
            if ball_count < 5:
                return DemoAction("Blue Depot", "pickup_5", "Quick pickup")
            else:
                return DemoAction("BLUE Near Top", "shoot", "Quick score")
        elif actions_performed < 6:
            # Shuttle to help teammates
            if ball_count < 5:
                return DemoAction("Blue Depot", "pickup_5", "Grab for shuttle")
            else:
                return DemoAction("BLUE Mid Bot", "shuttle", "Pass to zone")
        else:
            # Climb early for guaranteed points
            if not has_climbed:
                return DemoAction("Blue Tower", "climb_level_3", "Early climb for 30 pts!")
    
    # Default: just shoot if we have balls
    if ball_count > 0:
        return DemoAction("BLUE Near Top", "shoot", "Default shot")
    else:
        return DemoAction("Blue Depot", "pickup_1", "Default pickup")



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
        
        # Time
        time_val = self.game_state.current_time.get()
        arcade.draw_text(
            f"⏱️ Time: {time_val:.1f}s",
            x + 160, y, (200, 200, 200), 14, bold=True
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
# RENDERING HANDLERS
# =============================================================================

def draw_shooting_location(obj, canvas, theme, display_level, renderer=None):
    """Draw a shooting location as a target crosshair."""
    if not isinstance(obj, ShootingLocation):
        return
        
    # Use SpatialMeasurement directly
    x = obj.bounds.x
    y = obj.bounds.y
    radius = Inch(12) 
    
    color = (100, 150, 255) if obj.alliance == Alliance.BLUE else (255, 100, 100)
    
    # Draw crosshair using canvas methods (handles scaling)
    canvas.draw_line(x - radius, y, x + radius, y, color, 2)
    canvas.draw_line(x, y - radius, x, y + radius, color, 2)
    canvas.draw_circle(x, y, radius, color, filled=False, line_width=2)
    
    # Draw label
    # Offset text slightly above
    text_offset = Inch(18)
    canvas.draw_text(obj.location_key, x, y + text_offset, color, 10, anchor_x="center")

def draw_depot(obj, canvas, theme, display_level, renderer=None):
    """Draw depot as a dashed box."""
    if not isinstance(obj, Depot):
        return
    
    # Get center and dimensions as SpatialMeasurement
    if hasattr(obj.bounds, 'get_center'):
        center = obj.bounds.get_center()
        cx, cy = center[0], center[1]
    else:
        cx, cy = obj.bounds.x, obj.bounds.y

    # Dimensions
    if hasattr(obj.bounds, 'width'):
         width, height = obj.bounds.width, obj.bounds.height
    else:
         width, height = Inch(48), Inch(48)
    
    color = (100, 150, 255) if obj.alliance == Alliance.BLUE else (255, 100, 100)
    
    # Canvas.draw_rectangle takes center
    canvas.draw_rectangle(cx, cy, width, height, color, filled=False, line_width=2)
    canvas.draw_text(obj.name, cx, cy, color, 10, anchor_x="center", anchor_y="center")

def draw_hub(obj, canvas, theme, display_level, renderer=None):
    """Draw Hub as a hexagon."""
    if not isinstance(obj, Hub):
         return
         
    x = obj.bounds.x
    y = obj.bounds.y
    radius = Inch(24)
    
    color = (100, 150, 255) if obj.alliance == Alliance.BLUE else (255, 100, 100)
    
    # Draw Hub visual
    fill_color = (0, 0, 255, 100) if obj.alliance == Alliance.BLUE else (255, 0, 0, 100)
    canvas.draw_circle(x, y, radius, fill_color, filled=True)
    canvas.draw_circle(x, y, radius, color, filled=False, line_width=3)
    
    canvas.draw_text("HUB", x, y, (255, 255, 255), 12, anchor_x="center", anchor_y="center") # Bold not supported in canvas wrapper yet

def draw_tower(obj, canvas, theme, display_level, renderer=None):
    """Draw Tower as a rectangle."""
    if not isinstance(obj, Tower):
        return
        
    if hasattr(obj.bounds, 'get_center'):
        center = obj.bounds.get_center()
        cx, cy = center[0], center[1]
        width, height = obj.bounds.width, obj.bounds.height
    else:
        cx, cy = obj.bounds.x, obj.bounds.y
        width, height = Inch(30), Inch(30)
    
    color = (150, 150, 150) # Grey
    
    canvas.draw_rectangle(cx, cy, width, height, color, filled=True)
    canvas.draw_text("TWR", cx, cy, (0,0,0), 10, anchor_x="center", anchor_y="center")

def draw_alliance_zone(obj, canvas, theme, display_level, renderer=None):
    """Draw Alliance Zone."""
    if not isinstance(obj, AllianceZone):
        return
        
    if hasattr(obj.bounds, 'get_center'):
        center = obj.bounds.get_center()
        cx, cy = center[0], center[1]
        width, height = obj.bounds.width, obj.bounds.height
    else:
        cx, cy = obj.bounds.x, obj.bounds.y
        width, height = Inch(60), Inch(60)
    
    color = (100, 150, 255, 100) if obj.alliance == Alliance.BLUE else (255, 100, 100, 100)
    
    canvas.draw_rectangle(cx, cy, width, height, color, filled=True)
    canvas.draw_text("ZONE", cx, cy, (255,255,255), 10, anchor_x="center", anchor_y="center")

def draw_neutral_zone(obj, canvas, theme, display_level, renderer=None):
    """Draw Neutral Zone."""
    if not isinstance(obj, NeutralZone):
        return
        
    x = obj.bounds.x
    y = obj.bounds.y
    radius = Inch(36)
    
    canvas.draw_circle(x, y, radius, (200, 200, 200, 80), filled=True)
    canvas.draw_circle(x, y, radius, (150, 150, 150), filled=False, line_width=2)
    canvas.draw_text("Neutral", x, y, (50, 50, 50), 10, anchor_x="center", anchor_y="center")


# =============================================================================
# ANIMATION SYSTEM
# =============================================================================

class ProjectileAnimation:
    """Parametric animation of a projectile with optional arc for visibility."""
    
    def __init__(self, start_pos: Tuple[Any, Any], end_pos: Tuple[Any, Any], 
                 duration: float = 0.5, arc_offset: float = 0.0):
        """
        Args:
            start_pos: Starting position (x, y)
            end_pos: Ending position (x, y)
            duration: Animation duration in seconds
            arc_offset: Perpendicular offset at midpoint (in inches). Positive = arc left, negative = arc right.
                       Use this to swing the ball out so it's visible over the robot.
        """
        self.start_x = start_pos[0]
        self.start_y = start_pos[1]
        self.end_x = end_pos[0]
        self.end_y = end_pos[1]
        self.duration = duration
        self.elapsed = 0.0
        self.arc_offset = Inch(arc_offset)  # Perpendicular offset at peak
        
    def update(self, dt: float) -> bool:
        """Update animation. Return True if finished."""
        self.elapsed += dt
        return self.elapsed >= self.duration
        
    def draw(self, canvas, theme, display_level, renderer=None):
        progress = min(1.0, self.elapsed / self.duration)
        
        # Linear interpolation along the path
        curr_x = self.start_x + (self.end_x - self.start_x) * progress
        curr_y = self.start_y + (self.end_y - self.start_y) * progress
        
        # Calculate perpendicular offset using sin curve for smooth arc
        # This swings the ball out to the side for visibility
        arc_amount = math.sin(progress * math.pi)  # 0 at start/end, 1 at middle
        
        if self.arc_offset != Inch(0):
            # Calculate perpendicular direction to path
            dx = self.end_x - self.start_x
            dy = self.end_y - self.start_y
            
            # Perpendicular vector (rotate 90 degrees)
            # For SpatialMeasurement, we need to handle the math carefully
            # Use normalized-ish approach: offset perpendicular to travel direction
            try:
                # Get magnitude for normalization
                dist_sq = float(dx.to(Inch))**2 + float(dy.to(Inch))**2
                if dist_sq > 0:
                    dist = math.sqrt(dist_sq)
                    # Perpendicular unit vector (rotated 90 degrees)
                    perp_x = -float(dy.to(Inch)) / dist
                    perp_y = float(dx.to(Inch)) / dist
                    
                    # Apply arc offset
                    curr_x = curr_x + Inch(perp_x * float(self.arc_offset.to(Inch)) * arc_amount)
                    curr_y = curr_y + Inch(perp_y * float(self.arc_offset.to(Inch)) * arc_amount)
            except:
                pass  # If math fails, just use linear path
        
        # Scale ball size to simulate 3D height (ball appears larger when "higher")
        scale = 1.0 + arc_amount * 0.5
        radius = Inch(3.5) * scale
        
        # Draw ball (Fuel is orange) with glow effect for better visibility
        # Draw outer glow first
        canvas.draw_circle(curr_x, curr_y, radius * 1.3, (255, 200, 100, 150), filled=True)
        # Draw main ball
        canvas.draw_circle(curr_x, curr_y, radius, (255, 140, 0), filled=True)
        canvas.draw_circle(curr_x, curr_y, radius, (200, 100, 0), filled=False)


class AnimationLayer:
    """Manages active animations."""
    
    def __init__(self):
        self.animations: List[ProjectileAnimation] = []
        
    def add(self, anim: ProjectileAnimation):
        self.animations.append(anim)
        
    def update(self, dt: float):
        # Update all, keep only unfinished
        self.animations = [a for a in self.animations if not a.update(dt)]
        
    def draw(self, canvas, theme, display_level, renderer=None):
        for anim in self.animations:
            anim.draw(canvas, theme, display_level, renderer)
            
            
# Register AnimationLayer handler
def draw_animation_layer(obj, canvas, theme, display_level, renderer=None):
    if isinstance(obj, AnimationLayer):
        obj.draw(canvas, theme, display_level, renderer)


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
        initial_state = RobotState(start_x, start_y, Degree(0))
        # Set robot name in state so Tower climbing can track climbed robots
        initial_state.setValue("name", self.robot_base_name)
        self.server.init_robot(
            self.robot_base_name,
            initial_state
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
        self.animation_queue = []
        self.spawn_timer = 0.0
        
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
        
        # Initialize Animation Layer
        self.animation_layer = AnimationLayer()
        
        # Register HUD render handler
        from gamegine.render.renderer import ObjectRendererRegistry
        ObjectRendererRegistry.register_handler(GameStateHUD, lambda obj, canvas, theme, display_level, renderer=None: obj.draw(canvas, theme, display_level, renderer))
        
        # Register Animation render handler
        ObjectRendererRegistry.register_handler(AnimationLayer, draw_animation_layer)
        
        # Register INTERACTABLE handlers for visualization
        ObjectRendererRegistry.register_handler(ShootingLocation, draw_shooting_location)
        ObjectRendererRegistry.register_handler(Depot, draw_depot)
        ObjectRendererRegistry.register_handler(Hub, draw_hub)
        ObjectRendererRegistry.register_handler(Tower, draw_tower)
        ObjectRendererRegistry.register_handler(AllianceZone, draw_alliance_zone)
        ObjectRendererRegistry.register_handler(NeutralZone, draw_neutral_zone)

        # Add all interactables to renderer so they are drawn
        for interactable in self.game.get_interactables():
             self.renderer.add(interactable)
        
        self.renderer.add_dynamic(lambda: self.hud, "ui")
        self.renderer.add(self.animation_layer)
        
        # Register callbacks
        def update_with_anim(dt):
            self.update(dt)
            self.animation_layer.update(dt)
            
        self.renderer.on_update_callback(update_with_anim)
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
        reset_state = RobotState(start_x, start_y, Degree(0))
        reset_state.setValue("name", self.robot_base_name)  # Include name for Tower climbing
        self.current_robot_state = reset_state
        self.server.init_robot(
            self.robot_base_name,
            reset_state
        )
        
        # Reset game state
        self.server.match.reset()
        self._configure_robot_interactions()
        
        self.renderer.show_alert("Demo reset! Press SPACE to start", AlertType.INFO)

    def _get_center(self, obj) -> Tuple[Any, Any]:
        """Get the center coordinates of an object (RobotState or Interactable)."""
        if hasattr(obj, 'x') and hasattr(obj, 'y') and not hasattr(obj, 'bounds'):
            # RobotState attributes are ValueEntry - need to call .get()
            x = obj.x.get() if hasattr(obj.x, 'get') else obj.x
            y = obj.y.get() if hasattr(obj.y, 'get') else obj.y
            return (x, y)
        if hasattr(obj, 'bounds'):
            b = obj.bounds
            if hasattr(b, 'get_center'):
                return b.get_center()
            return (b.x, b.y)
        raise ValueError(f"Unknown object type {type(obj)}")

    def _get_robot_state(self):
        """Get robot state from game state."""
        try:
            return self.server.match.game_state.get("robots").get(self.robot_base_name)
        except:
            return None

    def _get_ball_count(self) -> int:
        """Get current fuel count in robot inventory."""
        robot_state = self._get_robot_state()
        if not robot_state: return 0
        inventory = robot_state.gamepieces.get() if hasattr(robot_state, 'gamepieces') else {}
        return inventory.get(Fuel, 0)
        
    def _queue_burst(self, start_pos, end_pos, count: int, arc_offset: float = 0.0):
        """Queue a burst of projectile animations.
        
        Args:
            start_pos: Starting position
            end_pos: Ending position
            count: Number of projectiles
            arc_offset: Perpendicular arc offset in inches (for visibility)
        """
        for i in range(count):
            delay = i * 0.08 + random.uniform(0.0, 0.05) # Staggered with jitter
            
            # Simple jitter in inches
            sx, sy = start_pos
            ex, ey = end_pos
            
            # Add spatial jitter
            jitter_start = (sx + Inch(random.uniform(-4, 4)), sy + Inch(random.uniform(-4, 4)))
            jitter_end = (ex + Inch(random.uniform(-4, 4)), ey + Inch(random.uniform(-4, 4)))
            
            # Alternate arc direction for visual variety
            arc_dir = 1 if i % 2 == 0 else -1
            actual_arc = arc_offset * arc_dir + random.uniform(-10, 10)  # Add variation
            
            anim = ProjectileAnimation(jitter_start, jitter_end, duration=0.6, arc_offset=actual_arc)
            self.animation_queue.append({"delay": delay, "anim": anim})
            
        # Ensure we wait long enough for all to spawn + fly
        min_wait = (count * 0.1) + 0.6
        if self.wait_time < min_wait:
            self.wait_time = min_wait
    
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
            # Capture inventory before action
            balls_before = self._get_ball_count()
            
            # Execute action
            success, trajectory = self.server.drive_and_process_action(
                action.interactable,
                action.interaction,
                self.robot_base_name,
            )
            print(f"DEBUG: Action={action.interactable}:{action.interaction}")
            print(f"DEBUG: Success={success}")
            
            # Calculate inventory delta for animation
            balls_after = self._get_ball_count()
            delta = balls_after - balls_before
            
            if trajectory:
                print(f"DEBUG: Trajectory duration={trajectory.get_travel_time()}")
                self.current_trajectory = trajectory
                self.anim_time = 0.0
                self.is_animating = True
                self.last_result = success
                
                # Get robot's DESTINATION position from trajectory end
                end_state = trajectory.get_at_time(trajectory.get_travel_time())
                robot_dest_pos = (end_state.x, end_state.y)
                
                # Determine animation based on delta
                if success:
                    self._plan_animations_from_delta(action, delta, robot_dest_pos)
                    
            else:
                print("DEBUG: Already at location")
                self.renderer.show_alert("Already at location - Executing...", AlertType.INFO)
                self.current_trajectory = None
                self.is_animating = False
                # Start waiting logic which handles animations
                self.is_waiting = True
                self.wait_time = 1.0 
                self.spawn_timer = 0.0 # Reset spawn timer
                
                # Robot is already at destination, use current position
                robot_pos = self._get_center(self.current_robot_state)
                
                if success:
                    self._plan_animations_from_delta(action, delta, robot_pos)
            
        except Exception as e:
            self.renderer.show_alert(f"❌ Error: {str(e)}", AlertType.ERROR, 3.0)
            print(f"ERROR: {e}")
            self.current_action_idx += 1

    def _plan_animations_from_delta(self, action, delta: int, robot_pos: Tuple[Any, Any]):
        """Plan animations based on inventory change.
        
        Args:
            action: The action being performed
            delta: Change in inventory (positive = pickup, negative = shoot/shuttle)
            robot_pos: The robot's position (destination after movement)
        """
        count = abs(delta)
        
        # Special case: Shoot actions always visualize at least 1 ball if success, 
        # even if score logic is complex (though usually delta is -1).
        if count == 0 and action.interaction == "shoot":
            # Just show 1 for visual feedback
            count = 1
            delta = -1
            
        if count == 0: return

        interactable_obj = self.server.match.interactables.get(action.interactable)
        if not interactable_obj: return
        
        if delta > 0: 
            # PICKUP: Interactable -> Robot
            # Use large arc to make balls visible over robot
            start = self._get_center(interactable_obj)
            end = robot_pos  # Use passed robot destination
            self._queue_burst(start, end, count, arc_offset=60)  # 60 inch arc for visibility
            
        else:
            # LOSS (Shoot/Shuttle): Robot -> Target
            start = robot_pos  # Use passed robot destination
            
            # Determine Target
            end = None
            if action.interaction == "shoot" or "score" in action.interaction:
                 hub = next((i for i in self.game.get_interactables() if isinstance(i, Hub)), None)
                 if hub: end = self._get_center(hub)
            elif action.interaction == "shuttle":
                 zone = next((i for i in self.game.get_interactables() if isinstance(i, AllianceZone) and i.alliance == Alliance.BLUE), None)
                 if zone: end = self._get_center(zone)
            
            # Fallback: use interactable location
            if not end:
                end = self._get_center(interactable_obj)
                
            if end:
                self._queue_burst(start, end, count, arc_offset=30)  # Smaller arc for shooting
    
    def update(self, dt):
        """Update animation each frame."""
        if self.is_waiting:
            self.wait_time -= dt
            self.spawn_timer += dt
            
            # Process Animation Queue
            # Trigger items whose delay has passed
            remaining = []
            for item in self.animation_queue:
                if self.spawn_timer >= item["delay"]:
                     self.animation_layer.add(item["anim"])
                else:
                    remaining.append(item)
            self.animation_queue = remaining

            if self.wait_time <= 0:
                self.is_waiting = False
                self.spawn_timer = 0.0
                self.animation_queue = [] # Clear any leftovers
                
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
            
            # Prepare waiting phase (animations play here)
            self.is_waiting = True
            self.wait_time = 1.0 # Min wait, will be extended by queue check
            self.spawn_timer = 0.0
            
            # Check if queue has long items?
            # _queue_burst extended wait_time.

        else:
            # Continue animation
            state = self.current_trajectory.get_at_time(Second(self.anim_time))
            self.current_robot_state = RobotState(state.x, state.y, state.theta)
    
    def run(self):
        """Run the demo."""
        arcade.run()


# =============================================================================
# MULTI-ROBOT HUD (Top Right Corner)
# =============================================================================

class MultiRobotHUD:
    """HUD for multi-robot demo, positioned in top-right corner."""
    
    def __init__(self, demo: "MultiRobotDemo"):
        self.demo = demo
    
    def draw(self, canvas, theme, display_level, renderer=None):
        """Draw HUD in top-right corner."""
        if renderer:
            width = renderer.width
            height = renderer.height
        else:
            width = 800
            height = 600
        
        panel_width = 300
        panel_height = 360
        x = width - panel_width - 10
        y = height - 30
        line_height = 20
        
        # Background panel (top right)
        arcade.draw_lbwh_rectangle_filled(
            width - panel_width - 10, height - panel_height - 10, 
            panel_width, panel_height,
            (0, 0, 0, 200)
        )
        arcade.draw_lbwh_rectangle_outline(
            width - panel_width - 10, height - panel_height - 10, 
            panel_width, panel_height,
            (100, 200, 255, 255), 2
        )
        
        # Match time & score
        match_time = self.demo.match_time
        remaining = max(0, self.demo.match_duration - match_time)
        
        arcade.draw_text(
            f"⏱️ Match: {match_time:.1f}s / {self.demo.match_duration:.0f}s",
            x, y, (255, 255, 255), 14, bold=True
        )
        y -= line_height
        
        # Match phase info
        phase_name = self.demo.phase_manager.get_phase_name()
        phase_remaining = self.demo.phase_manager.get_time_remaining_in_phase(match_time)
        blue_active, red_active = self.demo.phase_manager.get_hub_activation(self.demo.phase_manager.current_phase)
        
        # Color based on phase
        phase_color = (255, 255, 0) if self.demo.phase_manager.current_phase.value.startswith("period") else (100, 255, 100)
        arcade.draw_text(
            f"⚡ {phase_name} ({phase_remaining:.0f}s left)",
            x, y, phase_color, 13, bold=True
        )
        y -= line_height
        
        # Hub activation status
        if blue_active and red_active:
            hub_status = "🔵🔴 Both ACTIVE"
            hub_color = (100, 255, 100)
        elif blue_active:
            hub_status = "🔵 Blue ACTIVE"
            hub_color = (100, 180, 255)
        elif red_active:
            hub_status = "🔴 Red ACTIVE"
            hub_color = (255, 100, 100)
        else:
            hub_status = "⏸️ Neither active"
            hub_color = (150, 150, 150)
        
        arcade.draw_text(hub_status, x, y, hub_color, 12)
        y -= line_height + 5
        
        # Score
        game_state = self.demo.server.match.game_state
        blue_score = game_state.blue_score.get() if hasattr(game_state, 'blue_score') else 0
        arcade.draw_text(
            f"⭐ Blue Score: {blue_score}",
            x, y, (100, 180, 255), 15, bold=True
        )
        y -= line_height + 10
        
        # Robot states
        arcade.draw_text("── Robot Status ──", x, y, (150, 150, 150), 12)
        y -= line_height
        
        for name, anim_state in self.demo.robot_states.items():
            config = anim_state.config
            color = config.color
            
            # Get ball count
            robot_state = self.demo.server.match.game_state.get("robots").get(name)
            balls = 0
            if robot_state:
                inv = robot_state.gamepieces.get() if hasattr(robot_state, 'gamepieces') else {}
                balls = inv.get(Fuel, 0) if inv else 0
            
            # Status
            if anim_state.has_climbed:
                status = "✓ CLIMBED"
                status_color = (0, 255, 0)
            elif anim_state.is_animating:
                status = "▶ DRIVING"
                status_color = (255, 255, 0)
            elif anim_state.is_waiting:
                status = "⏳ ACTION"
                status_color = (255, 165, 0)
            else:
                status = "○ READY"
                status_color = (150, 150, 150)
            
            arcade.draw_text(
                f"● {name}",
                x, y, (*color, 255), 13
            )
            arcade.draw_text(
                f"{status}",
                x + 90, y, status_color, 12
            )
            arcade.draw_text(
                f"🔵{balls}",
                x + 180, y, (255, 220, 100), 12
            )
            arcade.draw_text(
                f"#{anim_state.actions_performed}",
                x + 220, y, (180, 180, 180), 11
            )
            y -= line_height
        
        y -= 8
        
        # Zone ball counts
        arcade.draw_text("── Zone Counts ──", x, y, (150, 150, 150), 12)
        y -= line_height
        
        for zone_name, emoji in [("Blue Depot", "🏭"), ("Neutral Zone", "⚪"), ("Blue Alliance Zone", "🔵")]:
            count = self._get_zone_balls(zone_name)
            arcade.draw_text(
                f"{emoji} {zone_name}: {count}",
                x, y, (200, 200, 200) if count > 0 else (100, 100, 100), 13
            )
            y -= line_height
        
        y -= 8
        
        # Hub stats
        arcade.draw_text("── Hub Scores ──", x, y, (150, 150, 150), 12)
        y -= line_height
        
        # Access Hub state from game_state, not match.interactables
        try:
            interactables_state = self.demo.server.match.game_state.get("interactables")
            hub_state = interactables_state.get("Blue Hub") if interactables_state else None
            if hub_state and hasattr(hub_state, 'getValue'):
                active = hub_state.getValue("fuel_scored_active").get()
                inactive = hub_state.getValue("fuel_scored_inactive").get()
                arcade.draw_text(
                    f"🎯 Active: {active}  Inactive: {inactive}",
                    x, y, (100, 255, 100), 13
                )
            else:
                arcade.draw_text("🎯 Hub: Loading...", x, y, (150, 150, 150), 13)
        except Exception as e:
            arcade.draw_text(f"🎯 Hub: Error", x, y, (255, 100, 100), 13)
        y -= line_height
    
    def _get_zone_balls(self, zone_name: str) -> int:
        """Get ball count for a zone."""
        try:
            interactables_state = self.demo.server.match.game_state.get("interactables")
            if interactables_state:
                zone_state = interactables_state.get(zone_name)
                if zone_state and hasattr(zone_state, 'getValue'):
                    balls_val = zone_state.getValue("fuel_available")
                    if balls_val:
                        return balls_val.get()
        except:
            pass
        return 0

class MultiRobotDemo:
    """Multi-robot demonstration with collision avoidance and AI policies."""
    
    def __init__(self):
        # Create game
        self.game = create_rebuilt_game()
        
        # Game server
        self.server = DiscreteGameServer(ServerConfig())
        self.server.load_from_game(self.game)
        
        # Create and add all robots
        self.robots = {}  # name -> SwerveRobot
        self.robot_states = {}  # name -> RobotAnimState
        
        for config in ROBOT_CONFIGS:
            robot = create_robot(config.name, Alliance.BLUE)
            self.robots[config.name] = robot
            self.server.add_robot(robot)
            
            # Initialize robot state
            initial_state = RobotState(config.start_x, config.start_y, Degree(0))
            initial_state.setValue("name", config.name)
            self.server.init_robot(config.name, initial_state)
            
            # Configure interactions for this robot
            self._configure_robot_interactions(robot)
            
            # Create animation state
            self.robot_states[config.name] = RobotAnimState(
                robot_name=config.name,
                config=config,
                current_state=RobotState(config.start_x, config.start_y, Degree(0)),
            )
        
        # Demo state
        self.demo_running = False
        self.match_time = 0.0
        self.match_duration = 160.0  # Full match: 20s auto + 10s transition + 100s teleop + 30s endgame
        
        # Match phase manager for hub activation
        self.phase_manager = MatchPhaseManager()
        
        # Animation layer for projectiles
        self.animation_layer = AnimationLayer()
        
        # Setup renderer
        self.renderer = Renderer.create(game=self.game)
        self.renderer.display_level = DisplayLevel.SHOWCASE
        
        # Add obstacles
        for obs in self.game.get_obstacles():
            self.renderer.add(obs)
        
        # Add bump zones (for visualization)
        for zone in self.game.get_zones():
            self.renderer.add(zone)
        
        # Register render handlers
        from gamegine.render.renderer import ObjectRendererRegistry
        ObjectRendererRegistry.register_handler(AnimationLayer, draw_animation_layer)
        ObjectRendererRegistry.register_handler(ShootingLocation, draw_shooting_location)
        ObjectRendererRegistry.register_handler(Depot, draw_depot)
        ObjectRendererRegistry.register_handler(Hub, draw_hub)
        ObjectRendererRegistry.register_handler(Tower, draw_tower)
        ObjectRendererRegistry.register_handler(AllianceZone, draw_alliance_zone)
        ObjectRendererRegistry.register_handler(NeutralZone, draw_neutral_zone)
        
        # Add interactables
        for interactable in self.game.get_interactables():
            self.renderer.add(interactable)
        
        # Register custom multi-robot drawing
        class MultiRobotDisplay:
            def __init__(self, demo):
                self._demo = demo
            def __call__(self):
                return self._demo
        
        def draw_multi_robots(display_ref, canvas, theme, display_level, renderer=None):
            demo = display_ref()
            if demo is None:
                return
            
            for name, anim_state in demo.robot_states.items():
                config = anim_state.config
                state = anim_state.current_state
                
                x = canvas.to_pixels(state.x.get() if hasattr(state.x, 'get') else state.x)
                y = canvas.to_pixels(state.y.get() if hasattr(state.y, 'get') else state.y)
                
                # Draw robot as colored square
                robot_size = canvas.to_pixels(Inch(28))
                half = robot_size / 2
                
                heading = float((state.heading.get() if hasattr(state.heading, 'get') else state.heading).to(Degree))
                rad = math.radians(heading)
                cos_h, sin_h = math.cos(rad), math.sin(rad)
                
                corners = []
                for dx, dy in [(-1, -1), (1, -1), (1, 1), (-1, 1)]:
                    rx = dx * half
                    ry = dy * half
                    corners.append((
                        x + rx * cos_h - ry * sin_h,
                        y + rx * sin_h + ry * cos_h
                    ))
                
                color = config.color
                arcade.draw_polygon_filled(corners, (*color, 200))
                arcade.draw_polygon_outline(corners, (*color, 255), 2)
                
                # Draw robot name
                arcade.draw_text(
                    name, x, y + robot_size * 0.7,
                    (255, 255, 255), 10, anchor_x="center"
                )
                
                # Draw status indicator
                if anim_state.has_climbed:
                    arcade.draw_text("✓ CLIMBED", x, y - robot_size * 0.7,
                                   (0, 255, 0), 9, anchor_x="center")
                elif anim_state.is_animating:
                    arcade.draw_text("▶ DRIVING", x, y - robot_size * 0.7,
                                   (255, 255, 0), 9, anchor_x="center")
                elif anim_state.is_waiting:
                    arcade.draw_text("⏳ ACTION", x, y - robot_size * 0.7,
                                   (255, 165, 0), 9, anchor_x="center")
        
        ObjectRendererRegistry.register_handler(MultiRobotDisplay, draw_multi_robots)
        self.robot_display = MultiRobotDisplay(self)
        self.renderer.add(self.robot_display)
        self.renderer.add(self.animation_layer)
        
        # Add HUD (top-right corner)
        self.hud = MultiRobotHUD(self)
        ObjectRendererRegistry.register_handler(MultiRobotHUD, lambda obj, canvas, theme, display_level, renderer=None: obj.draw(canvas, theme, display_level, renderer))
        self.renderer.add_dynamic(lambda: self.hud, "ui")
        
        # Register callbacks
        self.renderer.on_update_callback(self.update)
        self.renderer.on_key_press_callback(self.on_key_press)
        
        self.renderer.show_alert(
            "MULTI-ROBOT DEMO | 3 Blue Robots | Press SPACE to start match",
            AlertType.INFO, 5.0
        )
    
    def _configure_robot_interactions(self, robot):
        """Configure robot interaction configs for all interactables."""
        for interactable in self.game.get_interactables():
            nav_point = interactable.get_navigation_point()
            for interaction in interactable.get_interactions():
                config = RobotInteractionConfig(
                    interactable_name=interactable.name,
                    interaction_identifier=interaction.identifier,
                    able_to_interact=lambda *args, **kwargs: True,
                    time_to_interact=lambda *args, **kwargs: 0.5,
                    navigation_point=nav_point,
                )
                robot.add_interaction_config(config)
    
    def on_key_press(self, key, modifiers):
        if key == arcade.key.SPACE:
            if not self.demo_running:
                self.demo_running = True
                self.renderer.show_alert("Match started! Robots are autonomous.", AlertType.SUCCESS, 3.0)
        elif key == arcade.key.R:
            self.reset_demo()
        elif key == arcade.key.ESCAPE:
            arcade.exit()
    
    def reset_demo(self):
        """Reset all robots and match state."""
        self.demo_running = False
        self.match_time = 0.0
        
        for config in ROBOT_CONFIGS:
            name = config.name
            initial_state = RobotState(config.start_x, config.start_y, Degree(0))
            initial_state.setValue("name", name)
            self.server.init_robot(name, initial_state)
            
            self.robot_states[name] = RobotAnimState(
                robot_name=name,
                config=config,
                current_state=RobotState(config.start_x, config.start_y, Degree(0)),
            )
        
        self.server.match.reset()
        self.server.physics_engine.clear_all_active_trajectories()
        
        # Reset phase manager
        self.phase_manager = MatchPhaseManager()
        
        self.renderer.show_alert("Demo reset! Press SPACE to start", AlertType.INFO)
    
    def update(self, dt):
        """Update all robots each frame."""
        if not self.demo_running:
            return
        
        self.match_time += dt
        self.animation_layer.update(dt)
        
        # Update match phase and hub activation
        phase_changed = self.phase_manager.update(self.match_time, self.server.match.game_state)
        if phase_changed:
            phase_name = self.phase_manager.get_phase_name()
            blue_active, red_active = self.phase_manager.get_hub_activation(self.phase_manager.current_phase)
            
            if blue_active and red_active:
                status = "Both hubs ACTIVE"
            elif blue_active:
                status = "🔵 Blue hub ACTIVE"
            elif red_active:
                status = "🔴 Red hub ACTIVE"
            else:
                status = "No hubs active"
            
            self.renderer.show_alert(f"⚡ {phase_name}: {status}", AlertType.WARNING, 2.0)
        
        # Check match end
        if self.match_time >= self.match_duration:
            self.demo_running = False
            self.renderer.show_alert("🎉 Match complete! Press R to reset", AlertType.SUCCESS, 10.0)
            return
        
        # Update each robot
        for name, anim_state in self.robot_states.items():
            self._update_robot(name, anim_state, dt)
    
    def _update_robot(self, name: str, anim_state: RobotAnimState, dt: float):
        """Update a single robot's state."""
        if anim_state.has_climbed:
            return  # Robot is done
        
        if anim_state.is_animating:
            # Update trajectory animation
            anim_state.anim_time += dt
            traj = anim_state.current_trajectory
            
            if traj:
                travel_time = float(traj.get_travel_time().to(Second))
                
                if anim_state.anim_time >= travel_time:
                    # Trajectory complete
                    state = traj.get_at_time(traj.get_travel_time())
                    anim_state.current_state = RobotState(state.x, state.y, state.theta)
                    anim_state.is_animating = False
                    anim_state.is_waiting = True
                    anim_state.wait_time = 1.0  # Action time
                    anim_state.spawn_timer = 0.0
                    
                    # Clear trajectory from collision avoidance
                    self.server.physics_engine.clear_active_trajectory(name)
                else:
                    # Update position along trajectory
                    state = traj.get_at_time(Second(anim_state.anim_time))
                    anim_state.current_state = RobotState(state.x, state.y, state.theta)
        
        elif anim_state.is_waiting:
            # Process waiting/action phase
            anim_state.wait_time -= dt
            anim_state.spawn_timer += dt
            
            # Process animation queue (projectiles)
            remaining = []
            for item in anim_state.action_queue:
                if anim_state.spawn_timer >= item["delay"]:
                    self.animation_layer.add(item["anim"])
                else:
                    remaining.append(item)
            anim_state.action_queue = remaining
            
            if anim_state.wait_time <= 0:
                anim_state.is_waiting = False
                anim_state.actions_performed += 1
                anim_state.action_queue = []
                
                # Check if climbed
                if anim_state.actions_performed > 0 and "climb" in str(getattr(anim_state, '_last_action', '')):
                    anim_state.has_climbed = True
        
        else:
            # Idle - select and execute next action
            self._execute_robot_action(name, anim_state)
    
    def _execute_robot_action(self, name: str, anim_state: RobotAnimState):
        """AI-driven action selection and execution for a robot."""
        config = anim_state.config
        
        # Get robot state from server
        robot_state = self.server.match.game_state.get("robots").get(name)
        if not robot_state:
            return
        
        # Select action based on policy
        action = select_next_action(
            name, robot_state,
            self.server.match.game_state,
            config.policy,
            anim_state.actions_performed,
            anim_state.has_climbed
        )
        
        if action is None:
            anim_state.has_climbed = True
            return
        
        anim_state._last_action = action.interaction
        
        # Capture inventory before action for animation planning
        robot_state = self.server.match.game_state.get("robots").get(name)
        balls_before = 0
        if robot_state:
            inv = robot_state.gamepieces.get() if hasattr(robot_state, 'gamepieces') else {}
            balls_before = inv.get(Fuel, 0) if inv else 0
        
        try:
            # Execute action with collision avoidance
            success, trajectory = self.server.drive_and_process_action(
                action.interactable,
                action.interaction,
                name,
            )
            
            # Get robot destination for animation planning
            robot_pos = None
            if trajectory:
                end_state = trajectory.get_at_time(trajectory.get_travel_time())
                robot_pos = (end_state.x, end_state.y)
            else:
                robot_state = self.server.match.game_state.get("robots").get(name)
                if robot_state:
                    robot_pos = (robot_state.x.get(), robot_state.y.get())
            
            # Plan projectile animations based on inventory change
            if success and robot_pos:
                robot_state = self.server.match.game_state.get("robots").get(name)
                balls_after = 0
                if robot_state:
                    inv = robot_state.gamepieces.get() if hasattr(robot_state, 'gamepieces') else {}
                    balls_after = inv.get(Fuel, 0) if inv else 0
                
                delta = balls_after - balls_before
                self._plan_animations_for_robot(name, anim_state, action, delta, robot_pos)
            
            if trajectory:
                anim_state.current_trajectory = trajectory
                anim_state.anim_time = 0.0
                anim_state.is_animating = True
                
                # Register trajectory for collision avoidance
                robot = self.robots[name]
                self.server.physics_engine.register_active_trajectory(
                    name, trajectory, self.match_time,
                    float(robot.get_bounding_radius().to(Meter))
                )
            else:
                # Already at location - start waiting immediately
                anim_state.is_waiting = True
                anim_state.wait_time = 1.0
                anim_state.spawn_timer = 0.0
                
        except Exception as e:
            print(f"Error executing action for {name}: {e}")
            anim_state.is_waiting = True
            anim_state.wait_time = 0.5
    
    def _plan_animations_for_robot(self, name: str, anim_state: RobotAnimState, 
                                    action: DemoAction, delta: int, robot_pos: Tuple[Any, Any]):
        """Plan projectile animations for a robot based on inventory change."""
        count = abs(delta)
        if count == 0:
            # For shoot actions, show at least 1 ball animation
            if "shoot" in action.interaction:
                count = 1
                delta = -1
        
        if count == 0:
            return
        
        # Get interactable position
        interactable = self.server.match.interactables.get(action.interactable)
        if not interactable:
            return
        
        interactable_pos = self._get_center(interactable)
        if not interactable_pos:
            return
        
        if delta > 0:
            # PICKUP: Interactable -> Robot
            self._queue_burst_for_robot(anim_state, interactable_pos, robot_pos, count, arc_offset=60)
        else:
            # SHOOT/SHUTTLE: Robot -> Target
            # Find target (Hub for shoot, AllianceZone for shuttle)
            target_pos = None
            if "shoot" in action.interaction or "score" in action.interaction:
                for inter in self.game.get_interactables():
                    if isinstance(inter, Hub):
                        target_pos = self._get_center(inter)
                        break
            elif "shuttle" in action.interaction:
                for inter in self.game.get_interactables():
                    if isinstance(inter, AllianceZone) and inter.alliance == Alliance.BLUE:
                        target_pos = self._get_center(inter)
                        break
            
            if not target_pos:
                target_pos = interactable_pos
            
            self._queue_burst_for_robot(anim_state, robot_pos, target_pos, count, arc_offset=30)
    
    def _get_center(self, obj) -> Optional[Tuple[Any, Any]]:
        """Get center coordinates of an object."""
        if hasattr(obj, 'bounds') and hasattr(obj.bounds, 'get_center'):
            c = obj.bounds.get_center()
            return (c[0], c[1])
        if hasattr(obj, 'x') and hasattr(obj, 'y'):
            x = obj.x.get() if hasattr(obj.x, 'get') else obj.x
            y = obj.y.get() if hasattr(obj.y, 'get') else obj.y
            return (x, y)
        return None
    
    def _queue_burst_for_robot(self, anim_state: RobotAnimState, start_pos, end_pos, 
                                count: int, arc_offset: float = 0.0):
        """Queue a burst of projectile animations for a specific robot."""
        for i in range(count):
            delay = i * 0.08 + random.uniform(0.0, 0.05)
            
            sx, sy = start_pos
            ex, ey = end_pos
            
            # Add jitter
            jitter_start = (sx + Inch(random.uniform(-4, 4)), sy + Inch(random.uniform(-4, 4)))
            jitter_end = (ex + Inch(random.uniform(-4, 4)), ey + Inch(random.uniform(-4, 4)))
            
            # Alternate arc direction
            arc_dir = 1 if i % 2 == 0 else -1
            actual_arc = arc_offset * arc_dir + random.uniform(-10, 10)
            
            anim = ProjectileAnimation(jitter_start, jitter_end, duration=0.6, arc_offset=actual_arc)
            anim_state.action_queue.append({"delay": delay, "anim": anim})
        
        # Ensure enough wait time
        min_wait = (count * 0.1) + 0.6
        if anim_state.wait_time < min_wait:
            anim_state.wait_time = min_wait
    
    def run(self):
        """Start the demo."""
        arcade.run()


# =============================================================================
# ENTRY POINT
# =============================================================================

def main():
    print("=" * 70)
    print("REBUILT MULTI-ROBOT DEMO")
    print("=" * 70)
    print("\nThis demo showcases multi-robot coordination with:")
    print("  • 3 Blue alliance robots with different strategies")
    print("  • AI-driven action selection (Cycle, Zone, Defender policies)")
    print("  • Trajectory-based collision avoidance")
    print("  • Zone-based ball storage and shooting locations")
    print("  • Tower climbing for endgame points")
    print()
    print("Controls:")
    print("  SPACE - Start match (robots run autonomously)")
    print("  R     - Reset demo")
    print("  ESC   - Exit")
    print()
    print(f"Match duration: 150 seconds")
    print()
    
    demo = MultiRobotDemo()
    demo.run()


if __name__ == "__main__":
    main()

