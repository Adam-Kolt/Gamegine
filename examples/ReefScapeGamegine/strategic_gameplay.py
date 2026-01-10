import math
import numpy as np
import time
from typing import Dict, List, Tuple, Optional
from dataclasses import dataclass

# Gamegine imports
from Reefscape import Names, Reefscape, StartingPositionsBlue, get_start_position
from gamepieces import Coral, Algae
# Import Reef from scoring to access ReefState and Reef interaction options
from scoring import ReefState, Reef, ProcessorState, Processor

from gamegine.first.alliance import Alliance
from gamegine.reference import gearing, motors
from gamegine.representation.interactable import RobotInteractionConfig
from gamegine.representation.robot import SwerveRobot, PhysicalParameters
from gamegine.simulation.GameServer import DiscreteGameServer, ServerConfig
from gamegine.simulation.game import GameState
from gamegine.simulation.robot import RobotState
from gamegine.utils.NCIM.Dimensions.angular import Degree, Radian
from gamegine.utils.NCIM.Dimensions.spatial import Inch, Meter
from gamegine.utils.NCIM.ncim import (
    MeterPerSecondSquared,
    Second,
    Kilogram,
    MetersPerSecond,
    KilogramMetersSquared,
)

from gamegine.reference.swerve import SwerveConfig, SwerveModule
from gamegine.reference.motors import MotorConfig, PowerConfig, KrakenX60
from gamegine.utils.NCIM.Dimensions.current import Ampere
from gamegine.representation.bounds import Rectangle, Boundary3D
from gamegine.utils.NCIM.Dimensions.spatial import Feet

# --- Constants ---
ROBOT_LENGTH = Inch(28)
ROBOT_WIDTH = Inch(28)
ROBOT_MASS = Kilogram(50)  # ~110 lbs
ROBOT_MOI = KilogramMetersSquared(5.0)  # kg*m^2 (estimated)

# Robot Configuration
ROBOT_SWERVE = SwerveConfig(
    SwerveModule(
        MotorConfig(
            KrakenX60,
            PowerConfig(Ampere(60), Ampere(240), 1.0),
        ),
        gearing.MK4I.L3,
        MotorConfig(
            KrakenX60,
            PowerConfig(Ampere(60), Ampere(240), 1.0),
        ),
        gearing.MK4I.L3,
    )
)

ROBOT_GEOMETRY = [
    Rectangle.from_center((Inch(0), Inch(0)), ROBOT_WIDTH, ROBOT_LENGTH).get_3d(
        Inch(0), Feet(3)
    )
]

# Interaction Times (s)
CORAL_PICKUP_TIME = 0.5
L1_SCORE_TIME = 0.5
L2_SCORE_TIME = 0.5
L3_SCORE_TIME = 0.5
L4_SCORE_TIME = 0.5
ALGAE_DISLODGE_TIME = 0.5
PROCESSOR_SCORE_TIME = 1.0


def create_robot(name: str, max_speed: MetersPerSecond, mass_multiplier: float = 1.0) -> SwerveRobot:
    """Create a configured SwerveRobot."""
    robot = SwerveRobot(
        name,
        ROBOT_SWERVE,
        ROBOT_GEOMETRY,
        PhysicalParameters(
            ROBOT_MASS * mass_multiplier, 
            ROBOT_MOI, 
            MeterPerSecondSquared(6.0),  # Max acceleration
            MeterPerSecondSquared(6.0),  # Max deceleration
        ),
    )
    robot.override_bounding_radius(Inch(16))
    return robot


def add_robot_interactions(robot: SwerveRobot):
    """Add interaction configurations to a robot."""
    
    # We define simple conditions for the Server's RuleEngine to use.
    # The Strategy class will do more complex selection logic.
    
    def has_no_coral(s, r, g) -> bool:
        return r.gamepieces.get().get(Coral, 0) == 0
        
    def has_coral(s, r, g) -> bool:
        return r.gamepieces.get().get(Coral, 0) > 0

    def has_algae(s, r, g) -> bool:
        return r.gamepieces.get().get(Algae, 0) > 0
        
    def has_no_algae(s, r, g) -> bool:
        return r.gamepieces.get().get(Algae, 0) == 0

    # Coral Pickup
    robot.add_interaction_config(RobotInteractionConfig(Names.TopCoralStation, "PickupCoral", has_no_coral, lambda s,r,g: CORAL_PICKUP_TIME))
    robot.add_interaction_config(RobotInteractionConfig(Names.BottomCoralStation, "PickupCoral", has_no_coral, lambda s,r,g: CORAL_PICKUP_TIME))

    # Reef interaction options from Source of Truth
    reef_options = {opt.identifier: opt for opt in Reef.get_interactions()}
    
    Reef_Position = (Inch(176.746), Reefscape.half_field_y())
    away = Inch(32.5800055942) + ROBOT_LENGTH + Inch(3)
    scoring_columns = ["A", "B", "C", "D", "E", "F", "G", "H", "I", "J", "K", "L"]
    angle_step = Degree(360) / 6
    curr_angle = Degree(180)

    for i, column in enumerate(scoring_columns):
        # Calculate Nav Point
        nav_angle = Degree(180) + curr_angle
        vector = np.array([math.cos(nav_angle.to(Radian)), math.sin(nav_angle.to(Radian))])
        compressed_angle = Radian(np.arctan2(vector[1], vector[0]))
        nav_point = (
            Reef_Position[0] + away * math.cos(curr_angle.to(Radian)),
            Reef_Position[1] + away * math.sin(curr_angle.to(Radian)),
            compressed_angle,
        )

        # Levels
        for level, score_time in [("l1", L1_SCORE_TIME), ("l2", L2_SCORE_TIME), ("l3", L3_SCORE_TIME), ("l4", L4_SCORE_TIME)]:
            identifier = f"{level}_{column}"
            if identifier in reef_options:
                # Use server-side condition + our time
                robot.add_interaction_config(
                    RobotInteractionConfig(
                        Names.Reef,
                        identifier,
                        reef_options[identifier].condition, 
                        lambda s,r,g, t=score_time: t,
                        navigation_point=nav_point,
                    )
                )
        if (i + 1) % 2 == 0:
            curr_angle += angle_step

    # Algae
    # For algae, we just add the config manually for now or iterate
    algae_sides = [["A", "B"], ["C", "D"], ["E", "F"], ["G", "H"], ["I", "J"], ["K", "L"]]
    for side in algae_sides:
        id_dislodge = f"dislodge_{side[0]}{side[1]}"
        if id_dislodge in reef_options:
            robot.add_interaction_config(RobotInteractionConfig(Names.Reef, id_dislodge, reef_options[id_dislodge].condition, lambda s,r,g: ALGAE_DISLODGE_TIME))
        id_pickup = f"pickup_{side[0]}{side[1]}"
        if id_pickup in reef_options:
             robot.add_interaction_config(RobotInteractionConfig(Names.Reef, id_pickup, reef_options[id_pickup].condition, lambda s,r,g: ALGAE_DISLODGE_TIME))

    # Processor
    robot.add_interaction_config(RobotInteractionConfig(Names.Processor, "processor", has_algae, lambda s,r,g: PROCESSOR_SCORE_TIME))


import time
import math
from gamegine.render.renderer import Renderer
from gamegine.render.handlers import *  # Import handlers to register them

# ... (Previous imports)

class SimVisualizer:
    def __init__(self, server: DiscreteGameServer, game: Reefscape):
        self.server = server
        self.game = game
        # Let Renderer auto-size based on game field (approx 1654x821 for Reefscape)
        self.renderer = Renderer.create(game, title="Strategic Gameplay Demo")
        
        # Add static obstacles once
        self.renderer.add_obstacles(self.game.get_obstacles())
            
    def render_static(self):
        """Render static elements (obstacles, etc)."""
        self.renderer.clear_dynamic()
        # Add robots at current server state
        for robot_name, robot in self.server.robots.items():
            robot_state = self.server.match.game_state.get("robots").get(robot_name)
            self.renderer.add_dynamic(lambda rs=robot_state: rs, layer="robot")
            
    def _draw_hud(self, score: int, game_time: float, robot_statuses: dict):
        """Draws the HUD overlay with a background for visibility."""
        # Draw semi-transparent background for HUD
        arcade.draw_lbwh_rectangle_filled(0, self.renderer.height - 80, 400, 80, (255, 255, 255, 200))
        
        # Main Stats
        arcade.draw_text(f"Game Time: {game_time:.1f}s", 10, self.renderer.height - 30, arcade.color.BLACK, 16, bold=True)
        arcade.draw_text(f"Score: {score}", 200, self.renderer.height - 30, arcade.color.DARK_BLUE, 16, bold=True)
        
        # Robot Statuses
        y_offset = self.renderer.height - 55
        for name in sorted(self.server.robots.keys()):
            status = robot_statuses.get(name, "Unknown")
            arcade.draw_text(f"{name}: {status}", 10, y_offset, arcade.color.BLACK, 12)
            y_offset -= 15

    def animate_parallel_step(self, robot_actions: dict, step_duration: float = 0.0):
        """
        Animates multiple robots concurrently.
        """
        if not robot_actions:
            return

        # Determine total time for this animation step
        max_duration = max((a.get('duration', 0) for a in robot_actions.values()), default=1.0)
        total_time = max_duration if step_duration <= 0 else step_duration
        
        anim_time = 0.0
        start_real_time = time.time()
        
        while anim_time < total_time:
            current_real_time = time.time()
            dt = current_real_time - start_real_time
            start_real_time = current_real_time
            anim_time += dt * 1.0 # Realtime speed
            
            self.renderer.clear_dynamic()
            
            valid_statuses = {}
            for r_name in self.server.robots:
                valid_statuses[r_name] = "Ready"

            for robot_name, action in robot_actions.items():
                traj = action.get('trajectory')
                duration = action.get('duration', 0)
                desc = action.get('desc', 'Idle')
                
                # Update Status for HUD
                if anim_time < duration:
                    valid_statuses[robot_name] = f"{desc} ({duration - anim_time:.1f}s)"
                else:
                    valid_statuses[robot_name] = "Ready"

                # Render Robot
                if traj and anim_time < duration:
                    # Robot is moving along trajectory
                    travel_time = traj.get_travel_time().to(Second)
                    sample_time = min(anim_time, travel_time)
                    current_state = traj.get_at_time(Second(sample_time))
                    self.renderer.add_dynamic(lambda s=current_state: s, layer="robot")
                    
                    # Also draw trajectory path
                    # Also draw trajectory path
                    self.renderer.add_dynamic(lambda t=traj: t, layer="trajectory")
                    
                else:
                    # Stationary or finished
                    # In parallel loop, server logic runs FIRST.
                    # So server state is the TARGET state.
                    # If robot is stationary (no traj), it is AT target.
                    # If robot finished moving (anim_time > duration), it is AT target.
                    state = self.server.match.game_state.get("robots").get(robot_name)
                    self.renderer.add_dynamic(lambda s=state: s, layer="robot")

            # Handle robots that had NO action in the dict (just render state)
            for r_name in self.server.robots:
                if r_name not in robot_actions:
                    state = self.server.match.game_state.get("robots").get(r_name)
                    self.renderer.add_dynamic(lambda s=state: s, layer="robot")
                    valid_statuses[r_name] = "Idle"

            self.renderer.dispatch_events()
            self.renderer.on_draw()
            
            # Draw HUD AFTER on_draw (on top)
            server_time = self.server.match.game_state.current_time.get()
            score = self.server.match.game_state.score.get()
            self._draw_hud(score, server_time, valid_statuses)
            
            self.renderer.flip()
            
    def update_display(self, status_text: str):
        """Updates display for a single frame."""
        self.renderer.clear_dynamic()
        for robot_name in self.server.robots:
            robot_state = self.server.match.game_state.get("robots").get(robot_name)
            self.renderer.add_dynamic(lambda s=robot_state: s, layer="robot")
            
        self.renderer.dispatch_events()
        self.renderer.on_draw()

        # Draw HUD AFTER on_draw
        self._draw_hud(self.server.match.game_state.score.get(), 
                       self.server.match.game_state.current_time.get(), 
                       {r: status_text for r in self.server.robots})
        
        self.renderer.flip()



class Strategy:
    def __init__(self, name: str):
        self.name = name
    
    def select_action(self, server: DiscreteGameServer, robot_name: str) -> Optional[Tuple[str, str, str]]:
        """Returns (InteractableName, InteractionName, Description) or None."""
        return None

class ShooterStrategy(Strategy):
    """Prioritizes L4 scoring."""
    def select_action(self, server: DiscreteGameServer, robot_name: str):
        robot_state = server.match.game_state.get("robots").get(robot_name)
        reef_state: ReefState = server.match.game_state.get("interactables").get(Names.Reef)
        
        # Inventory Check
        has_coral = robot_state.gamepieces.get().get(Coral, 0) > 0
        
        if not has_coral:
            return (Names.TopCoralStation, "PickupCoral", "Going to Source")
        
        # Scoring Priority: L4 -> L3
        target_columns = ["A", "B", "G", "H", "C", "D"] # Preferred columns
        
        for level in ["l4", "l3", "l2", "l1"]:
            for col in target_columns:
                if reef_state.is_open(level, col):
                    return (Names.Reef, f"{level}_{col}", f"Scoring {level} {col}")
                    
        return None

class CyclerStrategy(Strategy):
    """Prioritizes fast L2/L3 cycles."""
    def select_action(self, server: DiscreteGameServer, robot_name: str):
        robot_state = server.match.game_state.get("robots").get(robot_name)
        reef_state: ReefState = server.match.game_state.get("interactables").get(Names.Reef)
        
        has_coral = robot_state.gamepieces.get().get(Coral, 0) > 0
        if not has_coral:
            return (Names.BottomCoralStation, "PickupCoral", "Going to Source")
            
        target_columns = ["C", "D", "E", "F", "I", "J"]
        for level in ["l3", "l2", "l1", "l4"]:
             for col in target_columns:
                if reef_state.is_open(level, col):
                    return (Names.Reef, f"{level}_{col}", f"Scoring {level} {col}")
        return None

class FeederStrategy(Strategy):
    """Pickup Algae and Score Processor."""
    def select_action(self, server: DiscreteGameServer, robot_name: str):
        robot_state = server.match.game_state.get("robots").get(robot_name)
        reef_state: ReefState = server.match.game_state.get("interactables").get(Names.Reef)
        
        has_algae = robot_state.gamepieces.get().get(Algae, 0) > 0
        
        if has_algae:
            return (Names.Processor, "processor", "Scoring Processor")
        
        # Find Algae to pickup
        algae_pairs = [["A", "B"], ["C", "D"], ["E", "F"], ["G", "H"], ["I", "J"], ["K", "L"]]
        algae_row = reef_state.get("algae")
        
        for pair in algae_pairs:
            # Check if algae exists (using 'A' or 'B' check from scoring.py logic)
            # scoring.py: __agae_left_condition checks column.
            # Algae is stored in 'algae' ReefRow. True = present.
            if algae_row.get_column(pair[0]).get():
                 return (Names.Reef, f"pickup_{pair[0]}{pair[1]}", f"Picking Algae {pair[0]}{pair[1]}")
                 
        return None


def run_strategic_gameplay():
    print("Initialize Server...")
    server = DiscreteGameServer(ServerConfig(fast_mode=True)) # Fast mode for logic check
    
    # 1. Setup Game
    # Reefscape is an instance, not a class
    game = Reefscape
    server.load_from_game(game)
    
    # 2. Add Robots
    robots = {}
    strategies = {}
    
    r1 = create_robot("Blue1", MetersPerSecond(4.5))
    add_robot_interactions(r1)
    server.add_robot(r1)
    # StartingPositionsBlue.A is a tuple (x, y)
    server.init_robot("Blue1", RobotState(*StartingPositionsBlue.A, heading=Degree(180)))
    strategies["Blue1"] = ShooterStrategy("Shooter")
    

    
    # Give initial coral to Shooter/Cycler
    server.match.pickup_gamepiece("Blue1", Coral)

    
    # 3. Setup Visualization
    vis = SimVisualizer(server, game)
    
    print("Starting Main Loop...")
    step = 0
    # max_steps = 20 # Removed limit

    while not server.match.is_game_over():
        step += 1
        current_time = server.match.game_state.current_time.get()
        print(f"\n--- Step {step} (Time: {current_time:.2f}s) ---")
        
        vis.update_display("Planning...")
        
        # 1. PLAN & EXECUTE PHASE
        # We collect actions for parallel animation
        step_actions = {} 
        
        for name in ["Blue1"]:
            strat = strategies[name]
            action = strat.select_action(server, name)
            
            if action:
                interactable, interaction, desc = action
                
                # Dynamic Duration Calculation
                robot_obj = server.robots[name]
                robot_config = robot_obj.interaction_configs.get(interactable, {}).get(interaction)
                
                duration = 1.0
                if robot_config:
                    g_state = server.match.game_state
                    r_state = g_state.get("robots").get(name)
                    i_state = g_state.get("interactables").get(interactable)
                    duration = robot_config.time_to_interact(i_state, r_state, g_state)
                
                print(f"  {name} [{strat.name}]: {desc} (Duration: {duration:.2f}s)")
                
                # Check for navigation needs
                nav_point = server.match.get_navigation_point(interactable, interaction, name)
                
                trajectory = None
                if nav_point:
                     # Drive (Logic Update)
                     # Note: This updates server time immediately
                     trajectory = server.drive_robot(name, nav_point[0], nav_point[1], nav_point[2])
                
                if trajectory:
                    # Robot is moving
                    step_actions[name] = {
                        'trajectory': trajectory,
                        'duration': trajectory.get_travel_time().to(Second),
                        'desc': desc
                    }
                else:
                    # Robot is already there, process interaction
                    success = server.process_action(interactable, interaction, name)
                    if success:
                         print(f"    -> SUCCESS")
                         # Note: process_action already updates server time based on config
                         
                         step_actions[name] = {
                            'trajectory': None,
                            'duration': duration,
                            'desc': desc
                         }
                    else:
                         print(f"    -> FAILED to process {interaction}")
                         step_actions[name] = {
                            'trajectory': None,
                            'duration': 0.5,
                            'desc': "Failed Action"
                         }
            else:
                 print(f"  {name}: IDLE (No valid action)")
                 step_actions[name] = {
                    'trajectory': None,
                    'duration': 0.5,
                    'desc': "Idle"
                 }

        # 2. ANIMATE PHASE
        # Animate all robots concurrently based on what they did this step
        vis.animate_parallel_step(step_actions)
                 
    # Print Final Score
    print("\n--- Match Ended ---")
    print(f"Final Score: {server.match.game_state.score.get()}")
                 
if __name__ == "__main__":
    run_strategic_gameplay()
