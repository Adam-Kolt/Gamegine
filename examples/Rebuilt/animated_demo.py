"""Rebuilt Animated Demo - Visual demonstration of all features

Shows robot driving over ramp zone (slow), picking up gamepieces,
with animated trajectory following and visual state updates.
"""

import sys
sys.path.insert(0, '.')

from examples.Rebuilt.Rebuilt import create_rebuilt_game, ALL_PIECES, HALF_LENGTH, HALF_WIDTH

from gamegine.representation.robot import SwerveRobot, PhysicalParameters
from gamegine.reference import gearing, motors
from gamegine.reference.swerve import SwerveConfig, SwerveModule
from gamegine.simulation.GameServer import DiscreteGameServer, ServerConfig
from gamegine.simulation.robot import RobotState
from gamegine.simulation.gamepiece import GamepieceManager, IntakeConfig
from gamegine.representation.bounds import Rectangle
from gamegine.utils.NCIM.Dimensions.spatial import Meter, Feet, Inch
from gamegine.utils.NCIM.Dimensions.angular import Degree
from gamegine.utils.NCIM.Dimensions.mass import Pound
from gamegine.utils.NCIM.Dimensions.temporal import Second
from gamegine.utils.NCIM.ComplexDimensions.acceleration import MeterPerSecondSquared
from gamegine.utils.NCIM.ncim import Ampere

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


def create_robot(name: str, height_ft: float) -> SwerveRobot:
    """Create a swerve robot with configurable height."""
    structure = [
        Rectangle.from_center((Inch(0), Inch(0)), ROBOT_WIDTH, ROBOT_WIDTH).get_3d(
            Inch(0), Feet(height_ft)
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
    return robot


# =============================================================================
# ZONE RENDERER - Draw traversal zones with color
# =============================================================================

def draw_zone(zone, canvas, theme, display_level, renderer=None):
    """Draw a traversal zone as a semi-transparent colored region."""
    from gamegine.representation.bounds import Rectangle
    
    bounds = zone.boundary
    if isinstance(bounds, Rectangle):
        x = canvas.to_pixels(bounds.x)
        y = canvas.to_pixels(bounds.y)
        w = canvas.to_pixels(bounds.width)
        h = canvas.to_pixels(bounds.height)
        
        # Slow zones: orange/yellow, fast zones: green
        if zone.speed_multiplier < 0.6:
            color = (255, 165, 0, 80)  # Orange - slow zone
        elif zone.speed_multiplier < 1.0:
            color = (255, 255, 0, 60)  # Yellow - medium
        else:
            color = (100, 255, 100, 60)  # Green - normal/fast
        
        points = [(x, y), (x + w, y), (x + w, y + h), (x, y + h)]
        arcade.draw_polygon_filled(points, color)
        
        # Zone label
        arcade.draw_text(
            f"{zone.name}\n{int(zone.speed_multiplier*100)}% speed",
            x + w/2, y + h/2,
            (255, 255, 255, 200),
            12,
            anchor_x="center",
            anchor_y="center",
        )


# =============================================================================
# MAIN DEMO
# =============================================================================

class AnimatedDemo:
    """Animated demonstration of Rebuilt features."""
    
    def __init__(self):
        # Create game
        self.game = create_rebuilt_game()
        
        # Create robots
        self.robot = create_robot("DemoBot", height_ft=2.5)
        self.robot_radius = Inch(16)
        
        # Game server
        self.server = DiscreteGameServer(ServerConfig())
        self.server.load_from_game(self.game)
        self.server.add_robot(self.robot)
        self.server.init_robot(
            self.robot.name, 
            RobotState(Feet(5), Feet(13.5), Degree(0))
        )
        
        # Gamepiece manager - spawn all field pieces
        self.gp_manager = GamepieceManager()
        for i, (x, y) in enumerate(ALL_PIECES):
            self.gp_manager.spawn_piece(x, y, name=f"fuel_{i}")
        
        # Intake config - FRONT intake only
        self.intake = IntakeConfig(
            sides=["front"],
            pickup_radius=Inch(24),
            capacity=3
        )
        
        # Animation state
        self.current_trajectory = None
        self.anim_time = 0.0
        self.current_robot_state = RobotState(Feet(5), Feet(13.5), Degree(0))
        self.is_animating = False
        self.pending_pickups = []
        self.picked_up_this_traj = set()
        
        # Intake state - starts OFF
        self.intake_running = False
        
        # Demo sequence - drive around the field
        self.demo_phase = 0
        self.phase_targets = [
            (HALF_LENGTH, HALF_WIDTH, Degree(0)),        # Drive to center (Hub area)
            (HALF_LENGTH + Inch(96), HALF_WIDTH, Degree(0)),  # Drive to blue side
            (HALF_LENGTH, HALF_WIDTH - Inch(60), Degree(90)), # Drive to lower pieces
            (HALF_LENGTH - Inch(96), HALF_WIDTH, Degree(180)), # Drive to red side
        ]
        
        # Setup renderer
        self.renderer = Renderer.create(game=self.game)
        self.renderer.display_level = DisplayLevel.SHOWCASE
        
        # Add obstacles
        for obs in self.game.get_obstacles():
            self.renderer.add(obs)
        
        # Add zones
        for zone in self.game.get_zones():
            from gamegine.render.renderer import ObjectRendererRegistry
            from gamegine.representation.zone import TraversalZone
            ObjectRendererRegistry.register_handler(TraversalZone, draw_zone)
            self.renderer.add(zone)
        
        # Add gamepiece manager
        self.renderer.add_dynamic(lambda: self.gp_manager, "gamepiece")
        
        # Track robot
        self.renderer.track_robot(lambda: self.current_robot_state)
        
        # Register update callback
        self.renderer.on_update_callback(self.update)
        
        # Show initial alert
        self.renderer.show_alert("Press SPACE to start | I=Toggle Intake | S=Shoot", AlertType.INFO, 5.0)
        
        # Key handling
        self.renderer.on_key_press_callback(self.on_key_press)
    
    def on_key_press(self, key, modifiers):
        """Handle key presses."""
        if key == arcade.key.SPACE and not self.is_animating:
            self.start_next_phase()
        elif key == arcade.key.R:
            self.demo_phase = 0
            self.reset_robot_position()
            self.renderer.show_alert("Demo reset", AlertType.INFO)
        elif key == arcade.key.I:
            # Toggle intake
            self.intake_running = not self.intake_running
            self.gp_manager.set_intake_active(self.robot.name, self.intake_running)
            status = "ON" if self.intake_running else "OFF"
            self.renderer.show_alert(f"Intake: {status}", AlertType.INFO, 1.0)
        elif key == arcade.key.S:
            # Shoot first held piece toward center of field
            held = self.gp_manager.get_robot_pieces(self.robot.name)
            if held:
                piece_id = held[0]
                result = self.gp_manager.shoot_piece(
                    piece_id,
                    target_x=Feet(27),
                    target_y=Feet(13.5),
                    shooter_x=self.current_robot_state.x.get(),
                    shooter_y=self.current_robot_state.y.get(),
                )
                if result.success:
                    self.renderer.show_alert(f"Shot {piece_id} -> target!", AlertType.SUCCESS, 1.5)
                else:
                    self.renderer.show_alert(f"Shot {piece_id} - MISSED!", AlertType.WARNING, 1.5)
            else:
                self.renderer.show_alert("No pieces to shoot!", AlertType.WARNING, 1.0)
    
    def reset_robot_position(self):
        """Reset robot to starting position."""
        self.current_robot_state = RobotState(Feet(5), Feet(13.5), Degree(0))
        self.is_animating = False
        self.current_trajectory = None
        self.intake_running = False
        self.gp_manager.set_intake_active(self.robot.name, False)
    
    def start_next_phase(self):
        """Start the next phase of the demo."""
        if self.demo_phase >= len(self.phase_targets):
            self.renderer.show_alert("Demo complete! Press R to reset", AlertType.SUCCESS)
            return
        
        # Turn on intake when starting movement
        self.intake_running = True
        self.gp_manager.set_intake_active(self.robot.name, True)
        self.renderer.show_alert("Intake: ON (auto)", AlertType.INFO, 1.0)
        
        target = self.phase_targets[self.demo_phase]
        self.start_drive_to(target[0], target[1], target[2])
        self.demo_phase += 1
    
    def start_drive_to(self, x, y, heading):
        """Generate trajectory and start animation."""
        start_x = self.current_robot_state.x.get()
        start_y = self.current_robot_state.y.get()
        start_heading = self.current_robot_state.heading.get()
        
        traversal_space = self.server.physics_engine.prepare_traversal_space(
            self.robot.name,
            self.robot,
            list(self.game.get_obstacles()),
            self.game.get_field_size(),
        )
        
        path = self.server.physics_engine.pathfind(
            self.robot.name,
            start_x, start_y,
            x, y,
            traversal_space,
        )
        
        self.current_trajectory = self.server.physics_engine.generate_trajectory(
            self.robot.name,
            self.robot,
            (start_x, start_y, start_heading),
            (x, y, heading),
            path,
            traversal_space,
            speed_zones=self.game.get_zones(),
        )
        
        # Pre-calculate pickups (only if intake will be on)
        self.pending_pickups = self.gp_manager.check_trajectory_pickups(
            self.current_trajectory,
            self.robot.name,
            self.intake,
            current_held=len(self.gp_manager.get_robot_pieces(self.robot.name))
        )
        self.picked_up_this_traj = set()
        
        self.anim_time = 0.0
        self.is_animating = True
        
        travel_time = float(self.current_trajectory.get_travel_time().to(Second))
        self.renderer.show_alert(
            f"Driving to ({float(x.to(Feet)):.0f}ft, {float(y.to(Feet)):.0f}ft) - {travel_time:.1f}s",
            AlertType.INFO
        )
    
    def update(self, dt):
        """Update animation each frame."""
        # Update gamepiece physics (velocity/friction)
        self.gp_manager.update_physics(dt)
        
        if not self.is_animating or self.current_trajectory is None:
            return
        
        self.anim_time += dt
        travel_time = float(self.current_trajectory.get_travel_time().to(Second))
        
        # Get current robot state
        if self.anim_time >= travel_time:
            state = self.current_trajectory.get_at_time(self.current_trajectory.get_travel_time())
            self.current_robot_state = RobotState(state.x, state.y, state.theta)
            self.is_animating = False
            
            # Turn off intake when stopped
            self.intake_running = False
            self.gp_manager.set_intake_active(self.robot.name, False)
            
            held = len(self.gp_manager.get_robot_pieces(self.robot.name))
            self.renderer.show_alert(
                f"Arrived! Holding {held} pieces. SPACE=continue, S=shoot",
                AlertType.SUCCESS
            )
        else:
            state = self.current_trajectory.get_at_time(Second(self.anim_time))
            self.current_robot_state = RobotState(state.x, state.y, state.theta)
        
        rx = self.current_robot_state.x.get()
        ry = self.current_robot_state.y.get()
        heading = self.current_robot_state.heading.get()
        
        # If intake is ON, try to pick up pieces on intake side
        if self.intake_running:
            picked = self.gp_manager.try_pickup_with_intake(
                self.robot.name, rx, ry, heading, self.intake
            )
            if picked:
                self.renderer.show_alert(f"Picked up {picked}!", AlertType.SUCCESS, 1.5)
        else:
            # If intake is OFF, push pieces away
            pushed = self.gp_manager.push_pieces_from_robot(rx, ry, self.robot_radius)
            if pushed:
                self.renderer.show_alert(f"Pushed {len(pushed)} piece(s)", AlertType.WARNING, 0.8)
    
    def get_robot_position(self, robot_name):
        """Get current robot position (for gamepiece rendering)."""
        if robot_name == self.robot.name:
            return (self.current_robot_state.x.get(), self.current_robot_state.y.get())
        return None
    
    def run(self):
        """Run the demo."""
        self.renderer.get_robot_position = self.get_robot_position
        arcade.run()


def main():
    print("=" * 60)
    print("REBUILT ANIMATED DEMO - Enhanced Physics")
    print("=" * 60)
    print("\nControls:")
    print("  SPACE - Start/continue demo (auto-enables intake)")
    print("  I     - Toggle intake ON/OFF")
    print("  S     - Shoot held piece to center of field")
    print("  R     - Reset demo")
    print()
    print("Physics:")
    print("  - Pieces only picked up when intake ON + front side contact")
    print("  - Pieces get pushed away when intake OFF")
    print("  - Pushed pieces have velocity and friction")
    print()
    
    demo = AnimatedDemo()
    demo.run()


if __name__ == "__main__":
    main()

