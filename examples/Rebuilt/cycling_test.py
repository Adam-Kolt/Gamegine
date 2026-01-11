"""Rebuilt Cycling Test - Demonstrates all new features

Shows:
1. Ramp zone slowing down robot traversal
2. Height bar blocking tall robots (short robots pass through)
3. Gamepiece spawning, pickup, and drop
4. Shooting accuracy model
"""

import sys
sys.path.insert(0, '.')

from examples.Rebuilt.Rebuilt import create_rebuilt_game, NEUTRAL_ZONE_PIECES

from gamegine.representation.robot import SwerveRobot, PhysicalParameters
from gamegine.reference import gearing, motors
from gamegine.reference.swerve import SwerveConfig, SwerveModule
from gamegine.simulation.GameServer import DiscreteGameServer
from gamegine.simulation.robot import RobotState
from gamegine.simulation.physics import PhysicsConfig
from gamegine.simulation.gamepiece import GamepieceManager, IntakeConfig
from gamegine.simulation.shooting import ShootingParameters, attempt_shot, ShotOutcome
from gamegine.representation.bounds import Rectangle
from gamegine.utils.NCIM.Dimensions.spatial import Meter, Feet, Inch
from gamegine.utils.NCIM.Dimensions.angular import Degree
from gamegine.utils.NCIM.Dimensions.mass import Pound
from gamegine.utils.NCIM.ComplexDimensions.acceleration import MeterPerSecondSquared
from gamegine.utils.NCIM.ncim import Ampere


# Robot configuration
ROBOT_WIDTH = Inch(30)
ROBOT_HEIGHT_TALL = Feet(4)    # Blocked by bar
ROBOT_HEIGHT_SHORT = Feet(2.5)  # Passes under bar


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


def create_robot(name: str, height: Feet) -> SwerveRobot:
    """Create a swerve robot with configurable height."""
    
    # Robot structure - rectangle with height
    structure = [
        Rectangle.from_center((Inch(0), Inch(0)), ROBOT_WIDTH, ROBOT_WIDTH).get_3d(
            Inch(0), height
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


def main():
    print("=" * 60)
    print("REBUILT CYCLING TEST")
    print("=" * 60)
    
    # Create game
    game = create_rebuilt_game()
    print(f"\nGame: {game.name}")
    print(f"Field: {game.get_field_size()[0]} x {game.get_field_size()[1]}")
    
    # Show zones
    print(f"\nZones ({len(game.get_zones())}):")
    for zone in game.get_zones():
        print(f"  - {zone.name}: {zone.speed_multiplier*100:.0f}% speed")
    
    # Create robots with different heights
    tall_robot = create_robot("TallBot", height=Feet(4))  # 4ft - blocked by bar
    short_robot = create_robot("ShortBot", height=Feet(2.5))  # 2.5ft - passes under bar
    
    print(f"\nRobots:")
    print(f"  - {tall_robot.name}: height={tall_robot.get_height()}")
    print(f"  - {short_robot.name}: height={short_robot.get_height()}")
    
    # Create game server
    from gamegine.simulation.GameServer import ServerConfig
    server = DiscreteGameServer(ServerConfig())
    server.load_from_game(game)
    
    # Register robots at different starting positions
    server.add_robot(tall_robot)
    server.init_robot(tall_robot.name, RobotState(Feet(5), Feet(13.5), Degree(0)))
    
    server.add_robot(short_robot)
    server.init_robot(short_robot.name, RobotState(Feet(49), Feet(13.5), Degree(180)))
    
    # Setup gamepiece manager
    gp_manager = GamepieceManager()
    for i, (x, y) in enumerate(NEUTRAL_ZONE_PIECES):
        piece_id = gp_manager.spawn_piece(x, y, name=f"ball_{i}")
        print(f"  Spawned {piece_id} at ({x}, {y})")
    
    # Intake configuration
    intake = IntakeConfig(
        sides=["front"],
        pickup_radius=Inch(18),
        capacity=3
    )
    
    print("\n" + "=" * 60)
    print("DEMONSTRATION 1: Height Bar Filtering")
    print("=" * 60)
    
    # Check which obstacles apply to each robot
    obstacles = list(game.get_obstacles())
    for robot in [tall_robot, short_robot]:
        print(f"\n{robot.name} (height={robot.get_height()}):")
        for obs in obstacles:
            if hasattr(obs, 'applies_to_robot'):
                applies = obs.applies_to_robot(robot.get_height())
                status = "BLOCKED" if applies else "CAN PASS"
                print(f"  {obs.name}: {status}")
    
    print("\n" + "=" * 60)
    print("DEMONSTRATION 2: Ramp Zone Slowdown")
    print("=" * 60)
    
    # Drive tall robot through ramp zone
    # First, prepare traversal space
    traversal_space = server.physics_engine.prepare_traversal_space(
        tall_robot.name,
        tall_robot,
        list(game.get_obstacles()),
        game.get_field_size(),
    )
    
    # Drive to a point that goes through the ramp
    start_pos = (Feet(5), Feet(13.5), Degree(0))
    target_pos = (Feet(40), Feet(13.5), Degree(0))
    
    print(f"\nDriving {tall_robot.name} from {start_pos[0]} to {target_pos[0]}")
    print("(Path goes through ramp zone at 40% speed)")
    
    # Generate trajectory with zones
    from gamegine.analysis import pathfinding
    path = server.physics_engine.pathfind(
        tall_robot.name,
        start_pos[0], start_pos[1],
        target_pos[0], target_pos[1],
        traversal_space,
    )
    
    trajectory = server.physics_engine.generate_trajectory(
        tall_robot.name,
        tall_robot,
        start_pos,
        target_pos,
        path,
        traversal_space,
        speed_zones=game.get_zones(),  # Apply zone slowdowns!
    )
    
    print(f"Trajectory duration: {trajectory.get_travel_time():.2f}s")
    print(f"Points: {len(trajectory.points)}")
    
    print("\n" + "=" * 60)
    print("DEMONSTRATION 3: Gamepiece Pickup Along Trajectory")
    print("=" * 60)
    
    # Check what pieces would be picked up along a trajectory
    print(f"\nAvailable pieces: {gp_manager.get_available_pieces()}")
    
    # Create a trajectory that passes near gamepieces
    pickup_target = (Feet(27), Feet(13.5), Degree(0))  # Near center pieces
    
    # Prepare for short robot (different traversal space due to height filtering)
    short_traversal = server.physics_engine.prepare_traversal_space(
        short_robot.name,
        short_robot,
        list(game.get_obstacles()),
        game.get_field_size(),
    )
    
    short_path = server.physics_engine.pathfind(
        short_robot.name,
        Feet(49), Feet(13.5),
        pickup_target[0], pickup_target[1],
        short_traversal,
    )
    
    pickup_traj = server.physics_engine.generate_trajectory(
        short_robot.name,
        short_robot,
        (Feet(49), Feet(13.5), Degree(180)),
        pickup_target,
        short_path,
        short_traversal,
    )
    
    # Check for pickups along trajectory
    pickups = gp_manager.check_trajectory_pickups(
        pickup_traj, 
        short_robot.name, 
        intake, 
        current_held=0
    )
    
    print(f"\nPieces that would be picked up along trajectory:")
    for time, piece_id in pickups:
        print(f"  - {piece_id} at t={time:.2f}s")
    
    # Actually pick them up
    for _, piece_id in pickups:
        gp_manager.pickup_piece(piece_id, short_robot.name)
    
    print(f"\nPieces held by {short_robot.name}: {gp_manager.get_robot_pieces(short_robot.name)}")
    print(f"Remaining on field: {gp_manager.get_available_pieces()}")
    
    print("\n" + "=" * 60)
    print("DEMONSTRATION 4: Shooting Accuracy")
    print("=" * 60)
    
    # Setup shooting parameters
    shot_params = ShootingParameters(
        base_accuracy=0.95,
        optimal_range=Meter(3),
        accuracy_falloff_per_meter=0.08,
        max_range=Meter(12),
    )
    
    # Test shots from different distances
    tower_pos = (Feet(2), Feet(13.5))
    
    test_positions = [
        ((Feet(8), Feet(13.5)), "Close range"),
        ((Feet(15), Feet(13.5)), "Medium range"),
        ((Feet(25), Feet(13.5)), "Long range"),
    ]
    
    from gamegine.simulation.shooting import calculate_shot_probability
    
    print("\nShot probability from different distances:")
    for pos, label in test_positions:
        prob = calculate_shot_probability(pos, tower_pos, shot_params)
        print(f"  {label}: {prob*100:.1f}%")
    
    # Attempt shots (deterministic mode for reproducibility)
    print("\nDeterministic shot attempts:")
    for pos, label in test_positions:
        result = attempt_shot(pos, tower_pos, shot_params, deterministic=True)
        status = "✓ SUCCESS" if result == ShotOutcome.SUCCESS else "✗ MISS"
        print(f"  {label}: {status}")
    
    print("\n" + "=" * 60)
    print("ALL DEMONSTRATIONS COMPLETE")
    print("=" * 60)


if __name__ == "__main__":
    main()
