import unittest
from gamegine.simulation.GameServer import GameServer
from gamegine.representation.game import Game
from gamegine.representation.robot import SwerveRobot
from gamegine.reference.motors import Falcon500, MotorConfig, PowerConfig
from gamegine.reference.gearing import GearSeries, Gear
from gamegine.reference.swerve import SwerveConfig, SwerveModule
from gamegine.utils.NCIM.ncim import Meter, Inch, Pound, NewtonMeter, Ampere
from gamegine.utils.NCIM.ComplexDimensions.electricpot import Volt
from gamegine.utils.NCIM.ComplexDimensions.omega import RotationsPerMinute
from gamegine.simulation.robot import RobotState
from gamegine.utils.NCIM.Dimensions.spatial import SpatialMeasurement

class TestE2E(unittest.TestCase):
    def test_simulation_loop(self):
        # 1. Setup Game
        game = Game("Test Game")
        game.set_field_size(Meter(10), Meter(10))
        
        # 2. Setup Server
        server = GameServer()
        server.load_from_game(game)
        
        # 3. Setup Robot
        from gamegine.reference.motors import MotorConfig, PowerConfig
        from gamegine.reference.gearing import GearSeries
        
        drive_motor = MotorConfig(Falcon500, PowerConfig(Ampere(40)))
        steer_motor = MotorConfig(Falcon500, PowerConfig(Ampere(30)))
        drive_ratio = Gear(1) + Gear(1)
        steer_ratio = Gear(1) + Gear(1)
        
        module = SwerveModule(drive_motor, drive_ratio, steer_motor, steer_ratio)
        drivetrain = SwerveConfig(module) # Default offsets are fine
        from gamegine.representation.robot import PhysicalParameters
        from gamegine.utils.NCIM.ncim import MassMeasurement, MOI, Kilogram, KilogramMetersSquared
        
        physics = PhysicalParameters(
            mass=Kilogram(50),
            moi=KilogramMetersSquared(5)
        )
        robot = SwerveRobot("TestBot", drivetrain, physics=physics)
        server.add_robot(robot)
        
        # 4. Initialize Robot State
        initial_state = RobotState(x=Meter(1), y=Meter(1))
        server.init_robot("TestBot", initial_state)
        
        # 5. Drive Robot
        # This will trigger pathfinding and trajectory generation (PhysicsEngine)
        server.drive_robot("TestBot", Meter(5), Meter(5), 0)
        
        # 6. Run Loop
        for _ in range(10):
            server.update(0.02)
            
        # 7. Verify logic
        robot_state = server.game_state.get("robots").get("TestBot")
        self.assertIsNotNone(robot_state)
        # Check if trajectory was generated
        trajectories = server.get_trajectories("TestBot")
        self.assertTrue(len(trajectories) > 0)

if __name__ == '__main__':
    unittest.main()
