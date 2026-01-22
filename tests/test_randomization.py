import sys
import os
print("Started test script", file=sys.stderr)
sys.path.insert(0, os.getcwd())
print(f"CWD: {os.getcwd()}", file=sys.stderr)

from gamegine.first.alliance import Alliance
from gamegine.rl import make_alliance_env, RobotConfig
from examples.Rebuilt.Rebuilt import create_rebuilt_game, FIELD_LENGTH, FIELD_WIDTH
from examples.Rebuilt.robot import create_robot, setup_robot_interactions
from gamegine.simulation.robot import RobotState
from gamegine.utils.NCIM.ncim import Inch, Degree
from gamegine.utils.NCIM.Dimensions.spatial import Meter
from gamegine.rl.config import TrainingConfig
import numpy as np

def create_robot_configs(alliance, game):
    configs = []
    team_str = "red" if alliance == Alliance.RED else "blue"
    name = f"{team_str.capitalize()}1"
    robot = create_robot(name, alliance)
    setup_robot_interactions(robot, game)
    start_state = RobotState(x=Inch(0), y=Inch(0), heading=Degree(0), alliance=alliance)
    configs.append(RobotConfig(robot=robot, start_state=start_state, name=name, team=team_str))
    return configs

game = create_rebuilt_game()
red_robots = create_robot_configs(Alliance.RED, game)
blue_robots = create_robot_configs(Alliance.BLUE, game)

env = make_alliance_env(game=game, red_robots=red_robots, blue_robots=blue_robots, mode="self_play")
env.training_config.randomize_start_pose = True

print("Resetting environment...")
try:
    env.reset()
    print("Reset successful!")
    
    # Check positions
    # Check positions
    game_state = env.server.match.game_state
    for name in ["Blue1", "Red1"]:
        robot = game_state.get_robot(name)
        if robot:
            x_val = float(robot.x.get().to(Meter))
            y_val = float(robot.y.get().to(Meter))
            print(f"Robot {name}: x={x_val}, y={y_val}")
        
    # Verify 5 resets
    for i in range(5):
        env.reset()
        robot = game_state.get_robot("Blue1")
        if robot:
            print(f"Reset {i+1}: Blue1 x={robot.x.get().to(Meter)}")

except Exception as e:
    import traceback
    traceback.print_exc()
