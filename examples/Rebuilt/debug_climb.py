from gamegine.simulation.robot import RobotState
from gamegine.simulation.game import GameState
from examples.Rebuilt.scoring import Tower, TowerState, robot_not_climbed_condition, can_climb_level_2_or_3
from gamegine.first.alliance import Alliance
from gamegine.utils.NCIM.ncim import Inch, Meter, Radian

def test_climb_logic():
    print("Testing Climb Logic...")
    
    # Setup
    robot_name = "Blue1"
    robot_state = RobotState(
        x=Meter(0), y=Meter(0), heading=Radian(0),
        alliance=Alliance.BLUE
    )
    # NOTE: NOT setting name manually to verify failure default behavior
    
    tower_state = TowerState()
    game_state = GameState()
    game_state.setValue("current_time", 10.0) # AUTO (<=15)
    game_state.setValue("auto_time", 15.0)
    
    print(f"Robot State keys: {robot_state.values.keys()}")
    
    try:
        # 1. Test robot_not_climbed_condition
        print("\n1. Testing robot_not_climbed_condition...")
        can_climb = robot_not_climbed_condition(tower_state, robot_state, game_state)
        print(f"Result: {can_climb}")
    except Exception as e:
        print(f"FAILED: {e}")
        
    # 2. Inject name and retry
    print("\n2. Injecting name and retrying...")
    robot_state.setValue("name", robot_name)
    try:
        can_climb = robot_not_climbed_condition(tower_state, robot_state, game_state)
        print(f"Result: {can_climb}")
    except Exception as e:
        print(f"FAILED: {e}")

    # 3. Test Full Climb Condition (AUTO)
    print("\n3. Testing can_climb_level_2_or_3 (AUTO)...")
    try:
        can_climb = can_climb_level_2_or_3(tower_state, robot_state, game_state)
        print(f"Allowed? {can_climb}")
    except Exception as e:
         print(f"FAILED: {e}")

if __name__ == "__main__":
    test_climb_logic()
