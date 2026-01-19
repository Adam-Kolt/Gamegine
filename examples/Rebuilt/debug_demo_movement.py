
from examples.Rebuilt.discrete_action_demo import DiscreteActionDemo, DEMO_ACTIONS
from gamegine.utils.NCIM.Dimensions.spatial import Inch, Feet
from gamegine.simulation.robot import RobotState
from gamegine.utils.NCIM.Dimensions.angular import Degree

def test_movement():
    print("Initializing Demo...")
    demo = DiscreteActionDemo()
    # Note: demo constructor initializes server and robot
    
    # Manually trigger what happens on reset
    demo.robot_base_name = "BlueBot" 
    start_x = Feet(6)
    start_y = Inch(1654/2) # HALF_WIDTH approx
    
    print(f"Robot Start: ({start_x}, {start_y})")
    
    # 1. Check Interaction Configs
    print("\nChecking Interaction Configs...")
    found_config = False
    for interactable_name, configs in demo.robot.interaction_configs.items():
        for indentifier, config in configs.items():
            if config.interactable_name == "Blue Depot":
                print(f"  Config for Blue Depot: {config.interaction_identifier}")
                print(f"  Nav Point: {config.navigation_point}")
                found_config = True
            
    if not found_config:
        print("  ERROR: No config found for Blue Depot!")
        
    # 2. Try Drive
    print("\nAttempting Drive to Blue Depot...")
    action = DEMO_ACTIONS[0] # Depot pickup
    print(f"Action: {action.interactable} -> {action.interaction}")
    
    try:
        success, trajectory = demo.server.drive_and_process_action(
            action.interactable,
            action.interaction,
            demo.robot_base_name,
        )
        print(f"Success: {success}")
        if trajectory:
            print(f"Trajectory Duration: {trajectory.get_travel_time()}")
            print(f"Start State: {trajectory.start_state.pose.translation}")
            print(f"End State: {trajectory.end_state.pose.translation}")
            
            # Check if start != end
            start = trajectory.start_state.pose.translation
            end = trajectory.end_state.pose.translation
            dist = ((start.x - end.x)**2 + (start.y - end.y)**2)**0.5
            print(f"Distance: {dist}")
        else:
            print("Trajectory is None (Robot already there?)")
            
    except Exception as e:
        print(f"EXCEPTION: {e}")
        import traceback
        traceback.print_exc()

if __name__ == "__main__":
    try:
        test_movement()
    except Exception as e:
        print(f"Main Exception: {e}")
