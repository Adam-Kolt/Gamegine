
from gamegine.rl import make_solo_env, RobotConfig
from gamegine.first.alliance import Alliance
from gamegine.simulation.robot import RobotState
from examples.ReefScapeGamegine.Reefscape import Reefscape
from examples.ReefScapeGamegine.ai_robot import SWERVE_ROBOT, init_robot_interaction
from gamegine.utils.NCIM.ncim import Inch, Degree
from examples.ReefScapeGamegine.scoring import Reef, Processor, Barge, Net
import time

# Init
init_robot_interaction()

def create_robot_configs(alliance: Alliance, num_robots: int = 1):
    configs = []
    for i in range(num_robots):
        start_state = RobotState(
            x=Inch(100),
            y=Inch(100 + i * 50),
            heading=Degree(0)
        )
        configs.append(RobotConfig(SWERVE_ROBOT, start_state, f"{alliance.name.lower()}_{i}", alliance.name.lower()))
    return configs

# Create env
robots = create_robot_configs(Alliance.BLUE, 1)
env = make_solo_env(
    game=Reefscape,
    robots=robots,
    alliance="blue",
    fast_mode=True,
    use_server_pool=True,
)

print(f"Env created. Max steps: {env._max_episode_steps}")

def extract_game_metrics(env):
    """Prototype for metric extraction logic."""
    metrics = {
        "coral_l1": 0,
        "coral_l2": 0,
        "coral_l3": 0,
        "coral_l4": 0,
        "algae_processed": 0,
        "algae_net": 0,
        "barge_climbs": 0,
    }
    
    # Iterate all interactables in the game
    # Access state via server.match.game_state.get(interactable_name) presumably
    # Or server.match.game_state holds the tree.
    
    # Actually, in Gamegine, interactables are stored in server.game.interactables
    # But their STATE is in server.match.game_state
    
    # Let's inspect how to access state for a key.
    # StateSpace.get(key) returns child Space or Value.
    
    game_state = env.server.match.game_state
    
    # Reefscape interactables names are like "Blue Reef", "Red Processor", etc.
    # We need to find them.
    
    if not hasattr(env.game, 'get_interactables'):
        print("Error: Game has no get_interactables method")
        return metrics
        
    for interactable in env.game.get_interactables():
        # Get the state for this interactable
        # The key in game_state should match interactable.name
        try:
            # We need to check if game_state has value for this interactable.
            # StateSpace stores values in ._values (dict) and subspaces in ._spaces (dict)
            # But we should use public methods.
            # Assuming interactable name is the key.
            if interactable.name not in game_state.spaces:
                # Maybe nested? Or name mismatch?
                # Reefscape adds them to root game_state?
                # Usually: game_state.registerSpace(interactable.name, interactable.initializeInteractableState())
                continue
                
            i_state = game_state.get(interactable.name)
            
            # Check type based on class or content
            # We can import Reef, Processor, etc.
            
            if isinstance(interactable, Reef):
                # Count L1
                l1_row = i_state.get("l1")
                for col in ["A", "B", "C", "D", "E", "F", "G", "H", "I", "J", "K", "L"]:
                    metrics["coral_l1"] += l1_row.get_column(col).get()
                
                # Count L2-L4
                for level in ["l2", "l3", "l4"]:
                    row = i_state.get(level)
                    for col in ["A", "B", "C", "D", "E", "F", "G", "H", "I", "J", "K", "L"]:
                        if row.get_column(col).get():
                            metrics[f"coral_{level}"] += 1
                            
            elif isinstance(interactable, Processor):
                metrics["algae_processed"] += i_state.processor.get()
                
            elif isinstance(interactable, Net):
                metrics["algae_net"] += i_state.ball_amount.get()
                
            elif isinstance(interactable, Barge):
                for cage in ["cage_1", "cage_2", "cage_3"]:
                    if i_state.getValue(cage).get():
                        metrics["barge_climbs"] += 1
                        
        except Exception as e:
            print(f"Error extracting metric for {interactable.name}: {e}")
            
    return metrics

# Run loop
env.reset()
print("Reset done.")

start = time.time()
steps = 0
episodes = 0

for i in range(20):
    actions = {agent: env.action_space[agent].sample() for agent in env.agents}
    obs, rewards, terminations, truncations, infos = env.step(actions)
    steps += 1
    
    if i % 10 == 0:
        metrics = extract_game_metrics(env)
        print(f"Step {i}: Metrics={metrics}")
        
    if any(truncations.values()) or any(terminations.values()):
        print(f"Episode finished at step {i+1}")
        episodes += 1
        env.reset()

env.close()
