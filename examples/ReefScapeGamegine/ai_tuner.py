import gymnasium as gym
from gymnasium.wrappers import FlattenObservation
from gymnasium import spaces
import logging
from typing import List
import numpy as np
import time
import pygame
import torch
from Reefscape import (
    CageLevel,
    Names,
    Reefscape,
    StartingPositionsBlue,
    add_cage_obstacles,
)
from ai_robot import (
    START_LOCATION,
    START_ROTATION,
    SWERVE_ROBOT,
    init_robot_interaction,
)
from gamegine.first.alliance import Alliance
from gamegine.reference import gearing, motors
from gamegine.reference.swerve import SwerveConfig, SwerveModule
from gamegine.render.style import Palette
from gamegine.representation.bounds import Circle, DiscreteBoundary, Point
from gamegine.representation.gamepiece import Gamepiece, GamepiecePhysicalProperties
from gamegine.representation.interactable import InteractionOption, RobotInteractable
from gamegine.simulation.game import GameState
from gamegine.simulation.robot import RobotState
from gamegine.simulation.state import StateSpace, ValueChange, ValueEntry, ValueIncrease
from stable_baselines3 import DQN, PPO
from stable_baselines3.common.env_util import make_vec_env
from stable_baselines3.common.evaluation import evaluate_policy
from stable_baselines3.common.callbacks import EvalCallback
from stable_baselines3.common.monitor import Monitor
import numpy as np
import os
from gamegine.utils.NCIM.Dimensions.spatial import Inch, Meter, SpatialMeasurement
from gamegine.utils.NCIM.Dimensions.mass import Ounce, Pound

from gamegine.render.renderer import Renderer
from gamegine.representation.bounds import (
    CircularPattern,
    Cylinder,
    Polygon,
    Rectangle,
    SymmetricalX,
    Transform3D,
)
from gamegine.representation.game import Game
from gamegine.representation.interactable import RobotInteractionConfig
from gamegine.representation.obstacle import Obstacle
from gamegine.representation.robot import (
    PhysicalParameters,
    SwerveRobot,
)
from gamegine.simulation.GameServer import GameServer
from gamegine.simulation.robot import RobotState
from gamegine.utils.NCIM.ncim import (
    Inch,
    Pound,
    Ampere,
    Feet,
    Degree,
    Second,
    TemporalMeasurement,
)
from gamegine.utils.logging import GetLogger
from gamepieces import Algae, Coral
from scoring import ReefState
import matplotlib.pyplot as plt
import matplotlib

matplotlib.pyplot.set_loglevel (level = 'warning')


GetLogger().setLevel(logging.CRITICAL)

plt.ion()

# Create a figure and axes
fig, ax = plt.subplots(figsize=(6, 4))


class ReefBotEnvironment(gym.Env):
    game_instance = GameServer()

    def create_observation_space(self):
        # Define the column space for ReefRow and L1Row (boolean or integer columns)
        column_space_l1 = spaces.Box(
            low=0, high=3, shape=(12,), dtype=np.int32
        )  # Columns A-L for L1Row (integer values, 0 to 3)
        column_space_reef = spaces.MultiBinary(
            12
        )  # Columns A-L for ReefRow (boolean values)

        # Define observation space for ReefState
        reef_state_space = spaces.Dict(
            {
                "l1": spaces.Dict(
                    {
                        "row_score": spaces.Box(
                            low=0, high=72, shape=(), dtype=np.float32
                        ),  # row_score for L1Row
                        "columns": column_space_l1,  # Columns A-L
                    }
                ),
                "l2": spaces.Dict(
                    {
                        "row_score": spaces.Box(
                            low=0, high=60, shape=(), dtype=np.float32
                        ),  # row_score for ReefRow
                        "columns": column_space_reef,  # Columns A-L
                    }
                ),
                "l3": spaces.Dict(
                    {
                        "row_score": spaces.Box(
                            low=0, high=72, shape=(), dtype=np.float32
                        ),  # row_score for ReefRow
                        "columns": column_space_reef,  # Columns A-L
                    }
                ),
                "l4": spaces.Dict(
                    {
                        "row_score": spaces.Box(
                            low=0, high=84, shape=(), dtype=np.float32
                        ),  # row_score for ReefRow
                        "columns": column_space_reef,  # Columns A-L
                    }
                ),
                "algae": spaces.Dict(
                    {
                        "row_score": spaces.Box(
                            low=0, high=100, shape=(), dtype=np.float32
                        ),  # row_score for ReefRow
                        "columns": column_space_reef,  # Columns A-L
                    }
                ),
            }
        )

        # Define observation space for GameState
        game_state_space = spaces.Dict(
            {
                "score": spaces.Box(
                    low=0, high=300, shape=(), dtype=np.int32
                ),  # Game score
                "auto_time": spaces.Box(
                    low=0, high=500, shape=(), dtype=np.int32
                ),  # Autonomous mode time
                "teleop_time": spaces.Box(
                    low=0, high=500, shape=(), dtype=np.int32
                ),  # Teleoperation mode time
                "endgame_time": spaces.Box(
                    low=0, high=500, shape=(), dtype=np.int32
                ),  # Endgame time
                "current_time": spaces.Box(
                    low=0, high=500, shape=(), dtype=np.float32
                ),  # Current time
            }
        )

        # Define observation space for RobotState
        robot_state_space = spaces.Dict(
            {
                "x": spaces.Box(
                    low=0, high=17.5, shape=(), dtype=np.float32
                ),  # x-coordinate
                "y": spaces.Box(
                    low=0, high=8, shape=(), dtype=np.float32
                ),  # y-coordinate
                "heading": spaces.Box(
                    low=-360, high=360, shape=(), dtype=np.float32
                ),  # Heading (angle in radians)
                "alliance": spaces.Discrete(2),  # Alliance (0 for RED, 1 for BLUE)
                "gamepieces": spaces.Dict(
                    {  # Gamepieces the robot is holding
                        "Coral": spaces.Box(
                            low=0, high=2, shape=(), dtype=np.int32
                        ),  # Number of coral gamepieces
                        "Algae": spaces.Box(
                            low=0, high=2, shape=(), dtype=np.int32
                        ),  # Number of algae gamepieces
                    }
                ),
            }
        )

        # Combine all spaces into a single observation space
        observation_space = spaces.Dict(
            {
                "reef_state": reef_state_space,
                "game_state": game_state_space,
                "robot_state": robot_state_space,
            }
        )

        return observation_space

    def __setup_server(self):
        self.game_instance.load_from_game(Reefscape)

        self.game_instance.init_robot(
            "DunceBot",
            RobotState(
                *START_LOCATION,
                START_ROTATION,
                gamepieces={Coral: 1, Algae: 0},
            ),
        )
        self.game_instance.game_state.get("robots").get("DunceBot").setValue(
            "gameover", False
        )

        init_robot_interaction()

    def __get_action_space(self):
        robot_actions = self.game_instance.get_actions_set("DunceBot")
        return gym.spaces.Discrete(len(robot_actions))

    def __init__(self):
        self.__setup_server()
        self.game_instance.add_robot(SWERVE_ROBOT)
        self.last_action = 0
        self.current_trajectory = None  # Captured during step for render
        self.action_space = self.__get_action_space()
        self.observation_space = self.create_observation_space()
        self.reward_range = (0, 400)
        self.bad_count = 0
        self.prev_time = 0
        self.prev_gamepieces = (
            self.game_instance.game_state.get("robots").get("DunceBot").gamepieces.get()
        )

        self.renderer = Renderer.create(game=Reefscape)

    def reset(self, seed=None, options=None):
        self.__setup_server()
        self.bad_count = 0
        return {
            "reef_state": self.__get_reef_observation(),
            "game_state": self.__get_game_observation(),
            "robot_state": self.__get_robot_observation(),
        }, {}

    def __get_reef_observation(self):
        reef_state: ReefState = self.game_instance.game_state.get("interactables").get(
            Names.Reef
        )
        observation = {
            "l1": {
                "row_score": reef_state.l1.get(),
                "columns": reef_state.get("l1").get_binary_list(),
            },
            "l2": {
                "row_score": reef_state.l2.get(),
                "columns": reef_state.get("l2").get_binary_list(),
            },
            "l3": {
                "row_score": reef_state.l3.get(),
                "columns": reef_state.get("l3").get_binary_list(),
            },
            "l4": {
                "row_score": reef_state.l4.get(),
                "columns": reef_state.get("l4").get_binary_list(),
            },
            "algae": {
                "row_score": reef_state.get("algae").row_score.get(),
                "columns": reef_state.get("algae").get_binary_list(),
            },
        }

        return observation

    def __get_game_observation(self):
        game_state: GameState = self.game_instance.game_state
        observation = {
            "score": game_state.score.get(),
            "auto_time": game_state.auto_time.get(),
            "teleop_time": game_state.teleop_time.get(),
            "endgame_time": game_state.endgame_time.get(),
            "current_time": game_state.total_time,
        }

        return observation

    def __get_robot_observation(self):
        robot_state: RobotState = self.game_instance.game_state.get("robots").get(
            "DunceBot"
        )

        alliance = 0
        if robot_state.alliance.get() == Alliance.BLUE:
            alliance = 1
        observation = {
            "x": robot_state.x.get().to(Meter),
            "y": robot_state.y.get().to(Meter),
            "heading": robot_state.heading.get().to(Degree),
            "alliance": alliance,
            "gamepieces": {
                "Coral": robot_state.gamepieces.get().get(Coral, 0),
                "Algae": robot_state.gamepieces.get().get(Algae, 0),
            },
        }

        return observation

    def step(self, action):

        # Get the action from the action space
        action = self.game_instance.get_actions_set("DunceBot")[action]
        self.last_action = action

        prev_score = self.game_instance.game_state.score.get()
        self.prev_gamepieces = (
            self.game_instance.game_state.get("robots")
            .get("DunceBot")
            .gamepieces.get()
            .copy()
        )

        # Perform the action - now returns (result, trajectory)
        result, trajectory = self.game_instance.drive_and_process_action(
            action[0], action[1], "DunceBot", self.game_instance.game_state.total_time
        )
        
        # Use trajectory returned directly from drive operation
        self.current_trajectory = trajectory

        # Get the game state
        game_state = self.game_instance.game_state

        # Get the reef state
        reef_state = self.game_instance.game_state.get("interactables").get(Names.Reef)

        # Get the robot state
        robot_state = self.game_instance.game_state.get("robots").get("DunceBot")

        # Get the reward
        reward = game_state.score.get() - prev_score
        if result:
            self.bad_count = 0
        if not result:
            self.bad_count += 1
            reward = -0.5

        # Check if the game is done
        done = (
            self.bad_count > 20
            or self.game_instance.game_state.current_time.get()
            > self.game_instance.game_state.auto_time.get()
        )  # self.game_instance.update(0) or self.bad_count > 20

        # Get the info
        info = {}

        return (
            {
                "reef_state": self.__get_reef_observation(),
                "game_state": self.__get_game_observation(),
                "robot_state": self.__get_robot_observation(),
            },
            reward,
            done,
            False,
            info,
        )

    def render(self, mode="human"):
        if mode == "human":
            import time

            current_game_time = self.game_instance.game_state.current_time.get()
            print(
                f"Time: {current_game_time}, Score: {self.game_instance.game_state.score.get()}"
            )

            traversal_space = self.game_instance.get_traversal_space("DunceBot")
            state = self.game_instance.game_state.get("robots").get("DunceBot")
            
            # Clear and setup renderer with static objects
            self.renderer.clear_objects()
            self.renderer.clear_dynamic()
            
            # Add obstacles to proper layers
            self.renderer.add_obstacles(Reefscape.get_obstacles())
            self.renderer.add_safety_padding(traversal_space.obstacles)
            self.renderer.add(traversal_space.traversal_map)
            
            # Use trajectory captured during step() (not get_latest_trajectory which might be stale)
            latest_traj = self.current_trajectory
            
            if latest_traj:
                self.renderer.add(latest_traj)
                traj_duration = latest_traj.get_travel_time().to(Second)
            else:
                traj_duration = 0
            
            # Calculate action pause time as: total_time_delta - trajectory_time
            # This gives us the actual time the robot spent on the action
            time_delta = current_game_time - self.prev_time
            action_pause = max(0, time_delta - traj_duration)
            
            # === ROBOT SELECTABILITY SETUP ===
            class RobotProxy:
                def __init__(self):
                    self.current_state = None
                    self.name = "DunceBot"
                def __repr__(self):
                    return f"<Robot: {self.name}>"
                
                def get_info_card_items(self):
                    """Return dynamic info for the info card."""
                    if self.current_state:
                         from gamegine.utils.NCIM.Dimensions.spatial import Meter
                         from gamegine.utils.NCIM.Dimensions.angular import Degree
                         
                         x_val = self.current_state.x.to(Meter) if hasattr(self.current_state.x, 'to') else self.current_state.x
                         y_val = self.current_state.y.to(Meter) if hasattr(self.current_state.y, 'to') else self.current_state.y
                         h_val = self.current_state.theta.to(Degree) if hasattr(self.current_state, 'theta') and hasattr(self.current_state.theta, 'to') else 0
                         
                         return {
                             "Name": self.name,
                             "Position": f"({x_val:.2f}m, {y_val:.2f}m)",
                             "Heading": f"{h_val:.1f}°",
                             "Velocity": f"{self.current_state.vx.to(Meter) if hasattr(self.current_state, 'vx') else 0:.1f} m/s", 
                         }
                    return {"Name": self.name, "Status": "No State"}

            robot_proxy = RobotProxy()
            robot_proxy.current_state = state
            
            def robot_hit_test(wx, wy):
                if robot_proxy.current_state is None:
                    return False
                from gamegine.utils.NCIM.Dimensions.spatial import Meter
                rx = robot_proxy.current_state.x.to(Meter) if hasattr(robot_proxy.current_state.x, 'to') else float(robot_proxy.current_state.x)
                ry = robot_proxy.current_state.y.to(Meter) if hasattr(robot_proxy.current_state.y, 'to') else float(robot_proxy.current_state.y)
                return ((wx - rx)**2 + (wy - ry)**2) < 0.16
            
            self.renderer.register_selectable(robot_proxy, robot_hit_test)
            
            # === ANIMATED RENDER LOOP ===
            # Part 1: Animate robot driving along trajectory
            # Part 2: Pause for action completion time
            
            frame_time = 1 / 60  # 60 FPS
            anim_time = 0.0
            total_anim_time = traj_duration + action_pause
            
            while anim_time < total_anim_time:
                # Determine current state
                if anim_time < traj_duration and latest_traj:
                    # Driving phase
                    current_robot_state = latest_traj.get_at_time(Second(anim_time))
                    current_action = f"Driving: {self.last_action[0]} -> {self.last_action[1]}"
                elif latest_traj:
                    # Action/pause phase (robot at end position)
                    current_robot_state = latest_traj.get_at_time(latest_traj.get_travel_time())
                    pause_remaining = total_anim_time - anim_time
                    current_action = f"Action: {self.last_action[1]} ({pause_remaining:.1f}s remaining)"
                else:
                    current_robot_state = state
                    current_action = "No trajectory"
                
                robot_proxy.current_state = current_robot_state
                
                # Render frame
                self.renderer.begin_frame()
                
                # Draw static layers
                for layer_name in ["safety_padding", "obstacles", "mesh"]:
                    if layer_name in self.renderer._layers:
                        for obj in self.renderer._layers[layer_name]:
                            if layer_name == "safety_padding":
                                self.renderer._draw_safety_padding_object(obj)
                            else:
                                self.renderer.draw_element(obj)
                
                # Draw trajectories
                if "trajectory" in self.renderer._layers:
                    for traj in self.renderer._layers["trajectory"]:
                        self.renderer.draw_element(traj)
                
                # Draw robot
                if current_robot_state:
                    self.renderer.draw_element(current_robot_state)

                # Draw HUD
                self.renderer.draw_rect(0, self.renderer.height - 95, 600, 95, (255, 255, 255))
                self.renderer.draw_text(
                    f"Score: {self.game_instance.game_state.score.get()}",
                    10, self.renderer.height - 25, (0, 0, 0), 18
                )
                self.renderer.draw_text(
                    f"Time: {anim_time:.1f}s / {total_anim_time:.1f}s (Drive: {traj_duration:.1f}s + Action: {action_pause:.1f}s)",
                    10, self.renderer.height - 45, (0, 0, 0), 14
                )
                self.renderer.draw_text(
                    f"Status: {current_action}",
                    10, self.renderer.height - 65, (0, 0, 0), 16
                )
                self.renderer.draw_text(
                    f"Selectables: {len(self.renderer._selectables)}",
                    10, self.renderer.height - 85, (128, 128, 128), 14
                )

                self.renderer.end_frame()
                
                # Process events
                if not self.renderer.loop():
                    break
                
                time.sleep(frame_time)
                anim_time += frame_time
                
            self.prev_time = current_game_time


    def close(self):
        pass

    def seed(self, seed=None):
        pass

    def configure(self, *args, **kwargs):
        pass

    def __del__(self):
        self.close()


env = FlattenObservation(Monitor(ReefBotEnvironment()))

# Register the custom environment (optional)

# Wrap the environment for vectorized training
vec_env = make_vec_env(lambda: env, n_envs=1)

# Define the PPO model
model = DQN(
    "MlpPolicy",
    vec_env,
    verbose=1,
    learning_rate=0.0001,
    tensorboard_log="./dqn_reefscape_tensorboard/",
)

# Define an evaluation callback
eval_callback = EvalCallback(
    vec_env,
    best_model_save_path="./logs/",
    log_path="./logs/",
    eval_freq=5000,
    deterministic=True,
    render=False,
)

# # Train the model
# model.learn(total_timesteps=10000, callback=eval_callback, progress_bar=True)

# # # Save the model
# model.save("duncebot_dqn")


# Load the model from the best modle saved, which is in the logs folder in current directory, make sure it gets relative to script
file_path = os.path.join(os.path.dirname(__file__), "logs", "best_model.zip")
model = DQN.load(file_path)

# model = DQN.load("duncebot_ppo")

# Evaluate the trained model
mean_reward, std_reward = evaluate_policy(model, env, n_eval_episodes=30)
print(f"Mean reward: {mean_reward} +/- {std_reward}")


num_actions = env.action_space.n

# Create labels for each action (or adapt to your environment)
action_labels = [f"Action {i}" for i in range(num_actions)]

# Initialize bar heights to zero
q_values_init = np.zeros(num_actions)

# Bar graph of q-values setup
bars = ax.bar(action_labels, q_values_init)
ax.set_title("Live Estimated 'Worth' of Each Action")
ax.set_ylabel("Q-Values")


def update_q_bar(new_q_values):
    """
    Update the bar chart:
      - set bar heights
      - highlight the highest q-value in red
      - set y-axis so the lowest value is 2 less than the smallest Q-value
    """
    # Find index of the highest q-value
    highest_index = np.argmax(new_q_values)

    # Update bars
    for i, (bar, val) in enumerate(zip(bars, new_q_values)):
        bar.set_height(val)
        # Reset colors to default for all bars
        bar.set_color("C0")

    # Highlight the highest Q-value bar
    bars[highest_index].set_color("red")

    # Manually adjust y-axis:
    # lowest axis value = (lowest Q-value - 2)
    # highest axis value = a bit above the highest Q-value
    y_min = min(new_q_values) - 0.5
    y_max = max(new_q_values) + 0.5
    ax.set_ylim([y_min, y_max])

    # Redraw
    plt.draw()
    plt.pause(0.01)


# Test the trained model
obs, info = env.reset()
done = False
while not done:

    obs_tensor = torch.tensor(obs, dtype=torch.float32).unsqueeze(0)
    q_values_tensor = model.policy.q_net(obs_tensor)
    q_values_array = q_values_tensor.detach().cpu().numpy().flatten()

    update_q_bar(q_values_array)

    action, _ = model.predict(obs, deterministic=True)
    obs, reward, done, truncate, info = env.step(action)

    env.render()

env.close()
