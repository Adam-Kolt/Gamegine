import logging
import time
import sys
import os
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '../../')))

from typing import List, Optional, Tuple, Deque
import collections
import matplotlib.pyplot as plt
import numpy as np
from gamegine.render.renderer import Renderer, run
from gamegine.representation.game import Game
from gamegine.representation.robot import SwerveRobot, PhysicalParameters
from gamegine.representation.interactable import RobotInteractionConfig
from gamegine.simulation.GameServer import GameServer
from gamegine.simulation.robot import RobotState
from dataclasses import dataclass
from gamegine.utils.NCIM.ncim import (
    Inch,
    Pound,
    Ampere,
    Feet,
    Degree,
    Second,
    NewtonMeter,
    RadiansPerSecond,
    MetersPerSecond,
    Torque
)
from gamegine.reference import gearing, motors
from gamegine.reference.swerve import SwerveConfig, SwerveModule
from gamegine.representation.bounds import Rectangle
from gamegine.first.alliance import Alliance

from Reefscape import Names, Reefscape, StartingPositionsBlue
from gamepieces import Coral
from scoring import ReefState

# Import specific game pieces if needed
from gamepieces import Coral

# -----------------------------------------------------------------------------
# Robot Setup (Duplicated from coral_point_sim.py but generalized)
# -----------------------------------------------------------------------------

ROBOT_WIDTH = Inch(32)
ROBOT_LENGTH = Inch(32)
ROBOT_MASS = Pound(120)
ROBOT_MOI = Pound(120) * Inch(30) ** 2

# Create specific SwerveConfig
def create_swerve_config():
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
            gearing.MK4I.L3,  # Steer gear ratio
        )
    )

ROBOT_GEOMETRY = [
    Rectangle.from_center((Inch(0), Inch(0)), ROBOT_WIDTH, ROBOT_LENGTH).get_3d(
        Inch(0), Feet(3)
    )
]

def create_robot(name: str):
    robot = SwerveRobot(
        name,
        create_swerve_config(),
        ROBOT_GEOMETRY,
        PhysicalParameters(ROBOT_MASS, ROBOT_MOI),
    )
    robot.override_bounding_radius(Inch(16))
    return robot

# -----------------------------------------------------------------------------
# Visualization Setup (Matplotlib)
# -----------------------------------------------------------------------------

# -----------------------------------------------------------------------------
# Visualization Setup (Matplotlib)
# -----------------------------------------------------------------------------

class SessionAnalyzer:
    def __init__(self):
        self.times = []
        self.wheel_vels = []  # rad/s
        self.torques = []     # Nm
        self.robot_vels = []  # m/s
        self.robot_omegas = [] # rad/s
        
    def record(self, t: float, robot_state: "SwerveTrajectoryState"):
        if not robot_state:
            return

        # Robot 1, Module 0 (FL)
        mod = robot_state.module_states[0]
        
        # Metrics
        w_omega = mod.wheel_omega.to(RadiansPerSecond) if hasattr(mod.wheel_omega, 'to') else mod.wheel_omega
        torque = mod.motor_torque.to(NewtonMeter) if hasattr(mod.motor_torque, 'to') and mod.motor_torque else 0.0
        
        vx = robot_state.vel_x.to(MetersPerSecond) if hasattr(robot_state.vel_x, 'to') else robot_state.vel_x
        vy = robot_state.vel_y.to(MetersPerSecond) if hasattr(robot_state.vel_y, 'to') else robot_state.vel_y
        r_vel = (vx**2 + vy**2)**0.5
        
        r_omega = robot_state.omega.to(RadiansPerSecond) if hasattr(robot_state.omega, 'to') else robot_state.omega

        self.times.append(t)
        self.wheel_vels.append(w_omega)
        self.torques.append(torque)
        self.robot_vels.append(r_vel)
        self.robot_omegas.append(r_omega)

    def plot(self):
        fig, axes = plt.subplots(4, 1, figsize=(10, 12), sharex=True)
        
        # 1. Torque
        axes[0].plot(self.times, self.torques, 'r-', label='Torque')
        axes[0].set_ylabel('Torque (Nm)')
        axes[0].set_title('Module 0 (FL) Analysis: Torque')
        axes[0].grid(True)
        
        # 2. Wheel Velocity
        axes[1].plot(self.times, self.wheel_vels, 'b-', label='Wheel Vel')
        axes[1].set_ylabel('Wheel Speed (rad/s)')
        axes[1].set_title('Wheel Velocity')
        axes[1].grid(True)
        
        # 3. Robot Velocity
        axes[2].plot(self.times, self.robot_vels, 'g-', label='Robot Speed')
        axes[2].set_ylabel('Speed (m/s)')
        axes[2].set_title('Robot Linear Velocity')
        axes[2].grid(True)
        
        # 4. Robot Omega
        axes[3].plot(self.times, self.robot_omegas, 'm-', label='Robot Omega')
        axes[3].set_ylabel('Omega (rad/s)')
        axes[3].set_xlabel('Time (s)')
        axes[3].set_title('Robot Angular Velocity')
        axes[3].grid(True)
        
        plt.tight_layout()
        plt.show()

# -----------------------------------------------------------------------------
# Simulation Manager
# -----------------------------------------------------------------------------

@dataclass
class VisualEvent:
    start_time: float
    end_time: float
    trajectory: Optional[any]  # None if just waiting/doing action
    robot_name: str

class DemoManager:
    def __init__(self):
        self.game_server = GameServer()
        self.game_server.load_from_game(Reefscape)
        
        # Create Robots
        self.robots = ["Robot1", "Robot2"]
        r1 = create_robot(self.robots[0])
        r2 = create_robot(self.robots[1])
        
        self.game_server.add_robot(r1)
        self.game_server.add_robot(r2)
        
        # Initialize Interactions
        self._init_interactions(r1)
        self._init_interactions(r2)
        
        # Initial State
        # Robot 1: Left side (Blue A)
        start_1 = StartingPositionsBlue.A
        self.game_server.init_robot(
            self.robots[0], 
            RobotState(*start_1, Degree(0), gamepieces={Coral: 1})
        )
        
        # Robot 2: Right side (Blue C)
        start_2 = StartingPositionsBlue.C
        self.game_server.init_robot(
            self.robots[1],
            RobotState(*start_2, Degree(0), gamepieces={Coral: 1})
        )
        
        self.visual_queue: Deque[VisualEvent] = collections.deque()
        self.playback_time = 0.0
        self.last_robot_finish_time = {r: 0.0 for r in self.robots}
        
        # Track last state for visualization
        from gamegine.analysis.trajectory.lib.trajectoryStates import SwerveTrajectoryState
        self.last_robot_states = {}
        for r_name in self.robots:
            robot_state = self.game_server.game_state.get("robots").get(r_name)
            # Create initial idle state
            self.last_robot_states[r_name] = SwerveTrajectoryState(
                x=robot_state.x.get(),
                y=robot_state.y.get(),
                theta=robot_state.heading.get()
            )
            
        self.analyzer = SessionAnalyzer()

    def _init_interactions(self, robot):
        # Add basic interactions similar to coral_point_sim.py
        # Using shorter times for quick demo
        interact_time = Second(0.5)
        
        configs = [
            (Names.TopCoralStation, "PickupCoral"),
            (Names.BottomCoralStation, "PickupCoral"),
            (Names.Reef, "l4"), # Simplify to just L4 for demo
        ]
        
        for interactable, name in configs:
            robot.add_interaction_config(
                RobotInteractionConfig(
                    interactable, name,
                    lambda s, r, g: True,
                    lambda s, r, g: interact_time
                )
            )

    def step_logic(self):
        # Simple Logic: 
        # Robot 1: Top Station -> Reef Left Columns (A-F)
        # Robot 2: Bottom Station -> Reef Right Columns (G-L)
        
        for robot_name in self.robots:
            sim_finish = self.last_robot_finish_time[robot_name]
            
            # Simple strategy state machine based on game pieces
            robot_state = self.game_server.game_state.get("robots").get(robot_name)
            has_coral = robot_state.gamepieces.get().get(Coral, 0) > 0
            
            traj = None
            
            # Determine action
            if has_coral:
                # Go Score
                target_col = "A" if robot_name == "Robot1" else "G"
                try:
                    success, traj = self.game_server.drive_and_process_action(
                        Names.Reef, f"l4_{target_col}", robot_name,
                        no_safety_cooridor=False 
                    )
                except Exception as e:
                    print(f"{robot_name} failed to drive: {e}")
            else:
                # Go Pickup
                station = Names.TopCoralStation if robot_name == "Robot1" else Names.BottomCoralStation
                try:
                    success, traj = self.game_server.drive_and_process_action(
                        station, "PickupCoral", robot_name
                    )
                except Exception as e:
                    print(f"{robot_name} failed pickup: {e}")

            if traj:
                duration = traj.get_travel_time().to(Second)
                start_t = sim_finish
                end_t = start_t + duration
                
                self.visual_queue.append(VisualEvent(start_t, end_t, traj, robot_name))
                self.last_robot_finish_time[robot_name] = end_t
                
                action_duration = 0.5 
                self.visual_queue.append(VisualEvent(end_t, end_t + action_duration, None, robot_name))
                self.last_robot_finish_time[robot_name] += action_duration

    def get_visual_state(self, robot_name: str):
        return self.last_robot_states.get(robot_name)

    def update(self, dt):
        self.playback_time += dt
        
        # Generate new commands if needed
        if min(self.last_robot_finish_time.values()) < self.playback_time + 2.0:
            self.step_logic()
            
        # Process visual queue
        for name in self.robots:
            active_event = None
            for event in self.visual_queue:
                if event.robot_name == name and event.start_time <= self.playback_time <= event.end_time:
                    active_event = event
                    break
            
            state = None
            if active_event and active_event.trajectory:
                rel_t = self.playback_time - active_event.start_time
                state = active_event.trajectory.get_at_time(Second(rel_t))
            elif name in self.last_robot_states:
                # Keep last state if idle
                # Ideally we want the exact end state of the last trajectory
                # current logic holds `last_robot_states` as the current sample
                # But if we are in an action gap, we should hold position.
                state = self.last_robot_states[name]

            if state:
                self.last_robot_states[name] = state
                if name == "Robot1":
                    self.analyzer.record(self.playback_time, state)
        
        # Cleanup
        while self.visual_queue and self.visual_queue[0].end_time < self.playback_time - 1.0:
            self.visual_queue.popleft()


# -----------------------------------------------------------------------------
# Main Loop 
# -----------------------------------------------------------------------------

def main():
    print("Initializing simulation...")
    manager = DemoManager()
    
    # Headless simulation loop
    dt = 0.02
    total_time = 30.0
    steps = int(total_time / dt)
    
    print(f"Running simulation for {total_time} seconds...")
    
    for i in range(steps):
        manager.update(dt)
        if i % 100 == 0:
            print(f" Simulating... T={manager.playback_time:.2f}s")
            
    print("Simulation complete. Generating plots...")
    manager.analyzer.plot()

if __name__ == "__main__":
    main()
