import gymnasium as gym
from urdfenvs.robots.prius import Prius
import numpy as np
#from obstacles_bakul import static_obstacles, dynamic_obstacles, wall_obstacles, parking_spaces, parking_limits, road_limits
from obstacles import wall_obstacles, static_obstacles
import time

class Robot:
    def __init__(self, spawn_pos: np.ndarray = np.array([5,5,0]), model: str = "prius"):
        self.name = "robot_"
        if model.lower() != "prius":
            raise Exception("Invalid model. Only 'prius' model is supported.")
        self.model = Prius(mode="vel")
        self.wheel_base = self.model._wheel_distance
        self.state = {"position": spawn_pos}

    def set_obstacles(self, obstacles):
        self.obstacles = obstacles

    def render(self):
        env = gym.make("urdf-env-v0", dt=0.01, robots=[self.model], render=True)

        for obstacle in self.obstacles:
            env.add_obstacle(obstacle)

        env.reset(pos=self.state["position"])

        # Run the simulation for a certain number of steps or indefinitely
        for _ in range(1000):  # Adjust the number of steps as needed
            env.render()
            time.sleep(0.01)  # Add a small delay between frames

        env.close()

if __name__ == "__main__":
    robot = Robot()
    # robot.set_obstacles(parking_spaces+parking_limits+wall_obstacles+road_limits)
    robot.set_obstacles(wall_obstacles + static_obstacles)
    robot.render()
