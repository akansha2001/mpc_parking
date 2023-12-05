import gymnasium as gym
from urdfenvs.robots.prius import Prius
import importlib
import numpy as np

def get_gobal_path():
    pass

def run_prius(n_steps=1000, render=False, goal=True, obstacles=True):
    robots = [
        Prius(mode="vel", spawn_offset=np.array([0.0, 0.0, 0.0])),
        Prius(mode="vel", spawn_offset=np.array([2.0, 2.0, 0.0])),
    ]
    env = gym.make(
        "urdf-env-v0",
        dt=0.01, robots=robots, render=render
    )
    # the size of "action" is the size of the command that a robot takes (if there is one robot)
    # for it would be of the format [forward velocity, yaw rate]
    n = env.n() # returns the number of actions possible in the whole environment

    # initialize an array 'action' with n elements, each set to -0.2
    action = np.ones(n) * -0.2

    # get the number of sensors per robot and the total number of sensors
    ns_per_robot = env.ns_per_robot()
    print("ns_per_robot:", ns_per_robot)
    n_per_robot = env.n_per_robot()
    print("n_per_robot:", n_per_robot)

    # create an array 'initial_positions' to store initial positions of each sensor
    initial_positions = np.array([np.zeros(n) for n in ns_per_robot])

    # iterating over the initial positions
    for i in range(len(initial_positions)):
        # checking whether the no. of sensors for a robot is not equal to the total number of sensors
        if ns_per_robot[i] != n_per_robot[i]:
            # setting the first two elements of the initial position to [0.0, i]
            initial_positions[i][0:2] = np.array([0.0, i])

    # creating an array 'mount_positions' with initial mount positions for each robot
    mount_positions = np.array([np.array([0.0, i, 0.0]) for i in range(len(ns_per_robot))])

    # resetting the environment with the specified initial positions and mount positions
    ob = env.reset(pos=initial_positions, mount_positions=mount_positions)

    # printing the initial observation
    print(f"Initial observation : {ob}")
    history = []
    for i in range(n_steps):
        ob, *_ = env.step(action)
        print(ob)
        if ob['robot_0']['joint_state']['steering'] > 0.2:
            action[1] = 0
        history.append(ob)
    env.close()
    return history

if __name__ == "__main__":
    run_prius(render=True)
