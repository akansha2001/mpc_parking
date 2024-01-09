import gymnasium as gym
from urdfenvs.robots.prius import Prius
import importlib
import numpy as np
from global_planner import GlobalPlanner
from local_planner import LocalPlanner
from trajectory import State
from trajectory import Trajectory
import time
from obstacles import generate_scene
from helper import FileOp
from helper import bcolors
import datetime
from helper import func_arc
'''
The Robot class describes a robot :). It contains the robot model (from class Prius), the local planner and global planner objects as class member variables.
'''

class Robot:
    def __init__(self, spawn_pos: np.ndarray = np.zeros(3), model: str = "prius", obstacles=None):
        # initializing robot attributes
        self.name = "robot_"
        # checking if the specified model is "prius"
        if model.lower() != "prius":
            raise Exception("Invalid model. Only 'prius' model is supported.")

        # creating a Prius model instance with "vel" mode
        self.model = Prius(mode="vel")
        self.wheel_base = self.model._wheel_distance
        
        self.state = State(L = self.wheel_base)
        # setting the spawn position and the current position of the robot
        self.spawn_pos = spawn_pos
        self.state.position = spawn_pos

        self.init_planner = False   # flag for initializing the planner
        # initializing a global planner of a certain 'type'

    def set_plan(self, global_plan):
        # TODO: move local planner to the parking lot env
        self.local_planner = LocalPlanner(Trajectory(global_plan))  # creating a LocalPlanner instance
        self.init_planner = True

    def get_target(self):
        if not self.init_planner:
            raise Exception("robot plan not set!")
        # currently getting the target from the local planner based on the current position
        return self.local_planner.plan(self.state)


class ParkingLotEnv:
    """
    Environment class for simulating a parking lot scenario with multiple robots.
    """
    GOAL = np.array([-2.5515, -8.9231, np.pi/2])
    #GOAL = np.array([-2.5515, -8.9231, np.pi/2])
    
    def __init__(self, render=True, stat_obs_flag = True, dyn_obs_flag = True):
        """
        - the constructor initializes the robots and sets the local and global planner 
        """
        self.render = render   # flag to set rendering (make this false to disable the new window)
        self.stat_obs_flag = stat_obs_flag
        self.dyn_obs_flag = dyn_obs_flag

        #Creating an object of file op to store the robot positions
        self.file=FileOp("data/robot_pos.csv")

        # creating a list of robots (only 1 is needed for now)
        self.robots = [Robot(spawn_pos=np.array([-2.5515, -8.9231, np.pi/2]))]
        self.robot_models = []
        self.rob_spawn_pos = np.array([])
        # assigning obstacles to class members
        static_obstacles, dynamic_obstacles, wall_obstacles = generate_scene()
        if self.stat_obs_flag:
            self.static_obstacles = static_obstacles
            self.wall_obstacles = wall_obstacles
            #self.static_obstacles.extend(self.wall_obstacles)

        if self.dyn_obs_flag:
            self.dynamic_obstacles = dynamic_obstacles

        # iterating over all robots
        for i in range(len(self.robots)):
            # appending the index to match the naming convention as defined in the obs object
            self.robots[i].name += str(i)
            # appending the model list, which is later used to create the env
            self.robot_models.append(self.robots[i].model)
            # spawn positions extracted, again used while creating the env
            self.rob_spawn_pos = np.append(self.rob_spawn_pos, self.robots[i].spawn_pos)
            self.set_global_plan(i)

        self.n_robots = len(self.robots)

    def set_global_plan(self, idx):
        # setting a global plan for each robot
            planner = GlobalPlanner(planner_type="rrt", static_obstacles=self.static_obstacles, wall_obstacles=self.wall_obstacles)
            start = datetime.datetime.now()
            global_plan = planner.plan(self.robots[idx].state.position, ParkingLotEnv.GOAL)
            #! uncomment to not use rrt
            # global_plan = [ParkingLotEnv.GOAL]
            end = datetime.datetime.now()
            delta = end - start
            print(bcolors.OKGREEN + "\n\nexecution time:", delta.total_seconds(), "s" + bcolors.ENDC)
            self.robots[idx].set_plan(global_plan)
    
    def setup_env(self):
        """
        - sets up the environment for simulation
        - creates a gym environment with specified parameters, initializes
        the action and state sizes, and resets the environment with the specified initial positions.
        """
        self.env = gym.make(
            "urdf-env-v0",
            dt=0.01, robots=self.robot_models, render=self.render
        )

        # add obstacles
        if self.stat_obs_flag:
            for obs in self.static_obstacles:
                self.env.add_obstacle(obs)
            for obs in self.wall_obstacles:
                self.env.add_obstacle(obs)
        if self.dyn_obs_flag:
            for obs in self.dynamic_obstacles:
                self.env.add_obstacle(obs)


        # the size of "action" is the size of the command that a robot takes (if there is one robot)
        # format: [forward velocity, yaw rate]
        # if there are multiple robots, actions are concatenated into a single 1D array
        # returns the number of actions possible in the whole environment
        self.action_size = self.env.n()

        # the size of the state variable (3 => x, y, theta(orientation)])
        self.ns_per_robot = self.env.ns_per_robot()
        # the size of the action variable (2 => forward_vel, yaw_rate)
        self.n_per_robot = self.env.n_per_robot()

        # resetting the environment with the specified initial positions
        print(self.rob_spawn_pos)
        self.ob = self.env.reset(pos=self.rob_spawn_pos)
        # printing the initial observation
        print(f"Initial observation : {self.ob}")

    # updating the state of the robot entities
    def update_robot_state(self):
        """
        - updates the state of the robots based on received joint state data from the 'obs' dictionary.
        - it iterates through each robot, and updates its attributes based on the joint state data.
        """
        for robot in self.robots:
            if robot.name in self.ob:
                robot_data = self.ob[robot.name]
                joint_state_data = robot_data.get('joint_state', {})
                # dynamically updating robot attributes based on received joint state data
                for key, value in joint_state_data.items():
                    setattr(robot.state, key, np.array(value))
            # print(robot.state.forward_velocity)
                self.file.write(robot.state.position)
            #print(robot.state.position)
        

    def get_action(self):
        """
        - get the concatenated action for all robots.

        Returns:
        - action (numpy.ndarray): Concatenated action array for all robots.
        """
        actions = []
        for robot in self.robots:
            actions.append(robot.get_target())
        action = np.concatenate(actions, axis=0)
        return action

    def run_env(self):
        """
        - sets up the environment, and runs the simulation loop (update states, get actions, send commands, repeat...)
        Returns:
        - history (list): List of observations from the simulation.
        """
        self.setup_env()
        # initializing an array 'action' with n elements
        action = np.ones(self.action_size)

        history = []
        try:
            while True:
                # updating all robot states (each robot in self.robots) by reading the 'obs' object
                self.update_robot_state()
                # getting actions from all robots based on the feedback
                action = self.get_action()
                self.ob, *_ = self.env.step(action)
                # * uncomment the line below to store observations in a list
                #history.append(action)

        except KeyboardInterrupt:
            pass  # this block will be executed on a keyboard interrupt
        print("\n\nEnding Simulation")
        self.env.close()
        return history