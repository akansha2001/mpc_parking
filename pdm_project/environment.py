import gymnasium as gym
from urdfenvs.robots.prius import Prius
import importlib
import numpy as np
from global_planner import GlobalPlanner
from local_planner import *
from trajectory import State
from trajectory import Trajectory
from trajectory import generate_spline
import time
from obstacles import generate_scene
from helper import FileOp
from helper import bcolors
from mpc import MPC
import datetime

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

        print(f"\nRobot {self.name} spawning at ({spawn_pos[0]}, {spawn_pos[1]})")
        # creating a Prius model instance with "vel" mode
        self.model = Prius(mode="vel")
        self.wheel_base = self.model._wheel_distance
        
        self.state = State(L = self.wheel_base)
        # setting the spawn position and the current position of the robot
        self.spawn_pos = spawn_pos
        self.state.position = spawn_pos
        self.obstacles = obstacles

        self.init_planner = False   # flag for initializing the planner
        # initializing a global planner of a certain 'type'

    def set_plan(self, global_plan, controller_type = ""):
        # TODO: move local planner to the parking lot env
        if controller_type.lower() == "mpc":
            self.local_planner = MPC(Trajectory(global_plan))  # creating a LocalPlanner instance
        elif controller_type.lower() == "pure_pursuit":
            self.local_planner = PurePursuit(Trajectory(global_plan))
        else:
            self.local_planner = DummyLocalPlanner()
        self.init_planner = True

    def get_target(self):
        if not self.init_planner:
            raise Exception("robot plan not set!")
        # currently getting the target from the local planner based on the current position
        return self.local_planner.plan(self)

    def extract_obstacles(self,distance_tolerance, angle_tolerance = 0.6):
        # TODO: tolerances tuning
        if self.obstacles is not None:
            position_self = self.state.position
            centers_obstacles = []
            for obstacle in self.obstacles:
                position_obstacle = obstacle.state.position
                
                angle = np.arctan2(position_obstacle[1] - position_self[1],position_obstacle[0] - position_self[0])
                distance = np.linalg.norm(position_obstacle[:2] - position_self[:2],ord=2)

                print(position_self[2] - angle_tolerance <= angle <= position_self[2] + angle_tolerance, distance <= distance_tolerance)
                if position_self[2] - angle_tolerance <= angle <= position_self[2] + angle_tolerance and distance <= distance_tolerance:
                    centers_obstacles.append(position_obstacle[:2])
            return centers_obstacles
class ParkingLotEnv:
    """
    Environment class for simulating a parking lot scenario with multiple robots.
    """
    #! move hardcoded variables
    #GOAL = np.array([-2.6,2.4, np.pi/2]) # goal of the robot
    GOAL = np.array([-2.3, -5.6, np.pi/2])
    START = np.array([-2.5515, -8.9231, np.pi/2])   # start of the robot
    CAR_SPAWN_LOCATIONS = np.array([[-0.9,1.2,0],[-5.0,0.0,np.pi],[-5.0,-2.4,np.pi],[-0.9,0,0]
    ,[-3.9,-3.6,np.pi/2]])  # car spawn locations
    DYNAMIC_CAR_INDEX = CAR_SPAWN_LOCATIONS.shape[0] - 1    # represents the dynamic cars
    N_CARS = CAR_SPAWN_LOCATIONS.shape[0]
    DYNAMIC_CAR_PATH_LOG_FILE = "data/dynamic_car_pos.csv"
    ROBOT_PATH_LOG_FILE="data/robot_pos.csv"
    def __init__(self, render=True, stat_obs_flag = True):
        """
        - the constructor initializes the robots and sets the local and global planner 
        """
        self.render = render   # flag to set rendering (make this false to disable the new window)
        self.stat_obs_flag = stat_obs_flag
        #Creating an object of file op to store the robot positions
        self.file_dynamic_car=FileOp(ParkingLotEnv.DYNAMIC_CAR_PATH_LOG_FILE)
        self.file_robot=FileOp(ParkingLotEnv.ROBOT_PATH_LOG_FILE)
        # creating a list of robots (only 1 is needed for now)
        rob_spawn_pos_list = []
        self.robots = [Robot(spawn_pos=ParkingLotEnv.START)]
        self.robot_models = []
        # assigning obstacles to class members
        static_obstacles, wall_obstacles = generate_scene()
        if self.stat_obs_flag:
            self.static_obstacles = static_obstacles
            self.wall_obstacles = wall_obstacles
        
        self.enemies = []
        for i in range(ParkingLotEnv.N_CARS):
            self.enemies.append(Robot(spawn_pos=ParkingLotEnv.CAR_SPAWN_LOCATIONS[i]))

        self.dynamic_obstacles = [self.enemies[ParkingLotEnv.DYNAMIC_CAR_INDEX]]
        
        # iterating over all robots
        for i in range(len(self.robots)):
            # appending the index to match the naming convention as defined in the obs object
            self.robots[i].name += str(i)
            self.robots[i].obstacles = self.dynamic_obstacles
            # appending the model list, which is later used to create the env
            self.robot_models.append(self.robots[i].model)
            # spawn positions extracted, again used while creating the env
            rob_spawn_pos_list.append(self.robots[i].spawn_pos)
            self.set_global_plan(i)
        
        for i in range(len(self.enemies)):
            # appending the index to match the naming convention as defined in the obs object
            self.enemies[i].name += "enemy"+str(i)
            # appending the model list, which is later used to create the env
            self.robot_models.append(self.enemies[i].model)
            # setting the global plan as the spawn position
            rob_spawn_pos_list.append(self.enemies[i].spawn_pos)
            goal_pos_list = [self.enemies[i].spawn_pos]
            if i == ParkingLotEnv.DYNAMIC_CAR_INDEX:
                points_path=generate_spline(self.enemies[i].spawn_pos, offset = 0.5, turning_radius = 2.4)
                final_path=goal_pos_list+points_path
                #Storing final_path into csv file
                
                with open("data/dynamic_car_planned_pos.txt", 'w') as file:
                    np.savetxt(file, np.array(final_path), fmt='%.6f', delimiter=', ')               
                self.enemies[i].set_plan(final_path, "pure_pursuit")
            else:
                self.enemies[i].set_plan(goal_pos_list, "dummy")
        
        self.rob_spawn_pos = np.vstack(rob_spawn_pos_list)
        self.n_robots = len(self.robots)

    def set_global_plan(self, idx):
        # setting a global plan for each robot
        planner = GlobalPlanner(planner_type="rrt", static_obstacles=self.static_obstacles, wall_obstacles=self.wall_obstacles)
        start = datetime.datetime.now()
        global_plan = planner.plan(self.robots[idx].state.position, ParkingLotEnv.GOAL)
        end = datetime.datetime.now()
        delta = end - start
        print(bcolors.OKGREEN + "\n\nexecution time:", delta.total_seconds(), "s" + bcolors.ENDC)
        self.robots[idx].set_plan(global_plan, "mpc")
    
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
        self.ob = self.env.reset(pos=self.rob_spawn_pos)
        # printing the initial observation

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
                self.file_robot.write(robot.state.position)
                # dynamically updating robot attributes based on received joint state data
                for key, value in joint_state_data.items():
                    setattr(robot.state, key, np.array(value))
                robot.obstacles = self.dynamic_obstacles
        
        for i, enemy in enumerate(self.enemies):
            enemy_name = "robot_"+ str(i + self.n_robots)
            if enemy_name in self.ob:
                enemy_data = self.ob[enemy_name]
                joint_state_data = enemy_data.get('joint_state', {})
                if i==ParkingLotEnv.DYNAMIC_CAR_INDEX:
                    self.file_dynamic_car.write(self.enemies[i].state.position)
                # dynamically updating enemy attributes based on received joint state data
                for key, value in joint_state_data.items():
                    setattr(enemy.state, key, np.array(value))

        self.dynamic_obstacles = [self.enemies[ParkingLotEnv.DYNAMIC_CAR_INDEX]]


    def get_action(self):
        """
        - get the concatenated action for all robots.

        Returns:
        - action (numpy.ndarray): Concatenated action array for all robots.
        """
        actions = []
        for robot in self.robots:
            target = robot.get_target()
            actions.append(target)
        for car in self.enemies:
            target = car.get_target()
            actions.append(target)
        action = np.array(actions)
        action = action.flatten()
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
