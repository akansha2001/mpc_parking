import numpy as np
# from obstacles import static_obstacles, wall_obstacles
try:
     from ompl import base as ob
     from ompl import geometric as og
except ImportError:
    #if the ompl module is not in the PYTHONPATH assume it is installed in a
    #subdirectory of the parent directory called "py-bindings."
    from os.path import abspath, dirname, join
    import sys
    sys.path.insert(0, join(dirname(dirname(abspath(__file__))), 'py-bindings'))
    from ompl import base as ob
    from ompl import geometric as og
import numpy as np
from functools import partial
from math import sin, cos
from shapely import Polygon
from trajectory import generate_spline

class GlobalPlanner:
    def __init__(self, planner_type="dummy",static_obstacles=None, wall_obstacles=None):
        # initializing the planner based on type
        self.staticObstacles=static_obstacles
        self.wallObstacles = wall_obstacles
        self.planner = self._initialize_planner(planner_type,static_obstacles=self.staticObstacles, wall_obstacles = self.wallObstacles)
    def _initialize_planner(self, planner_type,static_obstacles=None, wall_obstacles=None):
        """
        initializes the global planner based on the given type
        """
        if planner_type == "rrt":
            return RRT(static_obstacles, wall_obstacles)  # replace with the actual RRT planner class
        else:
            raise ValueError(f"Invalid planner type: {planner_type}")
    def plan(self, start, goal):
        """
        Plan a global path from the given start to the goal.

        Args:
        - start: start configurtion.
        - goal: soal configuration.

        Returns:
        - path: list of waypoints (np arrays) representing the global path.
        """
        return self.planner.plan(start, goal)

    def plan(self, start, goal):
        """
        Plan a global path from the given start to the goal.

        Args:
        - start: start configurtion.
        - goal: soal configuration.

        Returns:
        - path: list of waypoints (np arrays) representing the global path.
        """
        return self.planner.plan(start, goal)


class RRT():

    def __init__(self,staticObstacles=None, wallObstacles=None):
        
        # Public attributes
        self.space = ob.DubinsStateSpace(2.0, False)
        
        #self.space = ob.SE2StateSpace()
        # set the bounds for the R^2 part of SE(2)
        bounds = ob.RealVectorBounds(2)
        bounds.setLow(-10)
        bounds.setHigh(10)
        self.space.setBounds(bounds)
        self.static_obstacles=staticObstacles
        self.wall_obstacles=wallObstacles
        self.output_file= "data/path_output.txt"

        # define a simple setup class
        self.ss = og.SimpleSetup(self.space)
        self.si = self.ss.getSpaceInformation()
        isValidFn = ob.StateValidityCheckerFn(self.collision_checker)
        self.ss.setStateValidityChecker(isValidFn)

        self.planner=og.RRT(self.si)
        self.planner.setRange(0.6)
        self.planner.setGoalBias(0.1)

    def plan(self, start, goal):
        # print(goal)
        x_start ,y_start ,yaw_start = start[0], start[1], start[2]
        x_goal = goal[0]
        y_goal = goal[1]
        yaw_goal = goal[2]

        start=ob.State(self.space)
        end=ob.State(self.space)
        start().setX(x_start)
        start().setY(y_start)
        start().setYaw(yaw_start)
        end().setX(x_goal)
        end().setY(y_goal)
        end().setYaw(yaw_goal)
        self.ss.setStartAndGoalStates(start,end, 0.05)
        self.ss.setPlanner(self.planner)
        self.solved = self.ss.solve(200.0)
        if self.solved: 
            self.path=self.ss.getSolutionPath().printAsMatrix()
            self.path = np.fromstring(self.path.strip(), sep=' ')  #The output of printAsMatrix() is a string
            num_columns = 3  # 6 columns in the data
            self.data_array = self.path.reshape(-1, num_columns)
            self.states_final=self.data_array[:,0:3]

            park_path=generate_spline(self.states_final[-1],1.2,2.0,"counter_clockwise")
            trajectory_points=np.concatenate((self.states_final,np.array(park_path)))
            with open(self.output_file, 'w') as file:
                np.savetxt(file, trajectory_points, fmt='%.6f', delimiter=', ')
            print(f"Path saved to {self.output_file}")
            return trajectory_points
        else:
            print("No solution found")

    def carPolygon(self,x,y,yaw): #returns a polygon for the car given a position and heading
               
        #geometry of the car
        carLength = 1
        carWidth = 0.45
        return Polygon(shell=((x + carLength/2*np.cos(yaw) + carWidth/2*np.sin(yaw), y + carLength/2*np.sin(yaw) - carWidth/2*np.cos(yaw)),
                            (x - carLength/2*np.cos(yaw) + carWidth/2*np.sin(yaw), y - carLength/2*np.sin(yaw) - carWidth/2*np.cos(yaw)),
                            (x - carLength/2*np.cos(yaw) - carWidth/2*np.sin(yaw), y - carLength/2*np.sin(yaw) + carWidth/2*np.cos(yaw)),
                            (x + carLength/2*np.cos(yaw) - carWidth/2*np.sin(yaw), y + carLength/2*np.sin(yaw) + carWidth/2*np.cos(yaw))))


    def wallPolygon(self, obstacle):
        position = np.array([obstacle.position()[0], obstacle.position()[1]])
        width = obstacle.length() #+ 0.2
        length = obstacle.width() #+ 0.2
        return Polygon(shell=((position[0]+width/2, position[1]+length/2),
                                        (position[0]+width/2, position[1]-length/2),
                                        (position[0]-width/2, position[1]-length/2),
                                        (position[0]-width/2, position[1]+length/2)))

    def collision_checker(self, state):
        return self.clearance(self.si, state)

    def clearance(self, spaceInformation, state):
        car_x = state.getX()
        car_y = state.getY()
        car_yaw = state.getYaw()
        #print(len(self.static_obstacles))
        polygonCar = self.carPolygon(car_x, car_y, car_yaw)
        if self.static_obstacles is not None:
            for obstacle in (self.static_obstacles + self.wall_obstacles):
                polygonObstacles = self.wallPolygon(obstacle)
                if polygonCar.intersects(polygonObstacles):
                    #print("Collision Avoided",car_x, car_y)
                    return False
        return spaceInformation.satisfiesBounds(state)
