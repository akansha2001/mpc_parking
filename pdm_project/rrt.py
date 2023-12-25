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
from obstacles import static_obstacles
from mpscenes.obstacles.box_obstacle import BoxObstacle
from mpscenes.obstacles.dynamic_sphere_obstacle import DynamicSphereObstacle
from shapely import Polygon
class RRT():

    def __init__(self,staticObstacles=None, wallObstacles=None):
        
        # Public attributes
        self.space = ob.DubinsStateSpace()
        # set the bounds for the R^2 part of SE(2)
        bounds = ob.RealVectorBounds(2)
        bounds.setLow(-10)
        bounds.setHigh(10)
        self.space.setBounds(bounds)

        
        self.static_obstacles=staticObstacles
        self.wall_obstacles=wallObstacles
        # create a control space
        #self.cspace = oc.RealVectorControlSpace(self.space, 2)
    
        # set the bounds for the control space
        # cbounds = ob.RealVectorBounds(2)
        # cbounds.setLow(-.3)
        # cbounds.setHigh(.3)
        # self.cspace.setBounds(cbounds)

        # define a simple setup class
        self.ss = og.SimpleSetup(self.space)
        self.si = self.ss.getSpaceInformation()
        isValidFn = ob.StateValidityCheckerFn(self.collision_checker)
        self.ss.setStateValidityChecker(isValidFn)
        #self.ss.setStatePropagator(oc.StatePropagatorFn(self.propagate))
        #self.si.setPropagationStepSize(.6)
        self.planner=og.RRT(self.si)
        self.planner.setRange(0.2)
        self.planner.setGoalBias(0.1)

    def plan(self, start, goal, output_file="path_output.txt"):
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
        self.solved = self.ss.solve(10.0)
        if self.solved: 
            self.path=self.ss.getSolutionPath().printAsMatrix()
            self.path = np.fromstring(self.path.strip(), sep=' ')  #The output of printAsMatrix() is a string
            num_columns = 3  # 6 columns in the data
            self.data_array = self.path.reshape(-1, num_columns)
            self.states_final=self.data_array[:,0:3]
            #self.controls=self.data_array[:,3:5]
            #print(self.controls)
            
            with open(output_file, 'w') as file:
                np.savetxt(file, self.states_final, fmt='%.6f', delimiter=', ')
            print(f"Path saved to {output_file}")
            return self.states_final
        else:
            print("No solution found")

    def carPolygon(self, x, y, yaw): #returns a polygon for the car given a position and heading
        #geometry of the car
        carLength = 4.599 * 0.3
        carWidth = 1.782 * 0.3
        return Polygon(shell=((x + carLength/2*np.cos(yaw) + carWidth/2*np.sin(yaw), y + carLength/2*np.sin(yaw) - carWidth/2*np.cos(yaw)),
                            (x - carLength/2*np.cos(yaw) + carWidth/2*np.sin(yaw), y - carLength/2*np.sin(yaw) - carWidth/2*np.cos(yaw)),
                            (x - carLength/2*np.cos(yaw) - carWidth/2*np.sin(yaw), y - carLength/2*np.sin(yaw) + carWidth/2*np.cos(yaw)),
                            (x + carLength/2*np.cos(yaw) - carWidth/2*np.sin(yaw), y + carLength/2*np.sin(yaw) + carWidth/2*np.cos(yaw))))


    def obstaclePolygon(self, obstacle):
        position = np.array([obstacle.position()[0], obstacle.position()[1]])
        width = obstacle.width()
        length = obstacle.length()
        return Polygon(shell=((position[0]+width/2, position[1]+length/2),
                                        (position[0]+width/2, position[1]-length/2),
                                        (position[0]-width/2, position[1]-length/2),
                                        (position[0]-width/2, position[1]+length/2)))
    
    def wallPolygon(self,obstacle):
        position = np.array([obstacle.position()[1], obstacle.position()[0]])
        width = obstacle.width()
        length = obstacle.length()
        return Polygon(shell=((position[0]+width/2, position[1]+length/2),
                                        (position[0]+width/2, position[1]-length/2),
                                        (position[0]-width/2, position[1]-length/2),
                                        (position[0]-width/2, position[1]+length/2)))

    def collision_checker(self, state):
        return self.clearance(self.si, state)

    # def clearance(self, spaceInformation, state):
    #     car_x = state.getX()
    #     car_y = state.getY()
    #     car_yaw = state.getYaw()
    #     polygonCar = self.carPolygon_(car_x, car_y, car_yaw)
    #     validFlagStatic = True
    #     validFlagWall = True
    #     if self.static_obstacles is not None:
    #         for obstacle in self.static_obstacles:
    #             polygonObstacles_static = self.obstaclePolygon(obstacle)
    #             if polygonCar.intersects(polygonObstacles_static):
    #                 validFlagStatic = False
    #     if self.wall_obstacles is not None:
    #         for obstacle in self.wall_obstacles:
    #             polygonObstacles_wall = self.wallPolygon(obstacle)
    #             if polygonCar.intersects(polygonObstacles_wall):
    #                 validFlagWall = False
    # # Check if there is clearance from both types of obstacles
    #     if not validFlagStatic or not validFlagWall:
    #         return False
    #     else:
    #         return spaceInformation.satisfiesBounds(state)

    def clearance(self, spaceInformation, state):
        car_x = state.getX()
        car_y = state.getY()
        car_yaw = state.getYaw()
        polygonCar = self.carPolygon(car_x, car_y, car_yaw)
        if self.static_obstacles is not None:
            for obstacle in self.static_obstacles:
                polygonObstacles = self.obstaclePolygon(obstacle)
                if polygonCar.intersects(polygonObstacles):
                    return False
        return spaceInformation.satisfiesBounds(state)
