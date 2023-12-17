try:
     from ompl import base as ob
     from ompl import control as oc
except ImportError:
    #if the ompl module is not in the PYTHONPATH assume it is installed in a
    #subdirectory of the parent directory called "py-bindings."
    from os.path import abspath, dirname, join
    import sys
    sys.path.insert(0, join(dirname(dirname(abspath(__file__))), 'py-bindings'))
    from ompl import base as ob
    from ompl import control as oc
import numpy as np
from functools import partial
from math import sin, cos
from obstacles import static_obstacles
class RRT():

    def __init__(self,obstacles=None):
        
        # Public attributes
        self.space = ob.SE2StateSpace()
        # set the bounds for the R^2 part of SE(2)
        bounds = ob.RealVectorBounds(2)
        bounds.setLow(-10)
        bounds.setHigh(10)
        self.space.setBounds(bounds)

        
        self.obstacles=obstacles
        
        
        
        # create a control space
        self.cspace = oc.RealVectorControlSpace(self.space, 2)
    
        # set the bounds for the control space
        cbounds = ob.RealVectorBounds(2)
        cbounds.setLow(-.3)
        cbounds.setHigh(.3)
        self.cspace.setBounds(cbounds)

        # define a simple setup class
        self.ss = oc.SimpleSetup(self.cspace)
        isValidFn = ob.StateValidityCheckerFn(partial(self.collision_checker(), self.ss.getSpaceInformation()))
        self.ss.setStateValidityChecker(isValidFn)
        self.ss.setStatePropagator(oc.StatePropagatorFn(self.propagate))
        self.si = self.ss.getSpaceInformation()
        self.si.setPropagationStepSize(.6)
        self.planner=oc.RRT(self.si)

    def plan(self, start, goal):
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
            num_columns = 6  # 6 columns in the data
            self.data_array = self.path.reshape(-1, num_columns)
            self.states_final=self.data_array[:,0:3]
            self.controls=self.data_array[:,3:5]
            print(self.states_final)
            #print(self.controls)
            return self.states_final
        else:
            print("No solution found")

    def collision_checker(self):
        def isStateValid(spaceInformation, state):
        # perform collision checking or check if other constraints are
        # satisfied
            robot_position = np.array([state.getX(), state.getY()])
            if self.obstacles is not None:
                for x in self.obstacles:
                    obstacle_position = np.array([x['position'][0], x['position'][1]])
                    obstacle_size = np.array([x['width'], x['height']])
                    radius = 1.5*np.max(obstacle_size) #should add a factor of safety
                    d = np.linalg.norm(robot_position - obstacle_position) #euclidean distance between obstacle and state
                    if d < radius: #collision 
                        print("Avoided collision with obstacles")
                        return False
                    
                
            return spaceInformation.satisfiesBounds(state)
        return isStateValid
    
    def propagate(self,start, control, duration, state):
     state.setX(start.getX() + control[0] * duration * cos(start.getYaw()))
     state.setY(start.getY() + control[0] * duration * sin(start.getYaw()))
     state.setYaw(start.getYaw() + control[1] * duration)