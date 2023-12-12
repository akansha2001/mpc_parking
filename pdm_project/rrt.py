try:
     from ompl import base as ob
     from ompl import control as oc
except ImportError:
     # if the ompl module is not in the PYTHONPATH assume it is installed in a
     # subdirectory of the parent directory called "py-bindings."
     from os.path import abspath, dirname, join
     import sys
     sys.path.insert(0, join(dirname(dirname(abspath(__file__))), 'py-bindings'))
     from ompl import base as ob
     from ompl import control as oc
import numpy as np
from functools import partial
from math import sin, cos
class RRT():

    def __init__(self) -> None:
        
        # Public attributes
        self.space = ob.SE2StateSpace()
        # set the bounds for the R^2 part of SE(2)
        bounds = ob.RealVectorBounds(2)
        bounds.setLow(-1)
        bounds.setHigh(1)
        self.space.setBounds(bounds)
    
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
        self.si.setPropagationStepSize(.1)
        self.planner=oc.RRT(self.si)

    def plan(self, start, goal):
        self.ss.setStartAndGoalStates(start, goal, 0.05)
        self.ss.setPlanner(self.planner)
        self.solved = self.ss.solve(20.0)
        if self.solved:
            self.path = self.ss.getSolutionPath()
            #print(self.ss.getProblemDefinition().getSolutionCount())
            self.ss.getPathSimplifier.simplifyMax(self.path)
            self.ss.getPathSimplifier.smoothBSpline(self.path)
            print(self.path.printAsMatrix())
            #self.points=self.convert_path_to_points(self.path)
        else:
            print("No solution found")

    def collision_checker(self):
        def isStateValid(spaceInformation, state):
        # perform collision checking or check if other constraints are
        # satisfied
            return spaceInformation.satisfiesBounds(state)
        return isStateValid
    
    def convert_path_to_points(self,path_pointer):
    # checking if the input has data
        if path_pointer is None:
            raise ValueError("Invalid path pointer")
        # getting space info
        # space_info = path_pointer.getSpaceInformation()
        
        # extracting states from the path
        path_states = path_pointer.getStates()
        points = []
        for state in path_states:
            x = state.getX()
            y = state.getY()
            yaw = state.getYaw()
            # appending the point (x, y, yaw) to the list
            points.append([x, y, yaw])

        return points

    def propagate(self,start, control, duration, state):
     state.setX(start.getX() + control[0] * duration * cos(start.getYaw()))
     state.setY(start.getY() + control[0] * duration * sin(start.getYaw()))
     state.setYaw(start.getYaw() + control[1] * duration)