import numpy as np
from dummy_planner import DummyPlanner
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
class GlobalPlanner:
    def __init__(self, planner_type="dummy"):

        # initializing the planner based on type
        self.planner = self._initialize_planner(planner_type)

    def _initialize_planner(self, planner_type):
        """
        initializes the global planner based on the given type
        """
        if planner_type == "rrt":
            self.space = ob.SE2StateSpace()
    
            # set the bounds for the R^2 part of SE(2)
            bounds = ob.RealVectorBounds(2)
            bounds.setLow(-1)
            bounds.setHigh(1)
            self.space.setBounds(bounds)
        
            # create a control space
            self.cspace = oc.RealVectorControlSpace(space, 2)
        
            # set the bounds for the control space
            cbounds = ob.RealVectorBounds(2)
            cbounds.setLow(-.3)
            cbounds.setHigh(.3)
            self.cspace.setBounds(cbounds)
        
            # define a simple setup class
            self.ss = oc.SimpleSetup(cspace)
            self.si = self.ss.getSpaceInformation()
            
            return oc.RRT(self.si)
            
        elif planner_type == "dummy":
            return DummyPlanner()
        
        else:
            raise ValueError(f"Invalid planner type: {planner_type}")

    def plan_global_path(self, start: np.ndarray, goal: np.ndarray):
        """
        Plan a global path from the given start to the goal.

        Args:
        - start: start configurtion.
        - goal: soal configuration.

        Returns:
        - path: list of waypoints (np arrays) representing the global path.
        """
        self.ss.setStartAndGoalStates(start, goal, 0.05)
        self.ss.setPlanner(self.planner)
        self.path = self.ss.getSolutionPath().printAsMatrix()
        return self.path
