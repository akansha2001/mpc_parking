import numpy as np
from dummy_planner import DummyPlanner
from rrt import RRT
try:
     from ompl import base as ob
except ImportError:
     # if the ompl module is not in the PYTHONPATH assume it is installed in a
     # subdirectory of the parent directory called "py-bindings."
     from os.path import abspath, dirname, join
     import sys
     sys.path.insert(0, join(dirname(dirname(abspath(__file__))), 'py-bindings'))
     from ompl import base as ob

class GlobalPlanner:
    def __init__(self, planner_type="dummy"):

        # initializing the planner based on type
        self.planner = self._initialize_planner(planner_type)

    def _initialize_planner(self, planner_type):
        """
        initializes the global planner based on the given type
        """
        if planner_type == "rrt":
            return RRT()

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
        return self.planner.plan(start,goal)

if __name__ == "__main__":
    obj=GlobalPlanner("rrt")
    space = ob.SE2StateSpace()
    start = ob.State(space)
    start().setX(0)
    start().setY(0.0)
    start().setYaw(0.0)
  
     # create a goal state
    goal = ob.State(space)
    goal().setX(5.0)
    goal().setY(0.5)
    goal().setYaw(5.0)

    print(obj.plan_global_path(start,goal))




    
