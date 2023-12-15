# from rrt import RRT
import numpy as np
from dummy_planner import DummyPlanner
from rrt import RRT
class GlobalPlanner:
    def __init__(self, planner_type="dummy",obstacles=None):
        # initializing the planner based on type
        self.planner = self._initialize_planner(planner_type)
        self.obstacles=obstacles
    def _initialize_planner(self, planner_type):
        """
        initializes the global planner based on the given type
        """
        if planner_type == "rrt":
            return RRT(obstacles=self.obstacles)  # replace with the actual RRT planner class
        elif planner_type == "dummy":
            return DummyPlanner()
        else:
            raise ValueError(f"Invalid planner type: {planner_type}")

    def plan_global_path(self, start, goal):
        """
        Plan a global path from the given start to the goal.

        Args:
        - start: start configurtion.
        - goal: soal configuration.

        Returns:
        - path: list of waypoints (np arrays) representing the global path.
        """
        return self.planner.plan(start, goal)


    
