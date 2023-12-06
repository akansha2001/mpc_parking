# from rrt import RRT
import numpy as np
from dummy_planner import DummyPlanner
class GlobalPlanner:
    def __init__(self, planner_type="dummy"):
        # initializing the planner based on type
        self.planner = self._initialize_planner(planner_type)

    def _initialize_planner(self, planner_type):
        """
        initializes the global planner based on the given type
        """
        if planner_type == "rrt":
            pass
            # return RRTPlanner()  # replace with the actual RRT planner class
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
        path = self.planner.plan(start, goal)
        return path
