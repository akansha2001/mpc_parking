# from rrt import RRT
import numpy as np
from dummy_planner import DummyPlanner
from rrt import RRT
from obstacles import static_obstacles
class GlobalPlanner:
    def __init__(self, planner_type="dummy",obstacles=None):
        # initializing the planner based on type
        self.obstacles=static_obstacles
        self.planner = self._initialize_planner(planner_type,obstacles=self.obstacles)
    def _initialize_planner(self, planner_type,obstacles=None):
        """
        initializes the global planner based on the given type
        """
        if planner_type == "rrt":
            return RRT(obstacles)  # replace with the actual RRT planner class
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

if __name__ == "__main__":
    global_obj=GlobalPlanner(planner_type="rrt")
    start_goal=np.array([-2.0,0.0,0.0])
    dummy_goal=np.array([10.0,0.0,0.0])
    global_obj.plan_global_path(start_goal, dummy_goal)    
