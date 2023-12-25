# from rrt import RRT
import numpy as np
from dummy_planner import DummyPlanner
from rrt import RRT
from obstacles import static_obstacles, wall_obstacles
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
        elif planner_type == "dummy":
            return DummyPlanner()
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

if __name__ == "__main__":
    global_obj=GlobalPlanner(planner_type="rrt")
    start_goal=np.array([-6.0,0.0,0.0])
    dummy_goal=np.array([6.0,8,0.0])
    global_obj.plan(start_goal, dummy_goal)    
