import numpy as np
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
