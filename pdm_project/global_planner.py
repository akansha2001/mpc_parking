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

    def plan_global_path(self, x_start,y_start,yaw_start,x_goal,y_goal,yaw_goal):
        """
        Plan a global path from the given start to the goal.

        Args:
        - start: start configurtion.
        - goal: soal configuration.

        Returns:
        - path: list of waypoints (np arrays) representing the global path.
        """
        return self.planner.plan(x_start,y_start,yaw_start,x_goal,y_goal,yaw_goal)

if __name__ == "__main__":
    obj=GlobalPlanner("rrt")
    x_start=0.0
    y_start=0.0
    yaw_start=0.0
    x_goal=5.0
    y_goal=10.0
    yaw_goal=3.14
    print(obj.plan_global_path(x_start,y_start,yaw_start,x_goal,y_goal,yaw_goal))




    
