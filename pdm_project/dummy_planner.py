import numpy as np
class DummyPlanner:
    def __init__(self):
        pass

    def plan(self, start: np.ndarray, goal: np.ndarray):
        # path = [goal]
        path = []
        steps = 100
        # waypoints = [start,goal]
        waypoints = [start + np.array([10.0, 0.0, 0.0]),start + np.array([10.0, 10.0, 0.0]),
                    start + np.array([0.0, 10.0, 0.0])]
        waypoints =[start + np.array([10.0, 10.0, 0.0]),start + np.array([10.0, 10.0, 0.0])]
        
        for i in range(len(waypoints)-1):
            
            for j in range(steps):
                path.append(waypoints[i]+j*(waypoints[i+1]-waypoints[i])/steps)
        
        # path =[start + np.array([10.0, 10.0, 0.0])]
        print(path)
        return path