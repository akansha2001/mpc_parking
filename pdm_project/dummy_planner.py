import numpy as np
class DummyPlanner:
    def __init__(self):
        pass

    def plan(self, start: np.ndarray, goal: np.ndarray):
        path = [goal]
        return path