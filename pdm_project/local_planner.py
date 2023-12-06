import numpy as np

class LocalPlanner:
    def __init__(self):
        pass

    def get_target(self, feedback):
        dummy_command = np.ones(2)
        dummy_command[1] = 0.1
        return dummy_command