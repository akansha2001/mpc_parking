import numpy as np
# Import do_mpc package:
import do_mpc
from trajectory import Trajectory

class MPC:
    def __init__(self, trajectory: Trajectory):
        self.trajectory = trajectory 
    
    def plan(self, state):
        pass