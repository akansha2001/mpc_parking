import numpy as np

class LocalPlanner:
    def __init__(self):
        pass

    def set_trajectory(self,trajectory):

        self.trajectory = trajectory 
        self.look_ahead_time = 1.0

    def plan(self,state,trajectory):

        trajectory = self.trajectory


        # Find the look ahead index from the trajectory
        ind = trajectory.search_target_index(state)
        
        # index normalization making sure we don't overshoot the goal.
        if ind < len(trajectory.cx):
            tx = trajectory.cx[ind]
            ty = trajectory.cy[ind]
        else:  # toward goal
            tx = trajectory.cx[-1]
            ty = trajectory.cy[-1]
            ind = len(trajectory.cx) - 1
        
        # Calculate the slope of look ahead distance line which would be alpha.
        # If the car was heading along the velocity vector then that would be it but the 
        # car has a yaw from the heading vector and hence we need to subtract this
        # from slooe to get actual alpha
        alpha = np.arctan2(ty - state.rear_y, tx - state.rear_x) - state.yaw

        # steering angle = invtan(2Lsin(alpha)/Ld)
        # as established steering control law for pure pursuit controller
        delta = np.arctan2(2.0 * self.L * np.sin(alpha) / self.look_ahead_time * state.v, 1.0)
        omega = 2*np.sin(alpha)/self.look_ahead_time

        return omega, ind

    def get_target(self, feedback):
        dummy_command = np.ones(2)
        dummy_command[1] = 0.1
        return dummy_command