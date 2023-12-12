import numpy as np
from trajectory import Trajectory

# TODO: chang local planner to pure pursuit using inheritance
class LocalPlanner:
    def __init__(self, trajectory: Trajectory, max_vel = 5):
        self.trajectory = trajectory 
        self.max_vel = max_vel        
        #! HARDCODED
        self.look_ahead_time = 5.0
        self.speed_factor =  0.05

    
    def plan(self, state):
        # extracting necessary state variables
        u = state.get_forward_velocity()
        yaw = state.get_yaw()
        rear_x = state.get_rear_x()
        rear_y = state.get_rear_y()

        # finding the look ahead index from the trajectory
        look_ahead_dist = self.look_ahead_time * u
        # finding the closest index on the trajectory
        dx = self.trajectory.cx - rear_x
        dy = self.trajectory.cy - rear_y
        
        closest_index = np.argmin(np.hypot(dx, dy))
        
        # closest index on the trajectory
        ind = closest_index
        while look_ahead_dist > \
                state.distance_from_rear_axle(self.trajectory.cx[ind], self.trajectory.cy[ind]):
            if (ind + 1) >= len(self.trajectory.cx):
                break  # not exceed goal
            ind += 1
        
        # index normalization making sure we don't overshoot the goal.
        if ind < len(self.trajectory.cx):
            tx = self.trajectory.cx[ind]
            ty = self.trajectory.cy[ind]
        else:  # toward goal
            tx = self.trajectory.cx[-1]
            ty = self.trajectory.cy[-1]
            ind = len(self.trajectory.cx) - 1
        
        # Calculate the slope of look ahead distance line which would be alpha.
        # If the car was heading along the velocity vector then that would be it but the 
        # car has a yaw from the heading vector and hence we need to subtract this
        # from slope to get actual alpha
        alpha = np.arctan2(ty - rear_y, tx - rear_x) - yaw

        # steering angle = invtan(2Lsin(alpha)/Ld)
        # as established steering control law for pure pursuit controller
        delta = np.arctan2(2.0 * state.L * np.sin(alpha) / look_ahead_dist, 1.0)
        cmd_omega = 2 * np.sin(alpha)/self.look_ahead_time
        cmd_vel = self.speed_factor * self.max_vel
        self.trajectory.cx = self.trajectory.cx[closest_index:]
        self.trajectory.cy = self.trajectory.cy[closest_index:]
        
        return np.array([cmd_vel, cmd_omega])
        # return np.array([cmd_vel, 0.1])