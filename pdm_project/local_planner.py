import numpy as np
from abc import ABC, abstractmethod
from trajectory import Trajectory

class LocalPlanner(ABC):
    @abstractmethod
    def plan(self, robot):
        pass

class DummyLocalPlanner(LocalPlanner):
    def __init__(self):
        super().__init__()

    def plan(self, robot):
        # returns zero velocities
        cmd = np.array([0.0, 0.0])
        return cmd
class PurePursuit(LocalPlanner):
    def __init__(self, trajectory: Trajectory, max_vel = 5):
        self.trajectory = trajectory
        #print("Pure pursuit input trajectory",trajectory) 
        self.max_vel = max_vel        
        #! HARDCODED
        self.look_ahead_time = 1.0
        self.speed_factor =  0.35
        self.look_ahead_thresh = 0.1
        self.stop_thresh = 0.1
        self.Kp = 2.0

    def plan(self, robot):
        state = robot.state
        # extracting necessary state variables
        u = state.get_forward_velocity()
        yaw = state.get_yaw()
        rear_x = state.get_rear_x()
        rear_y = state.get_rear_y()

        # finding the look ahead index from the trajectory
        look_ahead_dist = 1.0
        if u >= 0.5:
            look_ahead_dist = u * self.look_ahead_time
        
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
        #print(tx, ty)
        
        # Calculate the slope of look ahead distance line which would be alpha.
        # If the car was heading along the velocity vector then that would be it but the 
        # car has a yaw from the heading vector and hence we need to subtract this
        # from slope to get actual alpha
        alpha = np.arctan2(ty - rear_y, tx - rear_x) - yaw

        # checking whether lookahead distance is large enough
        if look_ahead_dist > self.look_ahead_thresh:
            # steering angle = invtan(2Lsin(alpha)/Ld)
            # as established steering control law for pure pursuit controller
            delta = np.arctan2(2.0 * state.L * np.sin(alpha) / look_ahead_dist, 1.0)
        else:
            #print("warning: small lookahead distance!")
            delta = state.steering  # basically no steering input needed
    
        cmd_omega =  self.Kp * float(delta - state.steering[0])
        # cmd_omega = 2 * np.sin(alpha)/self.look_ahead_time
        goal_dist = state.get_distance(self.trajectory.cx[-1], self.trajectory.cy[-1])
        
        # print(goal_dist)
        #print(state.position)
        if goal_dist > self.stop_thresh:
            cmd_vel = self.speed_factor * self.max_vel
            self.trajectory.cx = self.trajectory.cx[closest_index:]
            self.trajectory.cy = self.trajectory.cy[closest_index:]
        else:
            cmd_vel = 0
            cmd_omega = 0        
        cmd = np.array([cmd_vel, cmd_omega])
        return cmd
        # return np.array([cmd_vel, 0.1])

