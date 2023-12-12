import numpy as np

class Trajectory:

    def __init__(self,cx,cy):
        self.cx = cx
        self.cy = cy
        
    def search_target_index(self, state):

        def closest_index_on_trajectory():
            dx = self.cx - state.rear_x
            dy = self.cy - state.rear_y
            return np.argmin(np.hypot(dx, dy))

        def look_ahead_idx_from(closest_index):
            target_index = closest_index
            while look_ahead_distance(state.v) > \
                    state.distance_from_rear_axle(self.cx[target_index], self.cy[target_index]):
                if (target_index + 1) >= len(self.cx):
                    break  # not exceed goal
                target_index += 1
            return target_index

        return look_ahead_idx_from(closest_index_on_trajectory())
    
    