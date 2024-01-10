import numpy as np
from helper import FileOp
class State:
    def __init__(self, position=np.zeros(3), forward_velocity=np.zeros(2),
                 velocity=np.zeros(3), steering=np.zeros(1), L = 0):
        # assigning class members
        self.position = np.array(position)
        self.forward_velocity = np.array(forward_velocity)
        self.velocity = np.array(velocity)
        self.steering = np.array(steering)
        self.L = L

    def distance_from_rear_axle(self, x: float, y: float):
        # computing the Euclidean distance to a point from the rear axle
        dist = np.sqrt((x - self.get_rear_x())**2 + (y - self.get_rear_y())**2)
        return dist
    
    def get_yaw(self):
        return self.position[2]
    
    def get_forward_velocity(self):
        return self.forward_velocity[0]
    
    def get_rear_x(self):
        rear_x = self.position[0] - (self.L / 2) * np.cos(self.position[2])
        return rear_x

    def get_rear_y(self):
        rear_y = self.position[1] - (self.L / 2) * np.sin(self.position[2])
        return rear_y

    def get_distance(self, x, y):
        dx = x - self.position[0]
        dy = y - self.position[1]
        return np.hypot(dx, dy)
    
class Trajectory:
    def __init__(self, points):
        # converting to an np array
        try:
            points = np.array(points)
            print("Trajectory input:",points)
        except (TypeError, ValueError) as e:
            raise ValueError(f"error converting input to an np array: {e}")

        # checking whether the trajectory is of the shape ->
        # np.array([[x1, y1], ...]) or np.array([[x1, y1, theta1], ...])
        if points.shape[1] == 2 or points.shape[1] == 3:
            # if shapes are fine, extract coordinates
            self.cx = points[:, 0]  # x
            self.cy = points[:, 1]  # y
            if points.shape[1] == 3:
                self.ct = points[:, 2]  # theta
        else:
            raise ValueError(
                "Invalid shape for position array. It should be either (n, 2) or (n, 3).")
        

    @staticmethod
    def toSE2(x, y, theta):
        cos_theta = np.cos(theta)
        sin_theta = np.sin(theta)

        se2_matrix = np.array([[cos_theta, -sin_theta, x],
                              [sin_theta, cos_theta, y],
                              [0, 0, 1]])
        return se2_matrix

def generate_spline(spawn_pos, offset=1.0, turning_radius=1.8):
    points_path=[]
    thetas=np.arange(0,np.pi/2+np.pi/30,np.pi/30)
    for theta in thetas:
        position=np.zeros(3)
        position[0]=spawn_pos[0]+(turning_radius-turning_radius*np.cos(theta))
        position[1]=spawn_pos[1]+turning_radius*np.sin(theta)
        position[2]=np.pi/2-theta
        points_path.append(position)
    position=np.zeros(3)
    position[0]=spawn_pos[0]+offset+turning_radius
    position[1]=spawn_pos[1]+turning_radius
    position[2]=0
    points_path.append(position)
    #print(turning_radius)
    return points_path