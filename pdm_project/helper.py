import numpy as np
import csv

import numpy as np
import matplotlib.pyplot as plt


#for plotting polygons
from matplotlib.path import Path
from matplotlib.patches import PathPatch
from matplotlib.collections import PatchCollection
#print(len(static_obstacles))
from matplotlib.patches import Rectangle
from matplotlib.animation import FuncAnimation

# for generate_points()
import random
class bcolors:
    HEADER = '\033[95m'
    OKBLUE = '\033[94m'
    OKCYAN = '\033[96m'
    OKGREEN = '\033[92m'
    WARNING = '\033[93m'
    FAIL = '\033[91m'
    ENDC = '\033[0m'
    BOLD = '\033[1m'
    UNDERLINE = '\033[4m'


class FileOp:
        def __init__(self, file_name):
            self.output_file=file_name
            open(self.output_file, 'w').close()
                
        def write(self,arr):
            with open(self.output_file, 'a') as f:
                self.writer = csv.writer(f)
                self.writer.writerow(['%.5f' % v for v in arr])

# def carPolygon(x,y,yaw): #returns a polygon for the car given a position and heading
#     return Polygon(shell=((x + carLength/2*np.cos(yaw) + carWidth/2*np.sin(yaw), y + carLength/2*np.sin(yaw) - carWidth/2*np.cos(yaw)),
#                      (x - carLength/2*np.cos(yaw) + carWidth/2*np.sin(yaw), y - carLength/2*np.sin(yaw) - carWidth/2*np.cos(yaw)),
#                      (x - carLength/2*np.cos(yaw) - carWidth/2*np.sin(yaw), y - carLength/2*np.sin(yaw) + carWidth/2*np.cos(yaw)),
#                      (x + carLength/2*np.cos(yaw) - carWidth/2*np.sin(yaw), y + carLength/2*np.sin(yaw) + carWidth/2*np.cos(yaw))))

# def obstaclePolygon(obstacle):
#     position = np.array([obstacle.position()[0], obstacle.position()[1]])
#     width = obstacle.width()
#     length = obstacle.length()
#     return Polygon(shell=((position[0]+width/2, position[1]+length/2),
#                                     (position[0]+width/2, position[1]-length/2),
#                                     (position[0]-width/2, position[1]-length/2),
#                                     (position[0]-width/2, position[1]+length/2)))

# def wallPolygon(obstacle):
#     position = np.array([obstacle.position()[1], obstacle.position()[0]])
#     width = obstacle.width()
#     length = obstacle.length()
#     return Polygon(shell=((position[0]+width/2, position[1]+length/2),
#                                     (position[0]+width/2, position[1]-length/2),
#                                     (position[0]-width/2, position[1]-length/2),
#                                     (position[0]-width/2, position[1]+length/2)))
 
# def plot_polygon(ax, poly, **kwargs):
#     path = Path.make_compound_path(
#         Path(np.asarray(poly.exterior.coords)[:, :2]),
#         *[Path(np.asarray(ring.coords)[:, :2]) for ring in poly.interiors])

#     patch = PathPatch(path, **kwargs)
#     collection = PatchCollection([patch], **kwargs)
    
#     ax.add_collection(collection, autolim=True)
#     ax.autoscale_view()
#     return collection    

class Plot:
    def __init__(self):
        pass

    @staticmethod
    def plot_path():
        # Load the path data from the generated text file
        path_data = np.genfromtxt('data/path_output.txt', delimiter=', ')

        # Extract x, y coordinates from the path data
        x_path = path_data[:, 0]
        y_path = path_data[:, 1]
        yaw_path = path_data[:,2]

        actual_path_data = np.genfromtxt('data/robot_pos.csv', delimiter=',')
        # Extract x, y coordinates from the actual path
        x_actual_path = actual_path_data[:, 0]
        y_actual_path = actual_path_data[:, 1]
        yaw_actual_path = actual_path_data[:,2]


        #geometry of the car
        carLength = 4.599 * 0.3
        carWidth = 1.782 * 0.3

        # list_intersection = []
        # list_cross = []

        # for i in range(len(x_path)):
        #     car_polygon = carPolygon(x_path[i], y_path[i], yaw_path[i])
        #     for obstacles in static_obstacles:
        #         obstacle_polygon = obstaclePolygon(obstacles)
        #         intersection = obstacle_polygon.intersects(car_polygon)
        #         #cross = car_polygon.crosses(obstacle_polygon)
        #         list_intersection.append(intersection)
        #         #list_cross.append(cross)
            

        # print(list_intersection)

        # Initialize empty plot for the car
        #patch = patches.Polygon(v,closed=True, fc='r', ec='r')

        # Initialize empty plot for the path
        fig, ax = plt.subplots()
        line1 = ax.plot(x_path, y_path, label='Planned Path')[0]
        line2 = ax.plot(x_actual_path, y_actual_path, label='Actual Path')[0]

        # Initialize empty plot for the car
        patch = plt.Rectangle((x_path[0] - carWidth / 2, y_path[0] - carLength / 2), carWidth, carLength,
                            angle=np.degrees(yaw_path[0]), edgecolor='r', facecolor='none', label='Car')
        ax.add_patch(patch)

        plt.plot(x_path[0], y_path[0], 'go', label = 'Start')
        plt.plot(x_path[-1], y_path[-1], 'bo', label = 'End')
        # Initialize obstacle plots
        obstacle_plots = []
        wall_plots = []

        for obstacle in static_obstacles:
            obstacle_position = np.array([obstacle.position()[0], obstacle.position()[1]])
            obstacle_width = obstacle.width()
            obstacle_height = obstacle.length()
            rect = plt.Rectangle((obstacle_position[0] - obstacle_width / 2, obstacle_position[1] - obstacle_height / 2),
                                obstacle_width, obstacle_height, color='red', alpha=0.5)
            obstacle_plots.append(rect)
            ax.add_patch(rect)
        for obstacle in wall_obstacles:
            obstacle_position = np.array([obstacle.position()[1], obstacle.position()[0]])
            obstacle_width = obstacle.width()
            obstacle_height = obstacle.length()
            rect = plt.Rectangle((obstacle_position[0] - obstacle_width / 2, obstacle_position[1] - obstacle_height / 2),
                                    obstacle_width, obstacle_height, color='black', alpha=0.5)
            wall_plots.append(rect)

        # Set plot limits and labels
        plt.xlim([-12, 12])
        plt.ylim([-12, 12])
        plt.xlabel('X-axis')
        plt.ylabel('Y-axis')
        plt.title('Path Planning with Obstacles')
        plt.legend()
        plt.minorticks_on()
        plt.grid(which='minor', linestyle=':', linewidth='0.5')
        plt.savefig('data/path_plot.png')
        plt.show()


def generate_points(controlPoints,n_points,frac,sigma_x,sigma_y):

    new_control_points = []
    z = controlPoints[0][-1]

    for i in range(len(controlPoints)-1):
        p1 = controlPoints[i][:2]
        p2 = controlPoints[i+1][:2]

        x1 = p1[0]
        y1 = p1[1]
        x2 = p2[0]
        y2 = p2[1]

        points_x = [x1 + j*(x2 - x1)/(n_points[i] + 1) for j in range(n_points[i])]
        points_y = [y1 + j*(y2 - y1)/(n_points[i] + 1) for j in range(n_points[i])]

        chosen_indices = random.sample([j for j in range(n_points[i])], k = int(frac*n_points[i]))
        chosen_indices.sort()

        chosen_points_x = [points_x[j] + sigma_x*random.random() for j in chosen_indices]
        chosen_points_y = [points_y[j] + sigma_y*random.random() for j in chosen_indices]

        for j in range(len(chosen_points_x)):
            new_control_points.append([chosen_points_x[j],chosen_points_y[j],z])
    return new_control_points

def func_arc(x_initial,b,R,theta):
    points_path=[]
    flag=1
    if theta==0:
        flag=-1
    for theta in np.arange(0,np.pi/2,np.pi/30):
        yaw=np.pi/2+theta*flag
        x=x_initial[0]+(R-R*np.cos(theta))*flag
        y=x_initial[1]+R*np.sin(theta)
        points_path.append([x,y,yaw])
    x_parking_spot=np.zeros(3)
    x_parking_spot[0]=x_initial[0]-(R+b)*flag
    x_parking_spot[1]=x_initial[1]+R
    x_parking_spot[2]=np.pi/2 + flag*(np.pi/2)
    points_path.append(x_parking_spot)
    points_path_array=np.array(points_path)
    # print(points_path_array.shape)
    return points_path_array