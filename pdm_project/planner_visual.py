import numpy as np
import matplotlib.pyplot as plt
from obstacles import static_obstacles, wall_obstacles
from mpscenes.obstacles.box_obstacle import BoxObstacle
from mpscenes.obstacles.dynamic_sphere_obstacle import DynamicSphereObstacle
from shapely import Polygon

#for plotting polygons
from matplotlib.path import Path
from matplotlib.patches import PathPatch
from matplotlib.collections import PatchCollection
#print(len(static_obstacles))
from shapely import wkt
from matplotlib.patches import Rectangle
from matplotlib.animation import FuncAnimation
from IPython.display import HTML, display

#Polygon functions
from helper import *


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
plt.savefig('simulation/path_plot.png')
plt.show()


# def init():
#     ax.add_patch(patch)
#     return patch,

# def animate(i):
#     x1 = x_actual_path[i] - carLength/2*np.cos(yaw_actual_path[i]) + carWidth/2*np.sin(yaw_actual_path[i])
#     y1 = y_actual_path[i] - carLength/2*np.sin(yaw_actual_path[i]) - carWidth/2*np.cos(yaw_actual_path[i])
#     patch.set_width(carLength)
#     patch.set_height(carWidth)
#     patch.set_xy([x1 , y1])
#     patch.angle = (yaw_actual_path[i])*180/np.pi
#     return patch,

# # Create the animation
# anim = FuncAnimation(fig, animate,init_func=init,frames=len(x_actual_path),interval=100,blit=True)
# anim.save('simulation/animation.html', writer='html')
# # Display the animation
# # display(HTML(anim.to_jshtml()))

# import webbrowser
# webbrowser.open('animation.html')


