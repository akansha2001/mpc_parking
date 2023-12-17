
from mpscenes.obstacles.box_obstacle import BoxObstacle
from mpscenes.obstacles.dynamic_sphere_obstacle import DynamicSphereObstacle
import numpy as np
import random

import os


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

def generate_dynamic_obstacle(obstacle_dict,frac=0.6,sigma_x=1,sigma_y=1,degree=2,duration=10):

    r = obstacle_dict["r"]
    controlPoints = obstacle_dict["controlPoints"]
    controlPoints = [control_point + [r] for control_point in controlPoints]
    n_points = obstacle_dict["n_points"]
    controlPoints = generate_points(controlPoints,n_points,frac,sigma_x,sigma_y)
    splineDict = {
        "degree": degree,
        "controlPoints": controlPoints,
        "duration": duration,
    }
    dynamicSphereDict = {
        "type": "splineSphere",
        "geometry": {"trajectory": splineDict, "radius": r},
    }
    return DynamicSphereObstacle(name="simpleSphere", content_dict=dynamicSphereDict)


# dynamic obstacles: spheres with control points and noisy motion

dynamic_obstacle_dicts = [
    {"r":0.2,
     "controlPoints":[[0.0,0.0],[4.0,0.0],[4.0,3.0],[0.0,0.0]],
     "n_points": [5,5,5] 
     },
     {"r":0.4,
     "controlPoints":[[1.0,0.0],[4.0,5.0],[4.0,3.0]],
     "n_points": [5,5] 
     }

]

dynamic_obstacles = [generate_dynamic_obstacle(dynamic_obstacle_dict) for dynamic_obstacle_dict in dynamic_obstacle_dicts]


# static obstacles: boxes of varying heights and widths

static_obstacle_dicts = [
    {
        'type': 'box',
        'geometry': {
            'position' : [2.0, 0.0, 0.1],
            'width': 0.2,
            'height': 0.2,
            'length': 0.2,
        },
        'movable': False,
        'high': {
            'position' : [5.0, 5.0, 0.1],
            'width': 0.2,
            'height': 0.2,
            'length': 0.2,
        },
        'low': {
            'position' : [0.0, 0.0, 0.1],
            'width': 0.2,
            'height': 0.2,
            'length': 0.2,
        }
    },

    {
        'type': 'box',
        'geometry': {
            'position' : [4.0, -3.0 , 0.2],
            'width': 0.4,
            'height': 0.4,
            'length': 0.4,
        },
        'movable': False,
        'high': {
            'position' : [7.0, 2.0, 0.2],
            'width': 0.4,
            'height': 0.4,
            'length': 0.4,
        },
        'low': {
            'position' : [0.0, 0.0, 0.2],
            'width': 0.4,
            'height': 0.4,
            'length': 0.4,
        }
    },

    {
        'type': 'box',
        'geometry': {
            'position' : [6.0, 0.0, 0.1],
            'width': 0.2,
            'height': 0.2,
            'length': 0.2,
        },
        'movable': False,
        'high': {
            'position' : [5.0, 5.0, 0.1],
            'width': 0.2,
            'height': 0.2,
            'length': 0.2,
        },
        'low': {
            'position' : [0.0, 0.0, 0.1],
            'width': 0.2,
            'height': 0.2,
            'length': 0.2,
        }
    }
]

static_obstacles = [BoxObstacle(name="static_box", content_dict=static_obstacle_dict) for static_obstacle_dict in static_obstacle_dicts]


# walls

wall_length = 10
wall_obstacles_dicts = [

    
    {
        'type': 'box', 
         'geometry': {
             'position': [wall_length/2.0, 0.0, 0.4], 'width': wall_length/3.0, 'height': 0.8, 'length': 0.1
        },
        'high': {
            'position' : [wall_length/2.0, 0.0, 0.4],
            'width': wall_length/3.0,
            'height': 0.8,
            'length': 0.1,
        },
        'low': {
            'position' : [wall_length/2.0, 0.0, 0.4],
            'width': wall_length/3.0,
            'height': 0.8,
            'length': 0.1,
        },
    },
    
    {
        'type': 'box', 
         'geometry': {
             'position': [0.0, wall_length/2.0, 0.4], 'width': 0.1, 'height': 0.8, 'length': wall_length
        },
        'high': {
            'position' : [0.0, wall_length/2.0, 0.4],
            'width': 0.1,
            'height': 0.8,
            'length': wall_length,
        },
        'low': {
            'position' : [0.0, wall_length/2.0, 0.4],
            'width': 0.1,
            'height': 0.8,
            'length': wall_length,
        },
    },
    {
        'type': 'box', 
         'geometry': {
             'position': [0.0, -wall_length/2.0, 0.4], 'width': 0.1, 'height': 0.8, 'length': wall_length
        },
        'high': {
            'position' : [0.0, -wall_length/2.0, 0.4],
            'width': 0.1,
            'height': 0.8,
            'length': wall_length,
        },
        'low': {
            'position' : [0.0, -wall_length/2.0, 0.4],
            'width': 0.1,
            'height': 0.8,
            'length': wall_length,
        },
    },
    {
        'type': 'box', 
         'geometry': {
             'position': [-wall_length/2.0, 0.0, 0.4], 'width': wall_length, 'height': 0.8, 'length': 0.1
        },
        'high': {
            'position' : [-wall_length/2.0, 0.0, 0.4],
            'width': wall_length,
            'height': 0.8,
            'length': 0.1,
        },
        'low': {
            'position' : [-wall_length/2.0, 0.0, 0.4],
            'width': wall_length,
            'height': 0.8,
            'length': 0.1,
        },
    },
]

wall_obstacles = [BoxObstacle(name=f"wall_{i}", content_dict=obst_dict) for i, obst_dict in enumerate(wall_obstacles_dicts)]

