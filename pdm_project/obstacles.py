from mpscenes.obstacles.box_obstacle import BoxObstacle
from mpscenes.obstacles.dynamic_sphere_obstacle import DynamicSphereObstacle
import numpy as np
import random
import os

def create_obstacle_objects(positions, widths, height, lengths):
    """Creates a list of BoxObstacle objects from lists of positions, widths, and heights."""
    obstacle_objects = [
        BoxObstacle(name=f"wall_{i}", content_dict={
            'type': 'box',
            'geometry': {
                'position': positions[i],
                'width': widths[i],
                'height': height,
                'length': lengths[i]
            },
            'high': {
                'position': positions[i],
                'width': widths[i],
                'height': height,
                'length': lengths[i],
            },
            'low': {
                'position': positions[i],
                'width': widths[i],
                'height': height,
                'length': lengths[i],
            },
            'rgba': [0.1, 0.3, 0.3, 1.0],
        }) for i in range(len(positions))
    ]

    return obstacle_objects


def generate_arithmetic_sequence(center, difference, n):
    """Generates an arithmetic sequence based on the center, difference, and length."""
    if n % 2 == 0:
        center -= difference / 2  # If the sequence length is even, adjust the center to the left
    tmp = [center + i * difference for i in range(-(n // 2), n // 2 + 1)]
    tmp = list(map(lambda x: round(x, 2), tmp))
    sequence = sorted(tmp)
    return sequence


def create_parking_positions(length_x, length_y, height, road_width):
    """Creates parking positions based on length_x, length_y, height, and road_width."""
    parking_lot_y = generate_arithmetic_sequence(0, length_y, 9)
    parking_lot_x = [-(length_x * 1.5 + road_width), -length_x / 2, length_x / 2, length_x * 1.5 + road_width]

    parking_positions = []
    for i in range(len(parking_lot_x)):
        for j in range(len(parking_lot_y)):
            position = [parking_lot_x[i], parking_lot_y[j], height / 2]
            parking_positions.append(position)
    print(parking_positions)
    return parking_positions


def generate_scene():
    '''WALLS'''
    wall_length_y = 7.59  # wall length along x -- half length
    wall_length_x = 6.6  # wall length along y -- half length
    wall_height = 1
    wall_thickness = 0.1
    mid_wall_length = 1.2*7
    wall_positions = [[wall_length_x, 0.0, wall_height / 2],
                      [0.0, wall_length_y, wall_height / 2],
                      [0.0, -wall_length_y, wall_height / 2],
                      [5.7, -wall_length_y, wall_height / 2],
                      [-5.7, -wall_length_y, wall_height / 2],
                      [-wall_length_x, 0.0, wall_height / 2],
                      [0, 0, wall_height / 2]]
    wall_widths = [wall_length_y * 2, wall_thickness, wall_thickness, wall_thickness, wall_thickness,
                   wall_length_y * 2, mid_wall_length]  # length along y-axis
    wall_lengths = [wall_thickness, wall_length_x * 2, 1.8 * 2, 1.8, 1.8, wall_thickness, wall_thickness]  # length along x-axis

    '''PARKING LIMITS'''
    parking_length_x = 1.8
    parking_length_y = 1.2
    parking_height = 0.25
    road_width = 3

    parking_positions = create_parking_positions(parking_length_x, parking_length_y, parking_height, road_width)
    parking_widths = [wall_thickness] * len(parking_positions)
    parking_lengths = [parking_length_x] * len(parking_positions)
    static_obstacles = create_obstacle_objects(parking_positions, parking_widths, parking_height, parking_lengths)
    dynamic_obstacles = []
    wall_obstacles = create_obstacle_objects(wall_positions, wall_widths, wall_height, wall_lengths)

    return static_obstacles, dynamic_obstacles, wall_obstacles


static_obstacles, dynamic_obstacles, wall_obstacles = generate_scene()


