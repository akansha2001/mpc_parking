import numpy as np
import matplotlib.pyplot as plt
import pygame
import threading
import sys

VIS_HZ = 10
PIXELS_PER_METER = 10  # Adjust this value based on your needs

def meters_to_pixels(meters):
    return int(meters * PIXELS_PER_METER)

class Obstacle:
    OBSTACLE_TYPES = ["wall", "box", "circle"]
    def __init__(self, obstacle_type, position, size):
        if obstacle_type not in self.OBSTACLE_TYPES:
            raise ValueError(f"invalid obstacle! Allowed types: {', '.join(self.OBSTACLE_TYPES)}")
        self.type = obstacle_type
        self.position = position
        self.size = size

    def get_position(self):
        return self.position

    def get_size(self):
        return self.size

class Visualizer:
    def __init__(self, live_plot_flag = True, robot_position = np.zeros(3), obstacles = [], grid_size = (800,600)):
        self.live_plot_flag = live_plot_flag
        self.hz = VIS_HZ
        # assigning empty paths
        self.actual_path = []   # actual path traced by the robot
        self.paths = []         # list of paths that you want to plot
        self.grid_size = grid_size
        self.lw = 0.05          # 5 cm
        self.update(robot_position, obstacles)

        # initializing pygame here
        pygame.init()
        self.screen = pygame.display.set_mode(grid_size)
        pygame.display.set_caption("Parking Lot Visualization")

        # launching the game loop in a separate thread
        self.game_thread = threading.Thread(target=self.game_loop)
        self.game_thread.daemon = True
        self.game_thread.start()
        
    def update(self, robot_position = [], obstacles = []):
        if len(robot_position) > 0:
            self.robot_position = robot_position
            self.actual_path.append((robot_position[0], robot_position[1]))
        if len(obstacles) > 0:
            self.obstacles = obstacles

    def set_line_width(self, lw):
        self.lw = lw  

    def plot_path(self, path, colour = "red"):
        # TODO: plot the path here
        pass
    
    def plot_point(self, point):
        # TODO: plot the point here
        pass

    def draw_elements(self):
        # clearing the screen
        self.screen.fill((255, 255, 255))
        # drawing obstacles
        for obstacle in self.obstacles:
            x, y, _ = obstacle.get_position()
            x *= PIXELS_PER_METER
            y *= PIXELS_PER_METER

            if obstacle.type == "wall":
                # plotting lines for wall boundaries (leave unimplemented)
                pass
            elif obstacle.type == "box":
                # assuming regular polygons
                size = obstacle.get_size()
                size_x, size_y = size[0] * PIXELS_PER_METER, size[1] * PIXELS_PER_METER
                pygame.draw.rect(self.screen, (0, 0, 255), (x, y, size_x, size_y))
            elif obstacle.type == "circle":
                radius = obstacle.get_size() * PIXELS_PER_METER
                pygame.draw.circle(self.screen, (255, 0, 0), (int(x), int(y)), int(radius))


        # drawing the robot
        pygame.draw.rect(self.screen, (0, 255, 0), (self.robot_position[0], self.robot_position[1], 30, 30))
        # drawing paths
        if len(self.actual_path) > 1:
            pygame.draw.lines(self.screen, pygame.Color("blue"), False, self.actual_path, self.lw)
        pygame.display.flip()

    def game_loop(self):
        # TODO: implement the game loop
        while True:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    pygame.quit()
                    sys.exit()

            # draw stuff
            self.draw_elements()
            # pygame.display.flip()
            pygame.time.Clock().tick(self.hz)
     
if __name__ == "__main__":
    # obstacle list
    obstacles = [
        Obstacle("box", (3, 1, 0), (2, 1)),
        Obstacle("circle", (1, 1, 0), 1)
    ]
    # sample robot
    robot_position = np.array([500, 500, np.radians(45)])
    visualizer = Visualizer(robot_position=robot_position, obstacles=obstacles)
    # Start the game loop
    visualizer.game_thread.join()