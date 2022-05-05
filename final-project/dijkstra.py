#import numpy as np
from asyncio import start_unix_server
import math

# initial settings

start_x = 0.0
start_y = 0.0

goal_x = 50.0
goal_y = 0.0

open = dict()
close = dict()

obstacle_map = []

graph_resolution = 2.0
vehicle_radius = 1.0

# define area
min_x = -500
min_y = -500
max_x = 500
max_y = 500

x_width = round((max_x - min_x) / graph_resolution)
y_width = round((max_y - min_y) / graph_resolution)

obstacle_map = [[False for _ in range(x_width)] for _ in range(y_width)]

print(obstacle_map)

