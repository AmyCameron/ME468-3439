#import numpy as np
from asyncio import start_unix_server
import math
import random

from numpy import append

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
ox, oy = [], []

for i in range(100):
    random.seed(10)
    ox.append(random.uniform(-500,500))
    oy.append(random.uniform(-500,500))

def calc_position(index, minp):
    pos = index * graph_resolution + minp
    return pos

def calc_xy_index(position, minp):
    return round((position - minp) / graph_resolution)

for ix in range(x_width):
    x = calc_position(ix, min_x)
    for iy in range(y_width):
        y = calc_position(iy, min_y)
        for iox,ioy in zip(ox,oy):
            d = math.hypot(iox - x, ioy - y)
            if d <= vehicle_radius:
                obstacle_map[ix][iy] = True
                break



print(obstacles[50])


