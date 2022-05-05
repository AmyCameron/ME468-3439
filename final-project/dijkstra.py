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

x_width = 


#obstacle_map = [[for _ in range(20)] for _ in range(20)]

ox, oy = [], []

for i in range(-10, 60):
    ox.append(i)
    oy.append(-10.0)
for i in range(-10, 60):
        ox.append(60.0)
        oy.append(i)
for i in range(-10, 61):
        ox.append(i)
        oy.append(60.0)
for i in range(-10, 61):
        ox.append(-10.0)
        oy.append(i)
for i in range(-10, 40):
        ox.append(20.0)
        oy.append(i)
for i in range(0, 40):
        ox.append(40.0)
        oy.append(60.0 - i)

min_x = round(min(ox))
min_y = round(min(oy))
max_x = round(max(ox))
max_y = round(max(oy))

x_width = round((max_x - min_x) / resolution)
y_width = round((max_y - min_y) / resolution)

def calc_position(index, minp):
    pos = index * resolution + minp
    return pos

obstacle_map = [[False for _ in range(y_width)] for _ in range(x_width)]
for ix in range(x_width):
    x = calc_position(ix, min_x)
    for iy in range(y_width):
        y = calc_position(iy, min_y)
        for iox, ioy in zip(ox, oy):
            d = math.hypot(iox - x, ioy - y)
            if d <= robot_radius:
                obstacle_map[ix][iy] = True
                break

class Node:
    def __init__(self, x, y, cost, parent_index):
        self.x = x  # index of grid
        self.y = y  # index of grid
        self.cost = cost
        self.parent_index = parent_index
    def __str__(self):
        return str(self.x) + ",,," + str(self.y) + "," + str(
            self.cost) + "," + str(self.parent_index)

start_node = Node(2,4,0,-1)
sample_node = Node(1,2,9,0)
sample_node_1 = Node(4,7,2,0)
sample_node_2 = Node(3,2,-2,0)

open_set = dict()
open_set[5] = start_node
open_set[2] = sample_node
open_set[3] = sample_node_1
open_set[4] = sample_node_2



motion = [[1, 0, 1],
            [0, 1, 1],
            [-1, 0, 1],
            [0, -1, 1],
            [-1, -1, math.sqrt(2)],
            [-1, 1, math.sqrt(2)],
            [1, -1, math.sqrt(2)],
            [1, 1, math.sqrt(2)]]


#for move_x, move_y, move_cost in motion:
 #   print(move_x, move_y, move_cost)
 #   print("***")


#print("hogehoge")

#print(motion)
#print(y)
#print(obstacle_map)

for i in range(20):
    print(i)

    if i%2 == 0:
        continue

    print(i, "= kisuu")




c_id = min(open_set, key=lambda o: open_set[o].cost)
#print(open_set)
#print("koredayo")
#print(c_id)



