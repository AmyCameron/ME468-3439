#import numpy as np
from asyncio import start_unix_server
import math
import random
#from numpy import append
print("OK")

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



class graphNode:
    def __init__(self,index_x,index_y,cost,parent):
        self.index_x = index_x 
        self.index_y = index_y
        self.cost = cost
        self.parent = parent

def calc_position(index, minp):
    pos = index * graph_resolution + minp
    return pos

def calc_xy_index(position, minp):
    return round((position - minp) / graph_resolution)

def calc_index(node):
    node = graphNode
    return (node.index_y - min_y) * x_width + (node.index_x - min_x)

def verifynode(node):
    if node.index_x < min_x:
        return False
    if node.index_y < min_y:
        return False
    if node.index_x > max_x:
        return False
    if node.index_y > max_y:
        return False
    if obstacle_map[node.index_x][node.index_y]:
        return False

    return True


for i in range(100):
    random.seed(10)
    ox.append(random.uniform(-500,500))
    oy.append(random.uniform(-500,500))


for ix in range(x_width):
    x = calc_position(ix, min_x)
    for iy in range(y_width):
        y = calc_position(iy, min_y)
        for iox,ioy in zip(ox,oy):
            d = math.hypot(iox - x, ioy - y)
            if d <= vehicle_radius:
                obstacle_map[ix][iy] = True
                break

node_around = [[1, 0, 1],
                [0, 1, 1],
                [-1, 0, 1],
                [0, -1, 1],
                [-1, -1, math.sqrt(2)],
                [-1, 1, math.sqrt(2)],
                [1, -1, math.sqrt(2)],
                [1, 1, math.sqrt(2)]]

startnode = graphNode(calc_xy_index(start_x, min_x), calc_xy_index(start_y, min_y),0,-1)
goalnode = graphNode(calc_xy_index(goal_x, min_x), calc_xy_index(goal_y, min_y),0,-1)
open[calc_index(startnode)] = startnode

while 1:
    c_id = min(open, key=lambda o: open[o].cost)
    current = open[c_id]

    if current.index_x == goalnode.index_x and current.index_y == goalnode.index_y:
        goalnode.cost = current.cost
        goalnode.parent = current.parent
        break

    del open[c_id]

    for plus_x,plus_y,plus_cost in node_around:
        nextnode = (current.index_x + plus_x,
                    current.index_y + plus_y,
                    current.cost + plus_cost,
                    c_id)
        n_id = calc_index(nextnode)

        if n_id in close:
            continue

        if not verifynode(nextnode):
            continue

        if n_id not in open:
            open[n_id] = nextnode
        else:
            if open[n_id].cost >= nextnode.cost:
                open[n_id] = nextnode





