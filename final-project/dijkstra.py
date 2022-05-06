#import numpy as np
from asyncio import start_unix_server
import math
import random
#from numpy import append


class dijkstra:
    def __init__(self, area_max_x, area_max_y, area_min_x, area_min_y, graphresolusion, vehiclesize):
        # define enviroment area
        self.area_max_x = 500
        self.area_max_y = 500
        self.area_min_x = - 500
        self.area_min_y = - 500
        # scaling
        self.graph_resolution = 20
        self.vehiclesize = 10
        # list
        self.obstacle_map = []
        # scaling
        self.x_width = round((self.area_max_x - self.area_min_x ) / self.graph_resolution)
        self.y_width = round((self.area_max_y  - self.area_min_y ) / self.graph_resolution)
        self.obstacle_map = [[False for _ in range(self.x_width)] for _ in range(self.y_width)]
        ox, oy = [], []
        self.node_around = []


    class graphNode:
        def __init__(self,index_x,index_y,cost,parent):
            self.index_x = index_x 
            self.index_y = index_y
            self.cost = cost
            self.parent = parent

    def planning(self,sx,sy,gx,gy):

        open, close = dict(), dict()

        startnode = self.graphNode(self.calc_xy_index(sx, self.area_min_x), self.calc_xy_index(sy, self.area_min_y),0,-1)
        goalnode = self.graphNode(self.calc_xy_index(gx, self.area_min_x), self.calc_xy_index(gy, self.area_min_y),0,-1)
        open[self.calc_index(startnode)] = startnode

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
                n_id = self.calc_index(nextnode)

                if n_id in close:
                    continue

                if not self.verifynode(nextnode):
                    continue

                if n_id not in open:
                    open[n_id] = nextnode
                else:
                    if open[n_id].cost >= nextnode.cost:
                        open[n_id] = nextnode

    def calc_position(self, index, minp):
        pos = index * self.graph_resolution + minp
        return pos

    def calc_xy_index(self, position, minp):
        return round((position - minp) / self.graph_resolution)

    def calc_index(self, node):
        node = self.graphNode
        return (node.index_y - self.area_min_y) * self.x_width + (node.index_x - self.area_min_x)

    def verifynode(self, node):
        if node.index_x < self.area_min_x:
            return False
        if node.index_y < self.area_min_y:
            return False
        if node.index_x > self.area_max_x:
            return False
        if node.index_y > self.area_max_y:
            return False
        if self.obstacle_map[node.index_x][node.index_y]:
            return False

        return True

    def generate_obstacles(self):

        for i in range(100):
            random.seed(10)
            self.ox.append(random.uniform(-500,500))
            self.oy.append(random.uniform(-500,500))

        return self.ox, self.oy

    def generate_obstaclemap(self):
        for ix in range(self.x_width):
            x = self.calc_position(ix, self.area_min_x)
            for iy in range(self.y_width):
                y = self.calc_position(iy, self.area_min_y)
                for iox,ioy in zip(self.ox,self.oy):
                    d = math.hypot(iox - x, ioy - y)
                    if d <= self.vehicle_radius:
                        self.obstacle_map[ix][iy] = True
                        break

    def node_around(self):
        node_around = [[1, 0, 1],
                        [0, 1, 1],
                        [-1, 0, 1],
                        [0, -1, 1],
                        [-1, -1, math.sqrt(2)],
                        [-1, 1, math.sqrt(2)],
                        [1, -1, math.sqrt(2)],
                        [1, 1, math.sqrt(2)]]
        return node_around

def main():
    print("hogehoge")


if __name__ == '__main__':
    main()





