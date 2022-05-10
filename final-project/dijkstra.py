import math
import random
import matplotlib
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import csv

class Dijkstra:
    def __init__(self, area_max_x, area_max_y, area_min_x, area_min_y, ox, oy, graph_resolusion, vehiclesize):
        # define enviroment area
        self.area_max_x = area_max_x
        self.area_max_y = area_max_y
        self.area_min_x = area_min_x
        self.area_min_y = area_min_y
        # scaling
        self.graph_resolution = graph_resolusion
        self.vehiclesize = vehiclesize
        self.x_width = None
        self.y_width = None
        # obstable map
        self.obstacle_map = []
        self.generate_obstaclemap(ox,oy)
        # 8 nodes around node
        self.node_around = self.node_around()

    class graphNode:
        def __init__(self,index_x,index_y,cost,parent):
            self.index_x = index_x 
            self.index_y = index_y
            self.cost = cost
            self.parent = parent

    def planning(self,sx,sy,gx,gy):

        startnode = self.graphNode(self.calc_xy_index(sx, self.area_min_x), self.calc_xy_index(sy, self.area_min_y),0,-1)
        goalnode = self.graphNode(self.calc_xy_index(gx, self.area_min_x), self.calc_xy_index(gy, self.area_min_y),0,-1)
        
        open, close = dict(), dict()
        open[self.calc_index(startnode)] = startnode

        while 1:
            c_id = min(open, key=lambda o: open[o].cost)
            current = open[c_id]

            if current.index_x == goalnode.index_x and current.index_y == goalnode.index_y:
                print("planning has done!")
                goalnode.cost = current.cost
                goalnode.parent = current.parent
                break

            del open[c_id]
            close[c_id] = current

            for plus_x,plus_y,plus_cost in self.node_around:
                nextnode = self.graphNode(current.index_x + plus_x, current.index_y + plus_y, current.cost + plus_cost, c_id)
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
        
        fr_x, fr_y  = self.finalresult(goalnode, close)

        return fr_x, fr_y
            

    def finalresult(self, goalnode,close):
        fr_x, fr_y = [self.calc_position(goalnode.index_x, self.area_min_x)], [self.calc_position(goalnode.index_y, self.area_min_y)]
        parent_index = goalnode.parent
        while parent_index != -1:
            n = close[parent_index]
            fr_x.append(self.calc_position(n.index_x, self.area_min_x))
            fr_y.append(self.calc_position(n.index_y, self.area_min_y))
            parent_index = n.parent
        return fr_x, fr_y    
 
    def calc_position(self, index, minp):
        pos = index * self.graph_resolution + minp
        return pos

    def calc_index(self, node):
        return (node.index_y - self.area_min_y) * self.x_width + (node.index_x - self.area_min_x)

    def calc_xy_index(self, position, minp):
        return round((position - minp) / self.graph_resolution)

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


    def generate_obstaclemap(self, ox, oy):
        self.x_width = round((self.area_max_x - self.area_min_x ) / self.graph_resolution)
        self.y_width = round((self.area_max_y  - self.area_min_y ) / self.graph_resolution)
        self.obstacle_map = [[False for _ in range(self.x_width)] for _ in range(self.y_width)]

        for ix in range(self.x_width):
            x = self.calc_position(ix, self.area_min_x)
            for iy in range(self.y_width):
                y = self.calc_position(iy, self.area_min_y)
                for iox,ioy in zip(ox,oy):
                    d = math.hypot(iox - x, ioy - y)
                    if d <= self.vehiclesize:
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

    sx = 20.0
    sy = 4.0
    gx = 9.2 
    gy = 1.0

    graph_resolosion = 2
    vehiclesize = 1.5

    area_min_x, area_max_x = 0, 200
    area_min_y, area_max_y = 0, 200
    
    ox, oy = [], []
    #for i in range(150):
        # random.seed(50)
     #   ox.append(random.randint(area_min_x,area_max_x))
    #  oy.append(random.randint(area_min_y,area_max_y))

    ox = [0.00E+00,
            2.00E-01,
            1.20E+00,
            2.20E+00,
            3.20E+00,
            4.20E+00,
            5.20E+00,
            0.00E+00,
            2.00E-01,
            1.20E+00,
            2.20E+00,
            3.20E+00,
            4.20E+00,
            5.20E+00,
            0,
            0,
            0,
            0,
            0,
            5.20E+00,
            5.20E+00,
            5.20E+00,
            5.20E+00,
            5.20E+00,
            1.32E+01,
            1.42E+01,
            1.52E+01,
            1.62E+01,
            1.72E+01,
            1.82E+01,
            1.92E+01,
            1.32E+01,
            1.42E+01,
            1.52E+01,
            1.62E+01,
            1.72E+01,
            1.82E+01,
            1.92E+01,
            1.32E+01,
            1.32E+01,
            1.32E+01,
            1.32E+01,
            1.32E+01,
            1.82E+01,
            1.92E+01,
            1.92E+01,
            1.92E+01,
            1.92E+01,
            0.00E+00,
            1.00E+00,
            2.00E+00,
            3.00E+00,
            4.00E+00,
            5.00E+00,
            6.00E+00,
            7.00E+00,
            8.00E+00,
            9.00E+00,
            1.00E+01,
            1.10E+01,
            1.20E+01,
            1.30E+01,
            1.40E+01,
            1.50E+01,
            1.60E+01,
            1.70E+01,
            1.80E+01,
            1.90E+01,
            2.00E+01]
    oy = [0,
            0,
            0,
            0,
            0,
            0,
            0,
            2.00E+00,
            2.00E+00,
            2.00E+00,
            2.00E+00,
            2.00E+00,
            2.00E+00,
            2.00E+00,
            0.00E+00,
            5.00E-01,
            1.00E+00,
            1.50E+00,
            2.00E+00,
            0.00E+00,
            5.00E-01,
            1.00E+00,
            1.50E+00,
            2.00E+00,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            2.00E+00,
            2.00E+00,
            2.00E+00,
            2.00E+00,
            2.00E+00,
            2.00E+00,
            2.00E+00,
            0.00E+00,
            5.00E-01,
            1.00E+00,
            1.50E+00,
            2.00E+00,
            0.00E+00,
            5.00E-01,
            1.00E+00,
            1.50E+00,
            2.00E+00,
            6.00E+00,
            6.00E+00,
            6.00E+00,
            6.00E+00,
            6.00E+00,
            6.00E+00,
            6.00E+00,
            6.00E+00,
            6.00E+00,
            6.00E+00,
            6.00E+00,
            6.00E+00,
            6.00E+00,
            6.00E+00,
            6.00E+00,
            6.00E+00,
            6.00E+00,
            6.00E+00,
            6.00E+00,
            6.00E+00,
            6.00E+00]

    dijkstra = Dijkstra(area_max_x, area_max_y, area_min_x, area_min_y, ox, oy, graph_resolosion, vehiclesize)
    rx, ry = dijkstra.planning(sx, sy, gx, gy)

    print(rx)
    print(ry)
    print(ox)
    print(oy)

    fig = plt.figure(figsize=(8, 6))

    plt.plot(rx, ry,label='result')
    plt.scatter(ox, oy,label='obstacle')
    plt.legend()
    plt.title("Dijkstra",fontsize=12)
    plt.show()

if __name__ == '__main__':
    main()