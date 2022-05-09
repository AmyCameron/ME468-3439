import rclpy
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile
from custom_msgs.msg import VehicleState, ObjectList, Object
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path

from ament_index_python.packages import get_package_share_directory

import math
import random

import os
import numpy as np
import matplotlib
import matplotlib.pyplot as plt
import matplotlib.patches as patches

from scipy.integrate import odeint

class Dijkstra(Node):
    def __init__(self, area_max_x, area_max_y, area_min_x, area_min_y, ox, oy, graph_resolusion, vehiclesize):
        super().__init__('path_planning_node')
        
        # define enviroment area
        self.area_max_x = area_max_x
        self.area_max_y = area_max_y
        self.area_min_x = area_min_x
        self.area_min_y = area_min_y
        # scaling
        self.graph_resolution = graph_resolusion
        self.vehiclesize = vehiclesize
        # obstable map
        self.x_width = None
        self.y_width = None
        self.obstacle_map = []
        self.generate_obstaclemap(ox,oy)
        # 8 nodes around node
        self.node_around = self.node_around()

        # Read in share directory location
        package_share_directory = get_package_share_directory('control_stack')
        
        self.declare_parameter('visualize', False)
        self.visualize = self.get_parameter(
            'visualize').get_parameter_value().bool_value

        self.declare_parameter('path_resolution', 10.0)
        self.path_resolution = self.get_parameter(
            'path_resolution').get_parameter_value().double_value

        ### ??? ###
        #target location
        self.declare_parameter('target', [350.0,350.0])
        self.target = self.get_parameter(
            'target').get_parameter_value().double_array_value

        self.declare_parameter('save', False)
        self.save = self.get_parameter(
            'save').get_parameter_value().bool_value

        self.declare_parameter('save_name', "planner_output")
        self.save_name = self.get_parameter(
            'save_name').get_parameter_value().string_value

        home_dir = os.getenv('HOME')
        save_dir = "me468_output"
        if(not os.path.exists(os.path.join(home_dir,save_dir))):
            os.mkdir(os.path.join(home_dir,save_dir))
            
        self.output_dir = os.path.join(home_dir,save_dir,self.save_name)
        if(not os.path.exists(self.output_dir)):
            os.mkdir(self.output_dir)
        self.frame_number = 0

        # DDS QOS Setup - important for detemining lag and packet drop behavior
        qos_profile = QoSProfile(depth=1)
        qos_profile.history = QoSHistoryPolicy.KEEP_LAST

        # subscribers
        self.sub_state = self.create_subscription(
            VehicleState, 'state', self.state_callback, qos_profile)
        self.sub_objects = self.create_subscription(
            ObjectList, 'objects', self.objects_callback, qos_profile)

        # publishers
        self.pub_path = self.create_publisher(Path, 'path', qos_profile)
        self.timer = self.create_timer(1/self.freq, self.pub_callback)

        # visualization setup
        if(self.visualize): # ???
            self.fig, self.ax = plt.subplots()
            plt.title("Path Planner")
            self.patches = []
            self.ax.set_xlim((-500,500))
            self.ax.set_ylim((-500,500))
            self.ax.set_xlabel("X [m] (Global Frame)")
            self.ax.set_ylabel("Y [m] (Global Frame)")
            self.path_points = None
        ### ??? ###

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

    def calc_index(self, node):
        return (node.index_y - self.area_min_y) * self.x_width + (node.index_x - self.area_min_x)

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

    def pub_callback(self):
        
        pts_x, pts_y = self.planning(sx, sy, gx, gy)
        msg = Path()

        for ix,iy in pts_x,pts_y:
            pt = PoseStamped()
            pt.pose.position.x = ix
            pt.pose.position.y = iy
            msg.poses.append(pt)
        
        self.pub_path.publish(msg)



def main():
    print("hogehoge")

    sx = 0.0
    sy = 0.0
    gx = 250.0 
    gy = 70.0

    graph_resolosion = 10
    vehiclesize = 50

    area_min_x, area_max_x = -500, 500
    area_min_y, area_max_y = -500, 500
    
    ox, oy = [], []
    for i in range(100):
        random.seed(10)
        ox.append(random.uniform(-500,500))
        oy.append(random.uniform(-500,500))

    dijkstra = Dijkstra(area_max_x, area_max_y, area_min_x, area_min_y, ox, oy, graph_resolosion, vehiclesize)
    rx, ry = dijkstra.planning(sx, sy, gx, gy)

    print(rx)
    print(ry)

if __name__ == '__main__':
    main()