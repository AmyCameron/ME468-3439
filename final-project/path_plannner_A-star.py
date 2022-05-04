import rclpy
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile
from custom_msgs.msg import VehicleState, ObjectList, Object
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path

from ament_index_python.packages import get_package_share_directory

import os
import numpy as np
import matplotlib
import matplotlib.pyplot as plt
import matplotlib.patches as patches

from scipy.integrate import odeint

class PathPlanning_Astar_Node(Node):
    def __init__(self):
        super().__init__('path_planning_Astar_node')
        
        self.start_x = 0
        self.start_y = 0
        self.goal_x = 200
        self.goal_y = 0

    class graphNode:
        def __init__(self, x, y, cost, parent):
            self.x = x
            self.y = y
            self.cost = cost
            self.parent = parent
    
    def planing(self, start_x, start_y, goal_x, goal_y):
        start_Node = self.graphNode(start_x, start_y, 0.0, None)
        goal_Node = self.graphNode(goal_x, goal_y, 0.0, None)
        



        