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