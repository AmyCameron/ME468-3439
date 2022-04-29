from matplotlib import bezier
import pychrono as ch
import numpy as np
import matplotlib.pyplot as plt
import argparse

#argument parser
parser = argparse.ArgumentParser(description='Path Comparison')
parser.add_argument('-p','--path', type=str, help="the file output from simulation which includes the position of the vehicle through time")
parser.add_argument('-o','--objects', type=str, help="file which describes where the objects are located")
args = parser.parse_args()

#intended path comes from bezier curve control points
objects = np.loadtxt(args.objects,delimiter=",")

#actual path comes from saved points from vehicle simulation
actual_path = np.loadtxt(args.path,delimiter=" ")[:,1:3]

#plot paths
plt.scatter(objects[:,0],objects[:,1],c='r')
plt.plot(actual_path[:,0],actual_path[:,1])
plt.legend(["Actual Path","Objects"])
plt.xlabel("X [m]")
plt.ylabel("Y [m]")
plt.title("Path taken through obstacle : perception")
plt.show()
