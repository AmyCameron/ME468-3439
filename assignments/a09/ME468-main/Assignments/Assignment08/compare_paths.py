from matplotlib import bezier
import pychrono as ch
import numpy as np
import matplotlib.pyplot as plt
import argparse

#argument parser
parser = argparse.ArgumentParser(description='Path Comparison')
parser.add_argument('-t','--target', type=str, help="target path file")
parser.add_argument('-a','--actual', type=str, help="actual path file")
args = parser.parse_args()

#intended path comes from bezier curve control points
intended_path = np.loadtxt(args.target,delimiter=",")

print("intended_path:",intended_path.shape)
ch_intended_path = []
for pt in intended_path:
    ch_intended_path.append(ch.ChVectorD(pt[0],pt[1],0))

#actual path comes from saved points from vehicle simulation
actual_path = np.loadtxt(args.actual,delimiter=" ")[:,1:3]

#generate bezier curve
bezier_path = ch.ChBezierCurve(ch_intended_path)

#eval bezier curve to plot it
plt_path = []
n = 100
for  i in range(1000):
    pt = bezier_path.eval(1/n * i)
    plt_path.append([pt.x,pt.y])
plt_path = np.asarray(plt_path)

#eval bezier curve at vehicle points to get errors
errors = []
tracker = ch.ChBezierCurveTracker(path=bezier_path)
for pt in actual_path:
    target = ch.ChVectorD(0,0,0)
    sentinel = ch.ChVectorD(pt[0],pt[1],0)
    tracker.calcClosestPoint(sentinel,target)
    errors.append((target-sentinel).Length())
errors = np.asarray(errors)

#print error
print("Max error:",np.max(errors))

#plot paths
plt.plot(plt_path[:,0],plt_path[:,1])
plt.scatter(intended_path[:,0],intended_path[:,1])
plt.plot(actual_path[:,0],actual_path[:,1])
plt.legend(["Intended Path","Control Points","Actual Path"])
plt.xlabel("X [m]")
plt.ylabel("Y [m]")
plt.title("Path Comparison")

plt.figure()
plt.plot(errors)
plt.title("Path Error")
plt.xlabel("Sample")
plt.ylabel("Distance to Target Curve [m]")
plt.show()
