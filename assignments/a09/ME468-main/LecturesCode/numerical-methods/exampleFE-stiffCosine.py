# Solves the IVP y_dot = -100*y + sin(t) and y(0)=0
# yExact is the analytical solution
# yFE is the solution obtained with Forward Euler
#
# Input: 
# - the integration step-size
# - end of the simulation time

import numpy as np
import matplotlib as plt
import matplotlib.pyplot as plt
import sys

# get input
h = float(sys.argv[1])
tend = float(sys.argv[2])
pngYesNo = int(sys.argv[3])

tm = np.arange(0,tend+h,h)
yExact = 1. / 10001 * (100 * np.sin(tm) - np.cos(tm) + np.exp(- 100 * tm))
yFE = np.zeros(tm.size)
for i in np.arange(1,tm.size):
    yFE[i] = yFE[i - 1] * (1 - 100 * h) + h * np.sin(tm[i - 1])

error = yExact - yFE

# plot as png or in window?
plt.plot(tm,error)
if pngYesNo == 1:
    plt.savefig("plot.png")
else:
    plt.show()