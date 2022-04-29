# Solves the IVP y_dot = -100*y and y(0)=1
# yExact is the analytical solution
# yFE is the solution obtained with Forward Euler
#
# Input: 
# - the integration step-size
# - end of the simulation time
# - plot as png or in a window?

import numpy as np
import matplotlib as plt
import matplotlib.pyplot as plt
import sys

# get input
h = float(sys.argv[1])
tend = float(sys.argv[2])
pngYesNo = int(sys.argv[3])

tm = np.arange(0,tend+h,h)
yExact = np.exp(- 100 * tm) 

# use forward Euler
yFE = np.zeros(tm.size)+1
for i in np.arange(1,tm.size):
    yFE[i] = yFE[i - 1] * (1 - 100 * h)

# print out error
error = yExact - yFE
print(error[0:6])

# plot as png or in a window?
plt.plot(tm,error)
if pngYesNo == 1:
    plt.savefig("plot.png")
else:
    plt.show()
