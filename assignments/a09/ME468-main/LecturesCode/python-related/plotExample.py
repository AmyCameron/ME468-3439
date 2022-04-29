import os
import numpy as np
import matplotlib.pyplot as pyplot

x = np.arange(0, 8, 0.2)

y = x**2

pyplot.plot(x, y)

pyplot.savefig("plot.png")
