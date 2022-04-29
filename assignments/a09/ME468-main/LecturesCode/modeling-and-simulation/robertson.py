"""
===================================
Robertson chemical kinetics problem
===================================

"""

import numpy as np
import matplotlib.pyplot as plt
import scipy.integrate as integrate

k1 = 0.04
k2 = 3e7
k3 = 1e4

def derivs(t,y):
    dy = np.zeros_like(y)
    dy[0] = -k1*y[0] + k3 * y[1]*y[2]
    dy[1] = k1*y[0] - k2*y[1]*y[1] - k3*y[1]*y[2]
    dy[2] = k2 * y[1]*y[1]
    return dy

tend = 1e6

# initial state
y0 = [1, 0, 0]

# integrate your ODE using scipy.integrate.
sol = integrate.solve_ivp(derivs, [0, tend], y0, method='LSODA', rtol=1e-4, atol=1e-8)

fig, ax = plt.subplots(1, 3)
fig.set_size_inches(12,5)
fig.tight_layout(pad=3.0)
for i in range(0,3):
  ax[i].plot(sol.t, sol.y[i])
  ax[i].set_xscale('log')
  ax[i].set_xlabel('time (s)')
  ax[i].set_title("y{0}".format(i))
  ax[i].grid()
  
ax[1].yaxis.set_major_formatter(plt.FormatStrFormatter('%.1e'))
  
plt.show()
