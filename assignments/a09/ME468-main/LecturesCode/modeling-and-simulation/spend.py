"""
===========================
The sngle pendulum problem
===========================

This animation illustrates the single pendulum problem.
Solution for both the linear (small angle approximation) and nonlinear DEs are superimposed.
Note: the small angle approximation
       sin(theta) ~ theta
is usually acceptable for theta < 5 degrees.
As you increase the initial angle, the differences between the two solutions will be more 
and more pronounced.
"""

from numpy import sin, cos
import numpy as np
import matplotlib.pyplot as plt
import scipy.integrate as integrate
import matplotlib.animation as animation

G = 9.8  # acceleration due to gravity, in m/s^2
L = 1.0  # length of pendulum in m
M = 1.0  # mass of pendulum in kg

initial_angle = 45  # pendulum angle at t=0 (in degrees!)

def derivs(state, t):

    dydx = np.zeros_like(state)

    # First two DEs: small angle approximation
    dydx[0] = state[1]
    dydx[1] = - (G / L) * state[0]

    # Last two DEs: nonlinear 2nd order
    dydx[2] = state[3]
    dydx[3] = -(G / L) * sin(state[2])
    
    return dydx

# create a time array from 0..tend sampled at dt second steps
period = 2 * np.pi * np.sqrt(L/G)
dt = 0.005
tend = 2 * period
t = np.arange(0.0, tend, dt)

# th is the initial angle (degrees)
# w is the initial angular velocity (degrees per second)
#th = 20
th = 45

w = 0.0

# Solve the two models (linear and nonlinear DEs) simultaneously
# First two DEs: small angle approximation
# Last two DEs: nonlinear 2nd order

# initial state
state = np.radians([th, w, th, w])

# integrate your ODE using scipy.integrate.
y = integrate.odeint(derivs, state, t)

x1 = L*sin(y[:, 0])
y1 = -L*cos(y[:, 0])

x2 = L*sin(y[:, 2])
y2 = -L*cos(y[:, 2])

fig = plt.figure(figsize=(6,6))
ax = fig.add_subplot(111, autoscale_on=False, xlim=(-1, 1), ylim=(-1.25, 0.25))
ax.axis('equal')
ax.grid()

line1, = ax.plot([], [], 'o-', lw=2, color='red')
line2, = ax.plot([], [], 'o-', lw=2, color='blue')
time_template = 'time = %.1fs'
time_text = ax.text(0.05, 0.9, '', transform=ax.transAxes)
h_text = ax.text(0.05, 0.825, 'harmonic', color='red', transform=ax.transAxes)
n_text = ax.text(0.05, 0.775, 'nonlinear', color='blue', transform=ax.transAxes)


def init():
    line1.set_data([], [])
    line2.set_data([], [])
    time_text.set_text('')
    return line1, line2, time_text


def animate(i):
    thisx1 = [0, x1[i]]
    thisy1 = [0, y1[i]]
    line1.set_data(thisx1, thisy1)
    thisx2 = [0, x2[i]]
    thisy2 = [0, y2[i]]
    line2.set_data(thisx2, thisy2)
    
    time_text.set_text(time_template % (i*dt))
    return line1, line2, time_text

ani = animation.FuncAnimation(fig, animate, np.arange(1, len(y)),
                              interval=30, blit=True, init_func=init)


f = "spend_{0}.mp4".format(th) 
writervideo = animation.FFMpegWriter(fps=30) 
ani.save(f, writer=writervideo, dpi=200)

plt.show()
