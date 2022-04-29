import itertools
import numpy as np
import scipy.optimize as sopt
import matplotlib.pyplot as pt

# cost function
def f(x):
    return 0.5*x[0]**2 + 2.5*x[1]**2

# gradient of cost function
def df(x):
    return np.array([x[0], 5*x[1]])

xmesh, ymesh = np.mgrid[-2:2:50j,-2:2:50j]
fmesh = f(np.array([xmesh, ymesh]))

# plot a 3d version of the cost function
ax = pt.axes(projection='3d')
ax.plot_surface(xmesh, ymesh, fmesh)
pt.savefig("3d.png")

# solve optimization problem
guesses = [np.array([2, 2./5])]
x = guesses[-1]
s = -df(x)
def f1d(alpha):
    return f(x + alpha*s)

iteration = 0
while np.linalg.norm(s)>1e-7:
    iteration = iteration + 1
    alpha_opt = sopt.golden(f1d)
    next_guess = x + alpha_opt * s
    print("Iteration: ", iteration, next_guess)
    guesses.append(next_guess)
    x = guesses[-1]
    s = -df(x)

# clear plot; show the iterations in 2d contour
pt.clf()
it_array = np.array(guesses)
pt.contour(xmesh, ymesh, fmesh, 50)
pt.plot(it_array.T[0], it_array.T[1], "x-")
pt.savefig("contour.png")