import numpy as np
import matplotlib.pyplot as plt


velocity = 2.2 #m/s
angular_velocity = 0.3 #rad/s

# range
t = 0
t_fin = 100
# stepsize
dt = 0.1 
# iteration
itr = int(t_fin/dt) 

# initial state
x_old = 0 
y_old = 0
angle_old = 0

# initializing lists for plotting
xt = []
yt = []

for i in range(itr):

    # forward euler
    x = x_old + dt * velocity * np.cos(angle_old)
    y = y_old + dt * velocity * np.sin(angle_old)
    angle = angle_old + dt * angular_velocity
    
    # updating lists
    xt.append(x)
    yt.append(y)

    # updating i-1
    x_old = x
    y_old = y
    angle_old = angle

print(xt)
# plotting Position of Sprung Mass
fig = plt.figure(figsize=(8, 6))
plt.title("Position",fontsize=12)
plt.plot(xt, yt,label='sample')
plt.xscale("linear")
plt.yscale("linear")
plt.ylabel("y",fontsize=12)
plt.xlabel("x",fontsize=12)
plt.show()

