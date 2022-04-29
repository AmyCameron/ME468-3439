import numpy as np
import matplotlib.pyplot as plt

A = np.array([[1,0,0], [0,1,0],[0,0,1]])
B = np.array([[0.1,0],[0,0],[0,0.1]])
C = np.array([[1,0,0], [0,1,0]])
R = np.diag([np.random.normal(0,0.5),np.random.normal(0,0.5),np.random.normal(0,0.1)])
Q = np.diag([np.random.normal(0,0.8),np.random.normal(0,0.8)])

velocity = 2.2 #m/s
angular_velocity = 0.3 #rad/s
U = np.array([velocity, angular_velocity])

# gps data set

# initial state
x_old = 0 
y_old = 0
angle_old = 0
# stepsize
dt = 0.1 
# iteration
t = 0
t_fin = 100
T = int(t_fin/dt) 
# plotting array
xt = []
yt = []

# GPS data set Z
Z = []
for i in range(T):
    # forward euler
    angle = angle_old + dt * angular_velocity 
    x = x_old + dt * velocity * np.cos(angle) + np.random.normal(0,0.8,1)
    y = y_old + dt * velocity * np.sin(angle) + np.random.normal(0,0.8,1)    
    z = np.array([x,y])
    Z.append(z)
    x_old = x
    y_old = y
    angle_old = angle

mu = np.array([0,0,0])
Sigma = np.diag([0.1,0.1,0.1])
M = [mu]

# Kalman Filtering
for i in range(T):

    # predicted (step1,step2)
    mu_ = A @ mu + B @ U
    Sigma_ = R + A @ Sigma @ A.T

    # kalmanGain~set new ones
    # K: kalman gain (step3)
    S = C @ Sigma_ @ C.T + Q
    K = Sigma_ @ C.T @ np.linalg.inv(S)
    # new average (step4)
    zi = Z[i] - C @ mu_
    mu = mu_ + K @ zi
    # new average (step5)
    Sigma = (A - K @ C) @ Sigma_

    M.append(mu)
    xt.append(mu[0])
    yt.append(mu[1])

# plotting 
fig = plt.figure(figsize=(8, 6))
plt.title("Position",fontsize=12)
plt.plot(xt, yt,label='sample')
plt.xscale("linear")
plt.yscale("linear")
plt.ylabel("y",fontsize=12)
plt.xlabel("x",fontsize=12)
plt.show()
