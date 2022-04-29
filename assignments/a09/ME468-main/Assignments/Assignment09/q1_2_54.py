
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
X_old = np.array([x_old, y_old, angle_old])

# erro
dx = 0
dy = 0

# initializing lists for plotting
xt = []
yt = []

# nd
A = np.array([[1,0,0], [0,1,0],[0,0,1]])
B = np.array([[0.1,0],[0,0],[0,0.1]])
C = np.array([[1,0,0], [0,1,0]])
R = np.diag([0.5,0.5,0.1])
Q = np.diag([0.8,0.8])
#R = np.diag([np.random.normal(0,0.5),np.random.normal(0,0.5),np.random.normal(0,0.1)])
#Q = np.diag([np.random.normal(0,0.8),np.random.normal(0,0.8)])
I = np.eye(3)
p_old = np.diag([0.1,0.1,0.1])
U = np.array([velocity,angular_velocity])






for i in range(itr):

    # motion model
    #x_t = x_old + dt * velocity + np.random.normal(0,0.5)
    #y_t = y_old + np.random.normal(0,0.5)
    #angle = angle_old + dt * angular_velocity + np.random.normal(0,0.1)
    # x_t = x_old + dt * velocity * np.cos(angle_old) + np.random.normal(0,0.5)
    # y_t = y_old + dt * velocity * np.sin(angle_old) + np.random.normal(0,0.5)
    #X_t = np.array([x_t,y_t,angle])
    X_t = A @ X_old + B @ U + np.random.multivariate_normal([0, 0, 0], R, 1).T

    # measurement model
    #z_x_t = x_t + np.random.normal(0,0.8)
    #z_y_t = y_t + np.random.normal(0,0.8)
    #z_t = np.array([z_x_t,z_y_t])
    z_t = C @ X_t + np.random.multivariate_normal([0, 0], Q, 1).T

    # step2
    p_pre = A @ p_old @ A.T + R 

    # step3
    kGain = p_pre @ np.transpose(C) @ np.linalg.inv(C @ p_pre @ np.transpose(C) + Q)

    # step4
    fx = X_t + kGain @ (z_t - C @ X_t)
 
    # step5
    fp = (I - kGain @ C) @ p_pre

    # updating lists
    xt.append(fx[0])
    yt.append(fx[1])

    # updating i-1
    x_old = fx[0]
    y_old = fx[1]
    p_old = fp
    X_old = np.array([x_old, y_old, angle_old])
    #angle_old = angle


# plotting Position
fig = plt.figure(figsize=(8, 6))
plt.title("Position",fontsize=12)
plt.plot(xt, yt,label='sample')
plt.xscale("linear")
plt.yscale("linear")
plt.ylabel("y",fontsize=12)
plt.xlabel("x",fontsize=12)
plt.show()

