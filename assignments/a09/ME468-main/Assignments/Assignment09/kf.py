import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D


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

Sigma = np.array([[0.1,0.0,0.0],[0.0,0.1,0.0],[0.0,0.0,0.1]])
R = np.array([[0.5,0.0,0.0],[0.0,0.5,0.0],[0.0,0.0,0.1]])
Q = np.array([[0.8,0.0],[0.0,0.8]])

A = np.array([[1,0,0],[0,1,0],[0,0,1]])
B = np.array([[dt,0],[0,0],[0,dt]])
C = np.array([[1,0,0],[0,1,0]])

# initializing lists for plotting
xt = []
yt = []
zt = []
xti = []
yti = []
zti = []
ct = 0

# uvec = np.array([[velocity], [angular_velocity]]) # (1, 2)
uvec = np.array([velocity, angular_velocity]) # (1, 2)

for i in range(itr):

  # forward euler
    angle = angle_old + dt * angular_velocity
    x = x_old + dt * velocity * np.cos(angle)
    y = y_old + dt * velocity * np.sin(angle)
    
    xvec = np.array([[x], [y], [angle]]) # should be 1 x 3
    # xvec = np.array([[x], [y], [0]]) # should be 1 x 3
    # xvec = np.array([x, y, 0]) # should be 1 x 3
    xti.append(xvec[0])
    yti.append(xvec[1])
    zti.append(xvec[2])

    # updating i-1
    x_old = x
    y_old = y
    angle_old = angle
    ct = ct + dt

    # prediction
    # xvec = np.matmul(A, xvec) + np.matmul(B, uvec) + np.random.multivariate_normal(np.array([0,0,0]), R).reshape(3,1) # (3,3)
    # xvec = np.matmul(A, xvec) + np.matmul(B, uvec) # + np.random.multivariate_normal(np.array([0,0,0]), R).reshape(3,1) # (3,3)
    zvec = C.dot(xvec) + np.random.multivariate_normal(np.array([0,0]), Q).reshape(2,1)

    Sigma = np.diag(np.diag(A.dot(Sigma).dot(A.T) + R)) # np.random.multivariate_normal(np.array([0,0,0]), R) # R (3,3)
    # print(xvec.shape)

    # update
    K = Sigma.dot(C.T).dot(np.linalg.inv(C.dot(Sigma).dot(C.T) + Q))
    
    xvec = xvec + K.dot(zvec - np.matmul(C, xvec))
    Sigma = (np.array([1])-np.matmul(K, C)).dot(Sigma)

    # # updating lists
    xt.append(xvec[0])
    yt.append(xvec[1])
    zt.append(xvec[2])

# plotting Position
fig = plt.figure(figsize=(8, 6))
ax = fig.add_subplot(111, projection='3d')
ax.scatter3D(xt, yt, zt)
ax.scatter3D(xti, yti, zti)
ax.set_xlabel("x", size = 14)
ax.set_ylabel("y", size = 14)
ax.set_zlabel("orientation")
ax.set_title("Scatter Plot")
#plt.plot(xti, yti,label='sample')
#plt.plot(xt, yt,label='sample')
#plt.xscale("linear")
#plt.yscale("linear")
#plt.ylabel("y",fontsize=12)
#plt.xlabel("x",fontsize=12)
plt.show()
