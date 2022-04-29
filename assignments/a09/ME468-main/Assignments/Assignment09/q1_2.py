from unittest import result
import numpy as np
import matplotlib.pyplot as plt


velocity = 2.2 #m/s
angular_velocity = 0.3 #rad/s
U = np.array([velocity, angular_velocity])

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

# erro
dx = 0
dy = 0

# initializing lists for plotting
xt = []
yt = []

# nd
mean = [0, 0]
cov = [[0.8,0], [0, 0.8]]
A = np.array([[1,0,0], [0,1,0],[0,0,1]])
B = np.array([[0.1,0],[0,0],[0,0.1]])
C = np.array([[1,0,0], [0,1,0]])
R = np.diag([0.5,0.5,0.1])
Q = np.diag([0.8,0.8])
I = np.eye(3)
p_old = np.diag([0.1,0.1,0.1])


for i in range(itr):

    # step1: predicted average?
    angle = angle_old + dt * angular_velocity
    x = x_old + dt * velocity * np.sin(angle) + np.random.normal(0,0.8)
    y = y_old + dt * velocity * np.cos(angle) + np.random.normal(0,0.8)
    X_pre = np.array([x,y,angle])
    X_motion = np.dot(A,X_pre) + np.dot(B,U) + R
    X = np.array([x,y,angle])
    Z = np.array([x,y])

    # step2: predicted co-variance matrix
    p = A * p_old * np.transpose(A) + R 

    # step3: set kalman gain
    kGain_1 = np.dot(p, np.transpose(C))
    kGain_2 = np.dot(C, p)
    kGain_3 = np.dot(kGain_2, np.transpose(C))
    kGain_4 = kGain_3 + np.random.normal(0,0.8,(2,2))
    kGain_5 = np.linalg.inv(kGain_4)
    kGain = np.dot(kGain_1,kGain_5)

    # step4: set new average?
    fx_error = Z - np.dot(C,X)
    fx = X + np.dot(kGain, fx_error)

    # step5: set new co-variance matrix
    fp_1 = np.dot(kGain, C)
    fp_2 = I - fp_1
    fp = np.dot(fp_2 ,p)


    # updating lists
    xt.append(fx[0])
    yt.append(fx[1])

    print(fx)
    # updating i-1
    x_old, y_old = fx[0],fx[1]
    p_old = fp
    angle_old = angle
    #ct = ct + dt



# plotting Position
fig = plt.figure(figsize=(8, 6))
plt.title("Position",fontsize=12)
plt.plot(xt, yt,label='sample')
plt.xscale("linear")
plt.yscale("linear")
plt.ylabel("y",fontsize=12)
plt.xlabel("x",fontsize=12)
plt.show()

