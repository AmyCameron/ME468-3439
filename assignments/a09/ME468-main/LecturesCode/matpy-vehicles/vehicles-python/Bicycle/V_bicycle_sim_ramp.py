import numpy as np
import matplotlib.pyplot as mpl

## bicycle model: ramp steering angle input 3deg/s Vx=50km/h
# # vehicle model parameters
a=1.14  # distance of c.g. from front axle (m)
b=1.4  # distance of c.g. from rear axle  (m)
Cf=-44000*2 # front axle cornering stiffness (N/rad)
Cr=-47000*2 # rear axle cornering stiffness (N/rad)
Cxf=5000*2 # front axle longitudinal stiffness (N)
Cxr=5000*2 # rear axle longitudinal stiffness (N)
m=1720  # the mass of the vehicle (kg)
Iz=2420 # yaw moment of inertia (kg.m^2)
Rr=0.285 # wheel radius
Jw=1*2  # wheel roll inertia

## steering input
# ramp steering input 3 (deg/s) and Vx=50/3.6 (m/s)
# delta3 = [None] * 4700
delta3 = np.zeros(4700)
for i in range(0, 4700):
    if i < 1000:
        delta3[i]=0
    else:
        delta3[i]=3*np.pi/180*(0.001*i-1)

#define initial state
x0=[[0],[50/3.6],[0],[0],[0],[0]]
Vy=x0[0][0] # lateral velocity
Vx=x0[1][0] # longitudinal velocity
psi=x0[2][0] # yaw anlge
psi_dot=x0[3][0] # yaw rate 
Y=x0[4][0] # Y position in global coordinates
X=x0[5][0] # X position in global coordinates
wf=Vx/Rr # front wheel rotation angular velocity
wr=Vx/Rr # rear wheel rotation angular velocity

Tsim=4.7 # simulation time
T=0.001 # time step
ts=T
i=0
# y(i,1:6)=x0'
tt = np.arange(0, Tsim, ts)
y = np.zeros( (len(tt),len(x0)) )
# ytemp = np.transpose(x0)
y[0][0] = x0[0][0]
y[0][1] = x0[1][0]
y[0][2] = x0[2][0]
y[0][3] = x0[3][0]
y[0][4] = x0[4][0]
y[0][5] = x0[5][0]
# print("y: ", y)
# print("x0: ", x0)

##
# for t=0:ts:Tsim
# for t in np.linspace(0, Tsim, (Tsim // ts)):
lateral_acc = []
for t in tt: # np.arange(0, Tsim, ts):
    delta_r=delta3[i]
    #longitudinal slips ratio
    sf=(Rr*wf-(Vx*np.cos(delta_r)+(Vy+a*psi_dot)*np.sin(delta_r)))/np.abs(Vx*np.cos(delta_r)+(Vy+a*psi_dot)*np.sin(delta_r))
    sr=(Rr*wr-Vx)/np.abs(Vx)
    #longitudinal tire force 
    Fxtf=Cxf*sf
    Fxtr=Cxr*sr
    # the wheel rotational equation, assuming no braking torque and accelerating torque
    dwf=-(1/Jw)*Fxtf*Rr
    dwr=-(1/Jw)*Fxtr*Rr
    wf=wf+T*dwf
    wr=wr+T*dwr

    # calculate the response according to vehicle model equations 
    Vy_dot=-Vx*psi_dot+(1/m)*(Cf*((Vy+a*psi_dot)/Vx-delta_r)+Cr*((Vy-b*psi_dot)/Vx))
    Vx_dot=Vy*psi_dot+(sf*Cxf+sr*Cxr)/m-delta_r*Cf*((Vy+a*psi_dot)/Vx-delta_r)/m
    dpsi_dot=1/Iz*(a*Cf*((Vy+a*psi_dot)/Vx-delta_r)-b*Cr*((Vy-b*psi_dot)/Vx))
    Y_dot=Vx*np.sin(psi)+Vy*np.cos(psi)
    X_dot=Vx*np.cos(psi)-Vy*np.sin(psi)

    Vy=Vy+T*Vy_dot
    Vx=Vx+T*Vx_dot
    psi=psi+T*psi_dot
    psi_dot=psi_dot+T*dpsi_dot
    Y=Y+T*Y_dot
    X=X+T*X_dot

    # output respones results
    #print(y)
    y[i][0]=Vy
    y[i][1]=Vx
    y[i][2]=psi
    y[i][3]=psi_dot
    y[i][4]=Y
    y[i][5]=X
    lateral_acc.append(Vy_dot+Vx*psi_dot)
    i=i+1


mpl.figure(1)
mpl.plot(tt,y[:,3],'r')
mpl.title('yaw rate vs time')
mpl.xlabel('time (s)')
mpl.ylabel('yaw rate (rad/s)')
mpl.show()

mpl.figure(2)
mpl.plot(tt,y[:,0],'r')
mpl.title('lateral velocity vs time')
mpl.xlabel('time (s)')
mpl.ylabel('lateral velocity (m/s)')
mpl.show()

mpl.figure(3)
mpl.plot(tt,y[:,2],'r')
mpl.title('yaw angle vs time')
mpl.xlabel('time (s)')
mpl.ylabel('yaw angle (rad)')
mpl.show()

mpl.figure(4)
mpl.plot(tt,delta3*180/np.pi,'r')
mpl.title('steering input angle vs time')
mpl.xlabel('time (s)')
mpl.ylabel('steer angle (deg)')
mpl.show()

mpl.figure(5)
mpl.plot(tt,lateral_acc,'r')
mpl.title('lateral acc vs time')
mpl.xlabel('time (s)')
mpl.ylabel('lateral acc (m/s^2)')
mpl.show()
