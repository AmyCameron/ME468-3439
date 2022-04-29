%% bicycle model: ramp steering angle input 3deg/s; Vx=50km/h
clc;
clear;
% % vehicle model parameters
a=1.14;  % distance of c.g. from front axle (m)
b=1.4;  % distance of c.g. from rear axle  (m)
Cf=-44000*2; % front axle cornering stiffness (N/rad)
Cr=-47000*2; % rear axle cornering stiffness (N/rad)
Cxf=5000*2; % front axle longitudinal stiffness (N)
Cxr=5000*2; % rear axle longitudinal stiffness (N)
m=1720;  % the mass of the vehicle (kg)
Iz=2420; % yaw moment of inertia (kg.m^2)
Rr=0.285; % wheel radius
Jw=1*2;  % wheel roll inertia

%% steering input
% ramp steering input 3 (deg/s) and Vx=50/3.6 (m/s)
for i=1:4701
if i<1000
    delta3(i)=0;
else 
    delta3(i)=3*pi/180*(0.001*i-1);
end
end

%define initial state
x0=[0;50/3.6;0;0;0;0];
Vy=x0(1); % lateral velocity
Vx=x0(2); % longitudinal velocity
psi=x0(3); % yaw anlge
psi_dot=x0(4); % yaw rate 
Y=x0(5); % Y position in global coordinates
X=x0(6); % X position in global coordinates
wf=Vx/Rr; % front wheel rotation angular velocity
wr=Vx/Rr; % rear wheel rotation angular velocity

Tsim=4.7; % simulation time
T=0.001; % time step
ts=T;
i=1;
y(i,1:6)=x0';
%%
for t=0:ts:Tsim  
    delta_r=delta3(i);
    %longitudinal slips ratio
    sf=(Rr*wf-(Vx*cos(delta_r)+(Vy+a*psi_dot)*sin(delta_r)))/abs(Vx*cos(delta_r)+(Vy+a*psi_dot)*sin(delta_r));
    sr=(Rr*wr-Vx)/abs(Vx);
    %longitudinal tire force 
    Fxtf=Cxf*sf;
    Fxtr=Cxr*sr;
    % the wheel rotational equation, assuming no braking torque and accelerating torque
    dwf=-(1/Jw)*Fxtf*Rr;
    dwr=-(1/Jw)*Fxtr*Rr;
    wf=wf+T*dwf;
    wr=wr+T*dwr;

    % calculate the response according to vehicle model equations 
    Vy_dot=-Vx*psi_dot+(1/m)*(Cf*((Vy+a*psi_dot)/Vx-delta_r)+Cr*((Vy-b*psi_dot)/Vx));
    Vx_dot=Vy*psi_dot+(sf*Cxf+sr*Cxr)/m-delta_r*Cf*((Vy+a*psi_dot)/Vx-delta_r)/m;
    dpsi_dot=1/Iz*(a*Cf*((Vy+a*psi_dot)/Vx-delta_r)-b*Cr*((Vy-b*psi_dot)/Vx));
    Y_dot=Vx*sin(psi)+Vy*cos(psi);
    X_dot=Vx*cos(psi)-Vy*sin(psi);

    Vy=Vy+T*Vy_dot;
    Vx=Vx+T*Vx_dot;
    psi=psi+T*psi_dot;
    psi_dot=psi_dot+T*dpsi_dot;
    Y=Y+T*Y_dot;
    X=X+T*X_dot;     

    % output respones results
     y(i,1)=Vy;
     y(i,2)=Vx;
     y(i,3)=psi;
     y(i,4)=psi_dot;
     y(i,5)=Y;
     y(i,6)=X;
     lateral_acc(i)=Vy_dot+Vx*psi_dot; 
     i=i+1;
end

t=0:ts:Tsim;
figure(1)
plot(t,y(:,4),'r');grid
title('yaw rate vs time');
xlabel('time (s)');
ylabel('yaw rate (rad/s)');

figure(2)
plot(t,y(:,1),'r');grid
title('lateral velocity vs time');
xlabel('time (s)');
ylabel('lateral velocity (m/s)');

figure(3)
plot(t,y(:,3),'r');grid
title('yaw angle vs time');
xlabel('time (s)');
ylabel('yaw angle (rad)');

figure(4)
plot(t,delta3*180/pi,'r');grid
title('steering input angle vs time');
xlabel('time (s)');
ylabel('steer angle (deg)');

figure(5)
plot(t,lateral_acc,'r');grid
title('lateral acc vs time');
xlabel('time (s)');
ylabel('lateral acc (m/s^2)');

