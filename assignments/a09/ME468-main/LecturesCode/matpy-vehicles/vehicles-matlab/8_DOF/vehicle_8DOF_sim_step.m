%% 8 DOF vehicle model: step steering anlge input 0.0087(rad) and Vx=33.73(m/s)
clc;
clear;
%% vehicle parameters definition
m=1400; % Sprung mass (kg)
Jx=900; % Sprung mass roll inertia (kg.m^2)
Jy=2000; % Sprung mass pitch inertia (kg.m^2)
Jz=2420; % Sprung mass yaw inertia (kg.m^2)
a=1.14; % Distance of sprung mass c.g. from front axle (m)
b=1.4; % Distance of sprung mass c.g. from rear axle (m)
Jxz=90; % Sprung mass XZ product of inertia
Jw=1;   % tire/wheel roll inertia kg.m^2
g=9.8;   % acceleration of gravity 
h=0.75;  % Sprung mass c.g. height (m)
cf=1.5;  % front track width (m)
cr=1.5;  % rear track width (m)
muf=80;    %front unsprung mass (kg)
mur=80;    %rear unsprung mass (kg)
ktf=200000;  %front tire stiffness (N/m)
ktr=200000;  %rear tire stiffness (N/m)
krof=29000;  %front roll stiffness (Nm/rad)
kror=29000;  %rear roll stiffness (Nm/rad)
brof=3000;   %front roll damping coefficient (Nm.s/rad)
bror=3000;   %rear roll damping coefficient (Nm.s/rad)
Cf=-44000; %fModellingront tire cornering stiffness (N/rad)
Cr=-47000;  %rear tire cornering stiffness (N/rad)
Cxf=5000; %front tire longitudinal stiffness (N)
Cxr=5000; %rear tire longitudinal stiffness (N)
r0=0.285; %nominal tire radius (m)
hrcf=0.65; %front roll center distance below sprung mass c.g.
hrcr=0.6;  %rear roll center distance below sprung mass c.g.
hrc=(hrcf*b+hrcr*a)/(a+b); % the vertical distance from the sprung mass C.M. to the vehicle roll center
mt=m+2*muf+2*mur; % vehicle total mass

%% steering angle input
% step steer angle input 0.0087(rad) and Vx=33.73
for i=1:11000
if i<1000
    delta3(i)=0;
elseif i>=1000 && i<1050
    delta3(i)=0.5*pi/180*sin(1/0.2*2*pi*0.001*(i-1000));
else 
    delta3(i)=0.5*pi/180;
end
end

%% vehicle initial state
u=33.73; % the longitudinal velocity 
v=0;     % the lateral velocity 
u_dot=0; % the longitudinal acceleration
v_dot=0; % the lateral acceleration
phi=0;   % roll angle
psi=0;   % yaw angle
dphi=0;  % roll angular velocity
dpsi=0;  % yaw angular velocity
wx=0;  % roll angular velocity
wy=0;  % pitch angular velocity
wz=0;  % yaw angular velocity
wx_dot=0; % roll angular acceleration
wz_dot=0; % yaw angular acceleration
wlf=u/r0; % angular velocity of left front wheel rotation 
wrf=u/r0; % angular velocity of right front wheel rotation
wlr=u/r0; % angular velocity of left rear wheel rotation
wrr=u/r0; % angular velocity of right rear wheel rotation

% the initial tire compression xtif
xtirf=((m*g*b)/(2*(a+b))+muf*g)/ktf;
xtilf=((m*g*b)/(2*(a+b))+muf*g)/ktf;
xtilr=((m*g*a)/(2*(a+b))+mur*g)/ktr;
xtirr=((m*g*a)/(2*(a+b))+mur*g)/ktr;
xtlf=xtilf;
xtrf=xtirf;
xtlr=xtilr;
xtrr=xtirr;

% for output record
i=1;
long_vel(i)=u;
long_acc(i)=u_dot;
roll_angle(i)=phi;
lat_acc(i)=v_dot;
lat_vel(i)=v;
psi_angle(i)=psi;
yaw_rate(i)=wz;

%%
Tsim=11000;
for tt=1:Tsim   
ts=(tt-1)*0.001:0.0001:tt*0.001;
delta=delta3(i);
% the instantaneous tire radius 
Rrf=r0-xtrf;
Rlf=r0-xtlf;
Rlr=r0-xtlr;
Rrr=r0-xtrr;
% position of front and rear unsprung mass
huf=Rrf; 
hur=Rrr;
% %the longitudinal and lateral velocities at the tire contact patch in coordinate frame 2
ugrf=u+(wz*cf)/2;
vgrf=v+wz*a;
uglf=u-(wz*cf)/2;
vglf=v+wz*a;
uglr=u-(wz*cr)/2;
vglr=v-wz*b;
ugrr=u+(wz*cr)/2;
vgrr=v-wz*b;
% tire slip angle of each wheel
delta_rf=atan(vgrf/ugrf)-delta;
delta_lf=atan(vglf/uglf)-delta;
delta_lr=atan(vglr/uglr);
delta_rr=atan(vgrr/ugrr);
%linear tire lateral force
Fytrf=Cf*delta_rf;
Fytlf=Cf*delta_lf;
Fytlr=Cr*delta_lr;
Fytrr=Cr*delta_rr;
% longitudinal slips
s_rf=(Rrf*wrf-(ugrf*cos(delta)+vgrf*sin(delta)))/abs(ugrf*cos(delta)+vgrf*sin(delta));
s_lf=(Rlf*wlf-(uglf*cos(delta)+vglf*sin(delta)))/abs(uglf*cos(delta)+vglf*sin(delta));
s_lr=(Rlr*wlr-uglr)/abs(uglr);
s_rr=(Rrr*wrr-ugrr)/abs(ugrr);
% linear tire longitudinal force 
Fxtrf=Cxf*s_rf;
Fxtlf=Cxf*s_lf;
Fxtlr=Cxr*s_lr;
Fxtrr=Cxr*s_rr;
% the forces Fxgij obtained by resolving the longitudinal and cornering forces at the tire contact patch 
Fxglf=Fxtlf*cos(delta)-Fytlf*sin(delta);
Fxgrf=Fxtrf*cos(delta)-Fytrf*sin(delta);
Fxglr=Fxtlr;
Fxgrr=Fxtrr;
Fyglf=Fxtlf*sin(delta)+Fytlf*cos(delta);
Fygrf=Fxtrf*sin(delta)+Fytrf*cos(delta);
Fyglr=Fytlr;
Fygrr=Fytrr;

% The normal forces at four tires are determined as
Z1=(m*g*b)/(2*(a+b))+(muf*g)/2;
Z2=((muf*huf)/cf+m*b*(h-hrcf)/(cf*(a+b)))*(v_dot+wz*u);
Z3=(krof*phi+brof*dphi)/cf;
Z4=((m*h+muf*huf+mur*hur)*(u_dot-wz*v))/(2*(a+b));
Fzglf=Z1-Z2-Z3-Z4;
Fzgrf=Z1+Z2+Z3-Z4;
Z5=(m*g*a)/(2*(a+b))+(mur*g)/2;
Z6=((mur*hur)/cr+m*a*(h-hrcr)/(cr*(a+b)))*(v_dot+wz*u);
Z7=(kror*phi+bror*dphi)/cr;
Z8=((m*h+muf*huf+mur*hur)*(u_dot-wz*v))/(2*(a+b));
Fzglr=Z5-Z6-Z7+Z8;
Fzgrr=Z5+Z6+Z7+Z8;

xtlf=Fzglf/ktf;
xtrf=Fzgrf/ktf;
xtlr=Fzglr/ktr;
xtrr=Fzgrr/ktr;

vglf1(i)=vglf;
uglf1(i)=uglf;
vglr1(i)=vglr;
uglr1(i)=uglr;
vgrf1(i)=vgrf;
ugrf1(i)=ugrf;
vgrr1(i)=vgrr;
ugrr1(i)=ugrr;
delta_lf1(i)=delta_lf;
delta_rf1(i)=delta_rf;
delta_lr1(i)=delta_lr;
delta_rr1(i)=delta_rr;
Fzlf(i)=Fzglf;
Fzrf(i)=Fzgrf;
Fzlr(i)=Fzglr;
Fzrr(i)=Fzgrr;

% chassis equations
dpsi=wz;
dphi=wx;
E1=-mt*wz*u+(Fyglf+Fygrf+Fyglr+Fygrr);
E2=(Fyglf+Fygrf)*a-(Fyglr+Fygrr)*b+(Fxgrf-Fxglf)*cf/2+(Fxgrr-Fxglr)*cr/2+(mur*b-muf*a)*wz*u;
E3=m*g*hrc*phi-(krof+kror)*phi-(brof+bror)*dphi+hrc*m*wz*u;
A1=mur*b-muf*a;
A2=Jx+m*hrc^2;
A3=hrc*m;
u_dot=wz*v+(1/mt)*((Fxglf+Fxgrf+Fxglr+Fxgrr)+(muf*a-mur*b)*(wz)^2-2*hrc*m*wz*wx);
v_dot=(E1*Jxz^2-A1*A2*E2+A1*E3*Jxz+A3*E2*Jxz-A2*E1*Jz-A3*E3*Jz)/(A2*A1^2-2*A1*A3*Jxz+Jz*A3^2+mt*Jxz^2-A2*Jz*mt);
wx_dot=(A1^2*E3-A1*A3*E2+A1*E1*Jxz-A3*E1*Jz+E2*Jxz*mt-E3*Jz*mt)/(A2*A1^2-2*A1*A3*Jxz+Jz*A3^2+mt*Jxz^2-A2*Jz*mt);
wz_dot=(A3^2*E2-A1*A2*E1-A1*A3*E3+A3*E1*Jxz-A2*E2*mt+E3*Jxz*mt)/(A2*A1^2-2*A1*A3*Jxz+Jz*A3^2+mt*Jxz^2-A2*Jz*mt);
[t,psi]= ode45(@(t,psi) dpsi,ts,psi);
[t,phi]= ode45(@(t,phi) dphi,ts,phi);
[t,u]= ode45(@(t,u) u_dot,ts,u);
[t,v]= ode45(@(t,v) v_dot,ts,v);
[t,wx]= ode45(@(t,wx) wx_dot,ts,wx);
[t,wz]= ode45(@(t,wz) wz_dot,ts,wz);
psi=psi(11);
phi=phi(11);
u=u(11);
v=v(11);
wx=wx(11);
wz=wz(11);
dpsi=wz;
dphi=wx;

%% the wheel rotational modeling
dwlf=-(1/Jw)*Fxtlf*Rlf;
dwrf=-(1/Jw)*Fxtrf*Rrf;
dwlr=-(1/Jw)*Fxtlr*Rlr;
dwrr=-(1/Jw)*Fxtrr*Rrr;
[t,wlf]=ode45(@(t,wlf) dwlf,ts,wlf);
[t,wrf]=ode45(@(t,wrf) dwrf,ts,wrf);
[t,wlr]=ode45(@(t,wlr) dwlr,ts,wlr);
[t,wrr]=ode45(@(t,wrr) dwrr,ts,wrr);
wlf=wlf(11);
wrf=wrf(11);
wlr=wlr(11);
wrr=wrr(11);

% % for output plot
i=tt+1;
long_vel(i)=u;
long_acc(i)=u_dot;
roll_angle(i)=phi;
lat_acc(i)=v_dot;
lat_vel(i)=v;
psi_angle(i)=psi;
yaw_rate(i)=wz;
% vehicle lateral acceleration
ay(i)=u*wz+v_dot;
% longitudinal slip ratio
s_re(i,1)=s_lf;
s_re(i,2)=s_rf;
s_re(i,3)=s_lr;
s_re(i,4)=s_rr;
end
t=0:Tsim;
figure(1)
plot(t*0.001,roll_angle,'r');grid
title('roll angle vs time');
xlabel('time (s)');
ylabel('roll angle(rad)');
figure(2)
plot(t*0.001,ay,'r');grid
title('lateral acceleration vs time');
xlabel('time (s)');
ylabel('lateral acceleration (m/s^2)');
figure(3)
plot(t*0.001,yaw_rate,'r');grid
title('yaw rate vs time');
xlabel('time (s)');
ylabel('yaw rate (rad/s)');
figure(4)
plot(t*0.001,lat_vel,'r');grid
title('lateral velocity vs time');
xlabel('time (s)');
ylabel('lateral velocity (m/s)');
figure(5)
plot(t*0.001,psi_angle,'r');grid
title('yaw angle vs time');
xlabel('time (s)');
ylabel('yaw angle (rad)');
t=0:Tsim-1;
figure(6)
plot(t*0.001,delta3,'r');grid
title('steering input angle vs time');
xlabel('time (s)');
ylabel('steer angle (rad)');

% subplot(2,2,1);
% plot(t*0.001,Fzlf1,'r');grid
% subplot(2,2,2);
% plot(t*0.001,Fzrf1,'r');grid
% subplot(2,2,3)
% plot(t*0.001,Fzlr1,'r');grid
% subplot(2,2,4)
% plot(t*0.001,Fzrr1,'r');grid
