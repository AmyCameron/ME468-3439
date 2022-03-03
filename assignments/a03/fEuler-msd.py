import numpy as np
import matplotlib.pyplot as plt


# given values
M = 1000
m = M/4
k_s = 5 * 10000
c_s = 4000
l_0_s = 0.4
g = 10

# range
t = 0
t_fin = 5
# ie-4 stepsize
dt = 0.0001 
# iteration
itr = int(t_fin/dt) 

# initial position
u_old = 0.4 
# initial velocity
v_old = 0 

# initializing lists for plotting
ut = []
u_ph =[]
tt = []
e_pos_4 = []
e_vel_4 = []

# analytical solution
def msd_ode(t):
    y = np.exp(-8*t) * ((7/20)*np.cos(np.sqrt(34)*t*2) + (56*np.sqrt(34)/1360)*np.sin(np.sqrt(34)*t*2)) + (1/20)
    return y

for i in range(itr):
    # counter
    t = dt * i

    # forward euler
    u = u_old + (v_old * dt)
    v = v_old + dt * (g - ((k_s/m)*(u_old)) - ((c_s/m) * v_old))
    
    # analytical solution
    ph = msd_ode(t)
    
    # updating lists
    ut.append(u)
    tt.append(t)
    u_ph.append(ph)
    e_pos_4.append(ph-u)
    v_num = (ph - u_ph[i-1]) / dt
    e_vel_4.append(np.abs(v_num-v))

    # updating i-1
    u_old = u
    v_old = v

print("max e_pos_4 is", max(e_pos_4))
print("max e_vel_4 is", max(e_vel_4))

# plotting Position of Sprung Mass
fig = plt.figure(figsize=(8, 6))
plt.title("Position of Sprung Mass",fontsize=12)
plt.plot(tt, ut,label='sample')
plt.plot(tt, u_ph,label='sample')
plt.xscale("linear")
plt.yscale("linear")
plt.ylabel("u(position)",fontsize=12)
plt.xlabel("t",fontsize=12)