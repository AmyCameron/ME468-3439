import pychrono as chrono
import numpy as np
import matplotlib.pyplot as plt

#------------------------------------------------------------------------------

def plot_function(x, f, title):
    y = np.empty(x.size)
    yd = np.empty(x.size)
    for i in np.arange(0,x.size):
        y[i] = f.Get_y(x[i])
        yd[i] = f.Get_y_dx(x[i])    
    
    fig, ax = plt.subplots(1, 2)
    fig.set_size_inches(12,5)
    fig.tight_layout(pad=3.0)
    fig.suptitle(title)

    ax[0].plot(x, y)
    ax[0].set_xlabel('x')
    ax[0].set_ylabel('y')
    ax[0].grid()

    ax[1].plot(x, yd)
    ax[1].set_xlabel('x')
    ax[1].set_ylabel('yd')
    ax[1].grid()

#------------------------------------------------------------------------------

# Define a custom function
class MyFunction(chrono.ChFunction):
    def Get_y(self, x):
        y = np.cos(np.pi * x)
        return y

#------------------------------------------------------------------------------

# Evaluate functions at 100 points in [0,6] 
x = np.linspace(0.0, 6.0, 100)

# Ramp function
# -------------

f = chrono.ChFunction_Ramp()
f.Set_ang(0.1)  # angular coefficient
f.Set_y0(0.1)   # y value at x = 0

plot_function(x, f, 'Ramp function')

# Sine function
# -------------

f = chrono.ChFunction_Sine()
f.Set_amp(2)     # amplitude
f.Set_freq(1.5)  # frequency

plot_function(x, f, 'Sine function')

# Custom function
# ---------------

# Create a custom function for y = f(pi*x)
f = MyFunction()

plot_function(x, f, 'Custom function')

# Function sequence
# -----------------

f_seq = chrono.ChFunction_Sequence()

f_const_acc1 = chrono.ChFunction_ConstAcc()
f_const_acc1.Set_end(0.5)  # ramp length
f_const_acc1.Set_h(0.3)    # ramp height
f_seq.InsertFunct(f_const_acc1, 0.5, 1, False, False, False, 0)

f_const = chrono.ChFunction_Const()
f_seq.InsertFunct(f_const, 0.4, 1, True, False, False, -1)

f_const_acc2 = chrono.ChFunction_ConstAcc()
f_const_acc2.Set_end(0.6)  # ramp length
f_const_acc2.Set_av(0.3)   # acceleration ends after 30% length
f_const_acc2.Set_aw(0.7)   # deceleration starts after 70% length
f_const_acc2.Set_h(-0.2)   # ramp height
f_seq.InsertFunct(f_const_acc2, 0.6, 1, True, False, False, -1)

f_seq.Setup();

plot_function(x, f_seq, 'Function sequence')

# Repeating sequence
# ------------------

f_part1 = chrono.ChFunction_Ramp()
f_part1.Set_ang(0.50)
f_part2 = chrono.ChFunction_Const()
f_part2.Set_yconst(1.0)
f_part3 = chrono.ChFunction_Ramp()
f_part3.Set_ang(-0.50)

f_seq = chrono.ChFunction_Sequence()
f_seq.InsertFunct(f_part1, 1.0, 1, True)
f_seq.InsertFunct(f_part2, 1.0, 1., True)
f_seq.InsertFunct(f_part3, 1.0, 1., True)

f_rep_seq = chrono.ChFunction_Repeat()
f_rep_seq.Set_fa(f_seq)
f_rep_seq.Set_window_length(3.0)
f_rep_seq.Set_window_start(0.0)
f_rep_seq.Set_window_phase(3.0)

plot_function(x, f_rep_seq, 'Repeating sequence')

plt.show()
