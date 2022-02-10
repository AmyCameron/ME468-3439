import numpy as np

# define function f(x) = e^x - 4
def f(x):
    return np.exp(x) - 4

# define Bisection method
    # function is the problem you want to solve
    # x_low, x_high define the section
    # errot is error range
def bisecRecrusive(function, x_low, x_high, error):
    count = 1 # counter

    if f(x_low) * f(x_high) >= 0:
        print("Error: section you defined is not valid")
        quit()

    while(True):
        x_half = (x_high + x_low)/2
        print(" ")
        print("Iteration", count, ": ", '{:.10f}'.format(x_low), '{:.10f}'.format(x_high), '{:.10f}'.format(x_half), '{:.10f}'.format(function(x_half)))
        if (f(x_half) * f(x_high)) > 0:
            x_high = x_half
        else:
            x_low = x_half
        if (x_high - x_low) < error:
            break
        count += 1

    return x_half

x_0 = bisecRecrusive(f, -0.2, 6.0, 1/(10*10*10*10*10*10))

print(" ")
print("Solution: When x=", x_0, "the value f(x) = 0 apptoximately")
