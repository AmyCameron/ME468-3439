# Newton Raphson used to solve the nonlinear system
# x-exp(y)=0
# log(1+x)-cos(y)=0

import numpy as np

def residual(x,y):
    return np.array([[x - np.exp(y)],[np.log(1 + x) - np.cos(y)]])

def jacobian(x,y):
    return np.array([[1,- np.exp(y)],[1 / (1 + x),np.sin(y)]])

# provide initial guess
x = 1.
y = 0.
normResidual = 1.
step = 0

while True:
    res = residual(x,y)
    jac = jacobian(x,y)
    correction = np.linalg.solve(jac,res)
    x = x - correction[0,0]
    y = y - correction[1,0]
    
    # reporting on progress to convergence
    normRes = np.linalg.norm(res)
    normCor = np.linalg.norm(correction)
    step = step + 1
    print("\nStep: ", step)
    print("x=", round(x,7), "\ty=",round(y,7))
    print("Residual norm: ", round(normRes,7)) 
    print("Correction norm: ", round(normCor,7)) 
    if normCor < 1.E-6:
        break
