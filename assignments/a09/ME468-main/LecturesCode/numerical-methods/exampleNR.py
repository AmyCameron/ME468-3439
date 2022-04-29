# Use Newton method to solve the nonlinear system
# x-exp(y)=0
# log(1+x)-cos(y)=0

import numpy as np

def getJac(q, Jac):
    x = q[0,0]
    y = q[1,0]
    Jac[0, 0] = 1.
    Jac[0, 1] = -np.exp(y)
    Jac[1, 0] = 1. / (1. + x)
    Jac[1, 1] = np.sin(y)


def get_phi(q, phi):
    x = q[0,0]
    y = q[1,0]
    phi[0,0] = x - np.exp(y)
    phi[1,0] = np.log(1. + x) - np.cos(y)


def main():
    q = np.array([[1.],[0.]])
    func = np.zeros(shape=(2,1))
    Jacobian = np.zeros(shape=(2,2))
    residual = 1.
    threasholdVal = 1.E-7
    iteration = 0

    while residual > threasholdVal:
        get_phi(q, func)
        getJac(q, Jacobian)
        correction = np.linalg.solve(Jacobian, func)
        q = q - correction
        residual = np.linalg.norm(correction)
        iteration += 1
        print('Iteration: ', iteration)
        print('Residual: ', residual)
        print('x: ', q[0,0])
        print('y: ', q[1,0])


main()