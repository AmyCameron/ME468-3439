import numpy as np
import matplotlib.pyplot as plt

def kf(T, Z, U, mu0, Sigma0, A, B, C, Q, R):
    # initial state
    mu = mu0
    Sigma = Sigma0

    # position
    M_x = [mu[0]]
    M_y = [mu[1]]

    for i in range(T):
        # predicted (step1,step2)
        mu_ = A * mu + B * U[i]
        Sigma_ = R + A * Sigma * A.T

        # kalmanGain~set new ones
        # K: kalman gain (step3)
        S = C * Sigma_ * C.T + Q
        K = Sigma_ * C.T * S.I
        # new average (step4)
        zi = Z[i] - C * mu_
        mu = mu_ + K * zi
        # new average (step5)
        Sigma = (A - K * C)* Sigma_
        M_x.append(mu[0])
        M_y.append(mu[1])
    
    return M_x, M_y

def main():
    # x = A * x_ + B * u + w, w ~ N(R,Q)
    # z = C * x + v, v ~ N(0,Q)
    
    # initial value
    mu0 = np.mat([[0],[0],[0]])    
    Sigma0 = np.mat([[0.1, 0, 0], [ 0, 0.1, 0], [ 0, 0, 0.1]])

    A = np.mat([[1,0,0], [0,1,0],[0,0,1]])
    #?
    #B = np.mat([[0.1,0],[0,0],[0,0.1]])
    B = np.mat([[0.1*np.cos(mu0[2]),0],[0.1*np.sin(mu0[2]),0],[0,0.1]])
    C = np.mat([[1,0,0], [0,1,0]])
    # noise distributions
    R = np.mat([[0.5,0,0], [0,0.5,0],[0,0,0.1]])
    Q = np.mat([[0.8,0],[0,0.8]])

    # 100sec & dt=0.1
    T = 1000
    # motion model 
    x = np.mat([[0],[0],[0]])
    X = [x] 
    # measurement model
    z = np.mat([[0],[0]])
    Z = [z]
    # constant for this assignment?
    u = np.mat([[2.2],[0.3]])
    U = [u] 

    # gps data set
    for i in range(T):
        x = A * x + B * u + np.random.multivariate_normal([0, 0, 0], R, 1).T
        X.append(x)
        z = C * x + np.random.multivariate_normal([0, 0], Q, 1).T
        Z.append(z)
        U.append(u)

    # run calmanfilter
    M_x, M_y = kf(T, Z, U, mu0, Sigma0, A, B, C, Q, R)
    print(M_x[3])
    # visual
    fig = plt.figure(figsize=(8, 6))
    plt.title("Position",fontsize=12)
    plt.plot(M_x, M_y,label='sample')
    plt.xscale("linear")
    plt.yscale("linear")
    plt.ylabel("y",fontsize=12)
    plt.xlabel("x",fontsize=12)
    plt.show()

if __name__ == '__main__':
    main()