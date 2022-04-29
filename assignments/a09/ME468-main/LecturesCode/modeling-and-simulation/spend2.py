"""
===========================
The sngle pendulum problem
===========================

Simple pendulum simulation and FFT analysis.
Note: to get meaningful results from FFT, we must simulate a longer time (more periods)
"""

import numpy as np
import scipy.integrate as integrate
import scipy.fftpack as fftpack
import scipy.signal as signal
import matplotlib.pyplot as plt

G = 9.8  # acceleration due to gravity, in m/s^2
L = 1.0  # length of pendulum in m
M = 1.0  # mass of pendulum in kg
omg = np.sqrt(G/L)

def derivs(t, y):
    dydt = np.zeros_like(y)
    dydt[0] = y[1]
    dydt[1] = - (G / L) * np.sin(y[0])
    #dydt[1] = - (G / L) * y[0]
    return dydt

dt = 0.01
tend = 100.0
t = np.arange(0.0, tend, dt)
y0 = np.radians([45.0, 0.0])
sol = integrate.solve_ivp(derivs, [0, tend], y0, method='LSODA', t_eval=t, rtol=1e-4, atol=1e-8)

theta = sol.y[0]
theta_dot = sol.y[1]

# Data analysis

fft = fftpack.fft(theta)
freqs = fftpack.fftfreq(len(fft)) * (1/dt)
f_peak = np.argmax(np.abs(fft))
print(' freq = ', freqs[f_peak], '  omega = ', 2 * np.pi * freqs[f_peak])

N = len(theta)
figC, axC = plt.subplots()
figC.set_size_inches(12,6)
axC.bar(freqs[:N // 2], np.abs(fft)[:N // 2] * 1 / N, width=0.01)
axC.set_xlim(0, 5)
axC.set_xlabel('frequency (Hz)')

th_peaks, _ = signal.find_peaks(theta, prominence=1)
print('Period = ', t[th_peaks[1]] - t[th_peaks[0]])


plt.show()
