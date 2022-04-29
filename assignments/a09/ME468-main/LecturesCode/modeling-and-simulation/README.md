Sample codes from the lecture on "Modeling and Simulation"
====================

- robertson.py

  This program solves the Robertson chemical kinetics problem, a stiff system of 3 ODEs.
  
  Note that we use the LSODA integrator from the `scipy.integrate` subpackage which is a 
  stiff multi-step integrator, appropriate for this type of problems.
  
  The solution shows the multiple time-scales involved in this problem, characteristic 
  of a (very) stiff ODE.

- spend.py

  This program solves the two models of a simple pendulum problem:
  - a linear ODE initial value problem, obtained with the small angle approximation.
  - a nonlinear ODE initial value problem, the original model derived from applying Newton's 2nd law.
  
  The two ODEs are converted to 1st order and solved simultaneously (as a system of 4 DEs).
  The resulting animation, generated using `matplotlib.animation`, shows the time evolution of a pendulum using the two solutions superimposed.

- spend2.py

  This program solves the nonlinear simple pendulum DEs and performs an FFT analysis (using functions from the `scipy.fftpack` subpackage) on the solution to extract the natural frequency. 

  The period of the pendulum is obtained with the function find_peaks from the `scipy.signal` subpackage.
