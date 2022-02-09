from random import random
from sys import argv

import numpy as np
import numpy.random as rd

import datetime
import time

dtime = datetime.datetime.now()
hour = dtime.hour
n = int(argv[1])
rd.seed(hour)
A = rd.uniform(low=-1, high=1, size=(n,n))
b = np.ones(n, dtype="float")
c = []
eltime_loop = []
eltime_np = []

#using double loop
for l in range(20):
    start_time = time.perf_counter()
    for i in range(A.shape[1]):
        temp = []
        for j in range(b.shape[0]):
            temp.append(A[i,j]*b[j])
        entry = sum(temp)
        c.append(entry)
    end_time = time.perf_counter()
    elapsed_time = end_time - start_time
    eltime_loop.append(elapsed_time)

meantime = np.mean(eltime_loop)
stdtime = np.std(eltime_loop)


print(c[-1])
print(meantime)
print(stdtime)

#using numpy
for l in range(20):
    start_time = time.perf_counter()
    c = np.dot(A,b)
    end_time = time.perf_counter()
    elapsed_time = end_time - start_time
    eltime_np.append(elapsed_time)

meantime = np.mean(eltime_np)
stdtime = np.std(eltime_np)


print(c[-1])
print(meantime)
print(stdtime)
