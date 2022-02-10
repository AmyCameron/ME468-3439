from random import random
from sys import argv
import numpy.random as rd

n = int(argv[1])
n_elems = rd.randint(low=0, high=1000, size=(n))

def sort(array):
    j = 0
    size = len(array)
    trial = int(size*size)

    for i in range(trial):
        if size-j-2 == -1:
            j = 0
        else:
            if array[size-j-1] < array[size-j-2]:
                array[size-j-1],array[size-j-2] = array[size-j-2], array[size-j-1] 
            j += 1
    return array

# before operation
print(n_elems[0], n_elems[-1])
# run function 'sort'
n_elems = sort(n_elems)
# after operation
print(n_elems[0], n_elems[-1])

    



