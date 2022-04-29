#!/usr/bin/python3

# define some helper function
def my_func(a):
    return a*a + 1

# function q passed like any other object
def apply_func(q, b):
    return q(b)

# call a function, with another function as argument
print(apply_func(my_func, 3))

# example of lambda
# use if you don't want to bother defining my_func
print('Result, via lambda: ', apply_func(lambda z: z*z+1, 3))