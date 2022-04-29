#!/usr/bin/python3

import cmath
myTpl = ('ok', complex(1,-1), 3.3, [1,2], {1,2}, -1, 'why?')

print("The third element from the end, via negative index: ", myTpl[-3])

print("Indexing works like in Matlab: ", myTpl[2:5])

print("All elements, from beginning till two elements from the back: ", myTpl[0:-2])

print("You can skip elements: ", myTpl[1:-2:2])

print("Pythonic, print from end to beginning: ", myTpl[::-1])