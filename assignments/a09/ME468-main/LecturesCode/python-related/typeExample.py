#!/usr/bin/python3

a = "python str"
print(type(a))

a = 99
print(type(a))

a = 38.91
print(type(a))

import cmath
a = complex(3,2)
print(type(a))

a = [1, 2.3, "ok"]
print(type(a))

a = (1, 2.3, "ok")
print(type(a))

a = range(3,9)
print(type(a))

a = {'name': 'Dan', 'age': 23}
print(type(a))

# tuple ok, list not ok
a = {'mimi', (1,"blah")}
print(type(a))

a = frozenset((3, 3.2, 'mimi'))
print(type(a))

a = True
print(type(a))

a = bytes('bucky', 'utf-8')
print(type(a))
print("a is: ", a)

a = bytearray('bucky', 'utf-8')
print(type(a))
print("a is: ", a)

a = memoryview(bytes('bucky', 'utf-8'))
print(type(a))
print("a is: ", a)
