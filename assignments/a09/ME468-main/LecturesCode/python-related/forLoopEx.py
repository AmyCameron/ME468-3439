#!/usr/bin/python3

myDict = {'fName': 'bucky', 'age': 82, 'gpa': 4.0}
for key, value in myDict.items():
    print(key)
    print(value)
    print()

# list of tuples
print("iterating over a list of tuples")
myList = [('a', 0), ('b', 1), ('c', 2), ('d', 3)]
for (x,y) in myList:
    print(x, "  ->  ", y)
