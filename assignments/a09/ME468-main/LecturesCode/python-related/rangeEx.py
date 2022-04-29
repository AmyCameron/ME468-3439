#!/usr/bin/python3

print("iterating over a list of tuples")
myList = [('a', 2.3), ('b', 'why dear?'), ('bucky', 'the', 'badger'), ('d', '!!!')]

# works, but not the elegant way to do it...
for i in range(len(myList)):
    print(i, ' --> ', myList[i])
print()

# a more pythonic way to do it:
print('more pythonic:')
for (i, item) in enumerate(myList):
    print(i, ' --> ', item)