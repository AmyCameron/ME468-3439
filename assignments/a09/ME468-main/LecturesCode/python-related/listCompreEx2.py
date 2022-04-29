#!/usr/bin/python3

myList = ['o', 'd', 'b', 't', 'v', 'r']
print('Original list: ', myList)

# applying a filter
myLIST = [elem.upper() for elem in myList if elem>'m']
print('Upper case list: ', myLIST)

# embedded comprehension
my2ndLIST = [itm + itm for itm in \
    [elem.upper() for elem in myList if elem>'m']]

print('2nd list: ', my2ndLIST)