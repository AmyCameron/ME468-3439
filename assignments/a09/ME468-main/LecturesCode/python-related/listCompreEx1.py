#!/usr/bin/python3

myList = ['o', 'd', 'b', 't', 'v', 'r']
print('Original list: ', myList)

myLIST = [elem.upper() for elem in myList if elem>'b']
print('Upper case list: ', myLIST)
