#!/usr/bin/python3

myDict = {'fName': 'bucky', 'sName': 'badger', 'age': 82, 'gpa': 4.0, 'major':'zoology'}
print('Keys: ', myDict.keys())
print('Values: ', myDict.values())
print('Items: ', myDict.items())

del myDict['major']
del myDict['gpa']
print('Got rid of some items: ', myDict.items())

myDict.clear()
print('Dictionary must be now empty: ', myDict.items())
