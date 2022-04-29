#!/usr/bin/python3

myDict = {'fName': 'bucky', 'sName': 'badger', 'age': 82, 'gpa': 4.0, 'major':'zoology'}
print(type(myDict))

for key in myDict.keys():
    print(key, myDict[key])

print('\nInserting a new pair:')
myDict['minor'] = 'ME'
for key in myDict.keys():
    print(key, myDict[key])