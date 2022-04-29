#!/usr/bin/python3

aList = [1, 2.3, "ok"]

aTouple = (1, 2.3, "ok")

aRange = range(3,9)

aDictionary = {'name': 'Dan', 'age': 23}

# tuple ok, list not ok
aSet = {'mimi', (1,"blah")}

theBigTouple = (1, aSet, aDictionary, aRange, aTouple, aList)
print(type(theBigTouple))

# working with a sequence allows me to do this:
print("An element of the sequence: ", theBigTouple[2])

# here's the range
print("The range stored by the big touple: ", theBigTouple[3])
print("Print now the elements in the range:")
for i in theBigTouple[3]:
    print(i)
