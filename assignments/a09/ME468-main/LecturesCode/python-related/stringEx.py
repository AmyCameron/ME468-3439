#!/usr/bin/python3

aString = "Bucky the badger"
print(type(aString))
print("aString: ", aString)

strLength = len(aString)
myRange = range(0,strLength)
for i in myRange:
    print(aString[i])

print("Again, pythonic now:")
for i in aString:
    print(i)

