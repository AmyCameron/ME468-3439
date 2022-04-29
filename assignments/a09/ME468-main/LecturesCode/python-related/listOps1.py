#!/usr/bin/python3

import cmath
myList = ['ok', complex(1,-1), 3.3, [1,2], 'why dear?']

myList.append(3)
print("One more term: \n", myList,"\n")

myList.extend(['why not?',[4,3]])
print("Two more terms to list, one a list itself: \n", myList,"\n")

myList.insert(2,complex(3,4))
print("First complex number has a very complex neighbor: \n", myList)