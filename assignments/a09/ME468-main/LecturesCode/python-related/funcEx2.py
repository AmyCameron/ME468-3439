#!/usr/bin/python3

# function with default arguments
def overview(person='Mary', age='30', gender='female'):
    return person + ' is a ' + age + ' old ' + gender + '.'


# call a function, no arguments
print(overview())
print(overview( age='27', gender='male', person='John'))
print(overview( 'Paul', '77', 'male'))
print(overview( 'Sue'))
