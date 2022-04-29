def argListAspects(firstArg, secondArg):
    firstArg += 9  # now the name firstArg points to object "10"
    secondArg.append('d') # the mutable list object changed here
    print('Inside the function:', firstArg, secondArg)

def main():
    i = 1
    theList = ['a', 'b' , 'c']
    print('Before function call:', i, theList)

    # call function; some args (the mutables) get changed, some not
    argListAspects(i, theList)
    print('After function call:', i, theList)

main()