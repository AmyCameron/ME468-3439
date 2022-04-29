#!/usr/bin/python3
# might have to change "/usr/bin/python3" to point to where python3 lives...

import sys, os, glob

def doSearch(fileNameSnippet): 
    dummy = "**/*"+fileNameSnippet+"*"
    matches = glob.glob(dummy, recursive=True)
    for match in matches:
        print(match)

os.chdir(sys.argv[1])
doSearch(sys.argv[2])
