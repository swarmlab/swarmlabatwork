#!/usr/bin/env python
TAG = "|   "
INDENT = 1

def level(line):
    i = 0
    while line.startswith(TAG):
        line = line[4:]
        i += 1
    return i

def strip(line):
    return line.replace(TAG, "")

def parse(lines, start, stop):
    if start >= stop:
        return

    if ":" in strip(lines[start]):
       # has a class label
       lev = level(lines[start])
       strs = strip(lines[start]).split(":")
       print " "*4*(lev+INDENT) + "if " + strs[0] + ":"
       label = strs[1]
       label = label[1:label.index("(")-1]
       print " "*4*(lev+1+INDENT) + "return \"" + label + "\""
       parse(lines, start+1, stop)
       return 

    # no class label expend
    lev = level(lines[start])
    print " "*4*(lev+INDENT) + "if " + strip(lines[start]) + ":"
    
    middle = None
    for i in xrange(start+1,stop):
        if level(lines[i]) == lev:
            middle = i
            break
    if middle == None:
        parse(lines, start+1,stop)
        # # no else
        # print " "*4*lev + "TODO"
        # if start+1 < middle:
        #     parse(lines, start+1, middle)
    else:
        # with else
        parse(lines, start+1, middle)
        #print " "*4*lev + "else:"
        parse(lines, middle, stop) 
    
fname = "tree.log"
f = open(fname, "r")
lines = [line.strip() for line in f]
f.close()

del lines[len(lines) - 1]

parse(lines, 0, len(lines))



