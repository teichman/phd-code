#!/usr/bin/python
import sys
from string import find

def main(args):
    infile = args[0]
    fin = open(infile,'r')
    flines = fin.readlines()
    fin.close()
    if len(args) > 1:
        outfile = args[1]
    else:
        outfile = args[0]
        print "Overwriting initial file"
    fout = open(outfile,'w')
    if find(flines[0], "YAML") < 0:
        fout.write("%YAML:1.0\n")
    for line in flines:
        if (find(line, "matrix") >=0 or find(line,"coefficients") >= 0) and find(line,"opencv")<0:
            fout.write("%s !!opencv-matrix\n"%line.strip())
            fout.write("  dt: f\n")
        else:
            fout.write(line)
    fout.close()

if __name__ == '__main__':
    main(sys.argv[1:])
