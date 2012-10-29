#!/usr/bin/python

from pylab import *
import numpy
import sys
import os
import common_plotting as cp

if(len(sys.argv) < 2):
    print "Usage: python plot_accuracy_vs_iter.py VALS [VALS ...]"
    sys.exit(1)

# -- Config.
cp.setup()

# -- Load data and draw.
fig = figure()
xlabel('Learning Iteration')
ylabel('Accuracy')
grid(True)
for valspath in sys.argv[1:]:
    vals = numpy.loadtxt(valspath)
    scatter(10 * arange(1, len(vals)+1), ones(len(vals)) - vals, marker=cp.markers[0], s=40)
ylim(0, 1)

savefig('accuracy_vs_iter.pdf')
savefig('accuracy_vs_iter.png')


