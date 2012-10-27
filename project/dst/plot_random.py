#!/usr/bin/python

from pylab import *
import matplotlib.pyplot as plt
from matplotlib import rc
import numpy as np
import sys
import os
import common_plotting as cp
            
if(len(sys.argv) != 2):
    print "Usage: plot_mask.py RANDOM_DIR"
    sys.exit(1)

# -- Config
cp.setup()

# -- Load data
path = sys.argv[1];
tmpfile = ".python-asoethusatohesntuhasoneth"
os.system("grep 'Overall mean capped normalized loss' `find " + path + " -name testing_results.txt` | awk '{print $NF}' > " + tmpfile)
losses = np.loadtxt(tmpfile)
os.system("rm " + tmpfile)

accuracies = 1 - losses

fig = plt.figure()
ax = fig.add_subplot(1, 1, 1)
ax.hist(accuracies, 10)

ax.set_xlim([0, 1])
ax.set_xlabel("Accuracy")
ax.set_ylabel("Samples")

ax.yaxis.set_ticks_position('left')
ax.xaxis.set_ticks_position('bottom')
#ax.xaxis.set_ticks(ax.xaxis.get_majorticklocs()[1:])
#ax.yaxis.set_ticks(ax.yaxis.get_majorticklocs()[1:])
#ax.xaxis.set_ticks([ax.xaxis.get_majorticklocs()[0], ax.xaxis.get_majorticklocs()[-1]])
ax.yaxis.set_ticks([ax.yaxis.get_majorticklocs()[-1]])
for loc, spine in ax.spines.iteritems():
    if loc in ['right','top']:
        spine.set_color('none')


# Save.
savefig(path + '/random.png')
savefig(path + '/random.pdf')


