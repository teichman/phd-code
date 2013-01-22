#!/usr/bin/python

from pylab import *
import matplotlib.pyplot as plt
from matplotlib import rc
import numpy as np
import sys
import os
import common_plotting as cp

if(len(sys.argv) != 2):
    print "Usage: plot_planarity_experiment.py PLANARITY_RESULTS_TXT"
    print "       Output will be saved in the directory that contains the txt results."
    sys.exit(1)

cp.setup()

path = sys.argv[1];
data = np.loadtxt(sys.argv[1])
print data

fig = plt.figure(figsize=(10,5))
ax = fig.add_subplot(1, 1, 1)
ax.set_xlabel('Distance (meters)')
ax.set_ylabel('Mean error (meters)')
grid(True)
ylim(0, np.amax(data[:, 1]) + 0.01)

ax.scatter(data[:, 0], data[:, 1], color='black', marker='x', s=40)

path, extension = os.path.splitext(sys.argv[1])
savefig(path + '.pdf')
savefig(path + '.png')

