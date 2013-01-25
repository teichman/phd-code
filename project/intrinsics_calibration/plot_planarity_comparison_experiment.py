#!/usr/bin/python

from pylab import *
import matplotlib.pyplot as plt
from matplotlib import rc
import numpy as np
import sys
import os
import common_plotting as cp

if(len(sys.argv) != 2):
    print "Usage: plot_planarity_comparison_experiment.py EXPERIMENT_DIR"
    sys.exit(1)

cp.setup()

path = sys.argv[1];
raw = np.loadtxt(path + "/raw.txt")
undistorted = np.loadtxt(path + "/undistorted.txt")

fig = plt.figure(figsize=(10,6))
ax = fig.add_subplot(1, 1, 1)
ax.set_xlabel('Distance (meters)')
ax.set_ylabel('RMS error (meters)')
grid(True)

ax.scatter(undistorted[:, 0], undistorted[:, 2], color='red', marker='o', s=40, label='calibrated')
ax.scatter(raw[:, 0], raw[:, 2], color='black', marker='x', s=40, label='raw')
legend(loc=2)

ylim(0, max(float(np.amax(undistorted[:, 2])), float(np.amax(raw[:, 2]))) + 0.01)

savefig(path + '/comparison.svg')
savefig(path + '/comparison.pdf')
savefig(path + '/comparison.png')

