#!/usr/bin/python

from pylab import *
import matplotlib.pyplot as plt
from matplotlib import rc
import numpy as np
import sys
import os
import common_plotting as cp

if(len(sys.argv) != 2):
    print "Usage: plot_quantity_experiment.py QUANTITY_EXPERIMENT_DIR"
    sys.exit(1)

path = sys.argv[1];
tmpfile = ".python-asotehusntahoesuthasonethusaonte"

os.system("for file in `find " + path + " -name results.txt | sort`; do echo $(cat ${file%/results.txt}/info.txt | grep 'Total seconds of data used for training' | awk '{print $NF}') $(cat $file | grep 'Error reduction' | awk '{print $NF}'); done > " + tmpfile)
data = np.loadtxt(tmpfile)
os.system("rm " + tmpfile)
# print data

fig = plt.figure(figsize=(10,5))
ax = fig.add_subplot(1, 1, 1)
ax.set_xlabel('Seconds of training data')
ax.set_ylabel('Error reduction')
grid(True)

ax.scatter(data[:, 0], data[:, 1], color='black', marker='x', s=40)
savefig(path + '/quantity_experiment.pdf')
savefig(path + '/quantity_experiment.png')

