#!/usr/bin/python

from pylab import *
import matplotlib.pyplot as plt
import numpy as np
import sys
import os
import common_plotting as cp
            

if(len(sys.argv) != 2):
    print "Usage: plot_mask.py AVS_DIR"
    sys.exit(1)

# -- Config
cp.setup()

# -- Load data
path = sys.argv[1];
tmpfile = ".python-astoehusatohesunthaosnetuh"
os.system("grep 'Average time' `find " + path + " -wholename '.*/mask0_skip01/testing_results.txt' | sort` | awk '{print $NF}' > " + tmpfile)
times = np.loadtxt(tmpfile)
os.system("grep 'Overall mean capped normalized loss' `find " + path + " -wholename '.*/mask0_skip01/testing_results.txt' | sort` | awk '{print $NF}' > " + tmpfile)
losses = np.loadtxt(tmpfile)
os.system("grep 'Average time' `find " + path + " -wholename '.*/mask1_skip01/testing_results.txt' | sort` | awk '{print $NF}' > " + tmpfile)
masktimes = np.loadtxt(tmpfile)
os.system("grep 'Overall mean capped normalized loss' `find " + path + " -wholename '.*/mask1_skip01/testing_results.txt' | sort` | awk '{print $NF}' > " + tmpfile)
masklosses = np.loadtxt(tmpfile)
os.system("rm " + tmpfile)

# -- Make a nice bargraph.
fig = plt.figure()
host = fig.add_subplot(111)
par1 = host.twinx()
width = 1

ind = np.array([0, 3])
vals = np.array([np.mean(losses), np.mean(masklosses)])
stdevs = np.array([np.std(losses), np.std(masklosses)])
loss_bars = host.bar(ind, vals, width, color='green', yerr=stdevs, ecolor='k')
host.set_ylabel("Loss")
host.set_xticks([1, 4])
host.set_xticklabels(["Original", "Boundary mask"])

ind = np.array([1, 4])
vals = np.array([np.mean(times), np.mean(masktimes)])
stdevs = np.array([np.std(times), np.std(masktimes)])
time_bars = par1.bar(ind, vals, width, color='gray', yerr=stdevs, ecolor='k')
par1.set_ylabel("Time (ms)")

# Add space for the annotations.
host.set_ylim([1.1 * x for x in host.get_ylim()])
par1.set_ylim([1.1 * x for x in par1.get_ylim()])

# Add the annotations.
def labelLosses(ax, rects, unit):
        for rect in rects:
            height = rect.get_height()
            stdev = 0.139485349534
            ax.text(rect.get_x()+rect.get_width()/2., 1.05*height, ('%0.2f $\pm$ %0.2f' % (height, stdev)) + " " + unit, ha='center', va='bottom', fontsize='smaller')

def labelTimes(ax, rects, unit):
        for rect in rects:
            height = rect.get_height()
            stdev = 4.351232345345
            ax.text(rect.get_x()+rect.get_width()/2., 1.05*height, ('%3.0f $\pm$ %3.0f' % (height, stdev)) + " " + unit, ha='center', va='bottom', fontsize='smaller')
labelTimes(par1, time_bars, "ms")
labelLosses(host, loss_bars, "")

# Save.
savefig('mask.png')
savefig('mask.pdf')
