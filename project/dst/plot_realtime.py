#!/usr/bin/python

from pylab import *
import matplotlib.pyplot as plt
from matplotlib import rc
import numpy as np
import sys
import os
import common_plotting as cp
            
if(len(sys.argv) != 2):
    print "Usage: plot_mask.py AVS_DIR"
    sys.exit(1)

# -- Config
cp.setup()
#rcParams['font.size'] = 10

# -- Load data
path = sys.argv[1];
tmpfile = ".python-astoehusatohesunthaosnetuh"
names = []
times = []
losses = []

names.append("Original");

os.system("grep 'Average time' `find " + path + " -wholename '*mask0_skip01/testing_results.txt' | sort` | awk '{print $NF}' > " + tmpfile)
times.append(np.loadtxt(tmpfile))
os.system("for file in `find " + path + " -name testing_results.txt | sort | grep mask0_skip01`; do cat $file | grep 'Mean capped normalized loss' | awk '{sum += $NF} END {print sum / NR}'; done > " + tmpfile)
losses.append(np.loadtxt(tmpfile))

names.append("Mask")
os.system("grep 'Average time' `find " + path + " -wholename '*mask1_skip01/testing_results.txt' | sort` | awk '{print $NF}' > " + tmpfile)
times.append(np.loadtxt(tmpfile))
os.system("for file in `find " + path + " -name testing_results.txt | sort | grep mask1_skip01`; do cat $file | grep 'Mean capped normalized loss' | awk '{sum += $NF} END {print sum / NR}'; done > " + tmpfile)
losses.append(np.loadtxt(tmpfile))

names.append("Mask \& \n 50\% downsample")
os.system("grep 'Average time' `find " + path + " -wholename '*mask1_skip02/testing_results.txt' | sort` | awk '{print $NF}' > " + tmpfile)
times.append(np.loadtxt(tmpfile))
os.system("for file in `find " + path + " -name testing_results.txt | sort | grep mask1_skip02`; do cat $file | grep 'Mean capped normalized loss' | awk '{sum += $NF} END {print sum / NR}'; done > " + tmpfile)
losses.append(np.loadtxt(tmpfile))

names.append("Mask \& \n 75\% downsample")
os.system("grep 'Average time' `find " + path + " -wholename '*mask1_skip04/testing_results.txt' | sort` | awk '{print $NF}' > " + tmpfile)
times.append(np.loadtxt(tmpfile))
os.system("for file in `find " + path + " -name testing_results.txt | sort | grep mask1_skip04`; do cat $file | grep 'Mean capped normalized loss' | awk '{sum += $NF} END {print sum / NR}'; done > " + tmpfile)
losses.append(np.loadtxt(tmpfile))

os.system("rm " + tmpfile)

# -- Make a nice bargraph.
fig = plt.figure(figsize=(13,4))
loss_ax = fig.add_subplot(111)
time_ax = loss_ax.twinx()
width = 1

ind = np.arange(0, size(names)) * 3 * width
vals = [np.mean(l) for l in losses]
loss_stdevs = [np.std(l) for l in losses]
loss_bars = loss_ax.bar(ind, vals, width, color='green', yerr=loss_stdevs, ecolor='k', capsize=5)
loss_ax.set_ylabel("Loss")
loss_ax.set_xticks(ind + width)
loss_ax.set_xticklabels(names)
for rect in loss_bars:
    rect.set_alpha(0.5)

time_ind = ind + width
vals = [np.mean(t) for t in times]
time_stdevs = [np.std(t) for t in times]
time_bars = time_ax.bar(time_ind, vals, width, color='gray', yerr=time_stdevs, ecolor='k', capsize=5)
time_ax.set_ylabel("Time (ms)")
for rect in time_bars:
    rect.set_alpha(0.5)

# Add space for the annotations.
loss_ax.set_ylim([1.2 * x for x in loss_ax.get_ylim()])
time_ax.set_ylim([1.2 * x for x in time_ax.get_ylim()])
xlim = loss_ax.get_xlim()
loss_ax.set_xlim([xlim[0] - width, xlim[1] + width])
#loss_ax.set_xlim([1.1 * x - width for x in loss_ax.get_xlim()])

# Add the annotations.
def labelLosses(ax, rects, stdevs, unit):
        for (idx, rect) in enumerate(rects):
            height = rect.get_height()
            ax.text(rect.get_x() + width / 2. + width * 0.05, height + 0.02, ('%0.2f \n $\pm$ %0.2f' % (height, stdevs[idx])) + unit, ha='center', va='bottom', fontsize='small', multialignment='center')

def labelTimes(ax, rects, stdevs, unit):
    for (idx, rect) in enumerate(rects):
            height = rect.get_height()
            width = rect.get_width()
            ax.text(rect.get_x() + width / 2., height + 30, ('%3.0f \n $\pm$ %3.1f' % (height, stdevs[idx])) + unit, ha='center', va='bottom', fontsize='small', multialignment='center')

labelTimes(time_ax, time_bars, time_stdevs, "ms")
labelLosses(loss_ax, loss_bars, loss_stdevs, "")

#plt.setp(loss_ax.get_xticklabels(), fontsize=10)

# Chop off unnecessary lines.
loss_ax.yaxis.set_ticks_position('none')
loss_ax.xaxis.set_ticks_position('bottom')
loss_ax.set_ylabel("")
loss_ax.set_xlabel("")
loss_ax.set_yticks([])
for loc, spine in loss_ax.spines.iteritems():
    if loc in ['right','top','left']:
        spine.set_color('none')

time_ax.yaxis.set_ticks_position('none')
time_ax.xaxis.set_ticks_position('bottom')
time_ax.set_ylabel("")
time_ax.set_xlabel("")
time_ax.set_yticks([])
for loc, spine in time_ax.spines.iteritems():
    if loc in ['right','top','left']:
        spine.set_color('none')

# Save.
savefig(path + '/realtime.png', bbox_inches='tight', pad_inches=0.0)
savefig(path + '/realtime.pdf', bbox_inches='tight', pad_inches=0.0)

