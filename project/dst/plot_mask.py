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
#os.system("grep 'Average time' `find " + path + " -wholename '.*/mask1_skip02/testing_results.txt' | sort` | awk '{print $NF}' > " + tmpfile)
#maskskiptimes = np.loadtxt(tmpfile)
#os.system("grep 'Overall mean capped normalized loss' `find " + path + " -wholename '.*/mask1_skip02/testing_results.txt' | sort` | awk '{print $NF}' > " + tmpfile)
#maskskiplosses = np.loadtxt(tmpfile)
os.system("rm " + tmpfile)

# -- Make a nice bargraph.
fig = plt.figure()
loss_ax = fig.add_subplot(111)
time_ax = loss_ax.twinx()
width = 1

ind = np.array([0, 3])
vals = np.array([np.mean(losses), np.mean(masklosses)])
loss_stdevs = np.array([np.std(losses), np.std(masklosses)])
loss_bars = loss_ax.bar(ind, vals, width, color='green', yerr=loss_stdevs, ecolor='k')
loss_ax.set_ylabel("Loss")
loss_ax.set_xticks([1, 4])
loss_ax.set_xticklabels(["Original", "Boundary mask"])

ind = np.array([1, 4])
vals = np.array([np.mean(times), np.mean(masktimes)])
time_stdevs = np.array([np.std(times), np.std(masktimes)])
time_bars = time_ax.bar(ind, vals, width, color='gray', yerr=time_stdevs, ecolor='k')
time_ax.set_ylabel("Time (ms)")

# Add space for the annotations.
loss_ax.set_ylim([1.2 * x for x in loss_ax.get_ylim()])
time_ax.set_ylim([1.2 * x for x in time_ax.get_ylim()])

# Add the annotations.
def labelLosses(ax, rects, stdevs, unit):
        for (idx, rect) in enumerate(rects):
            height = rect.get_height()
            ax.text(rect.get_x()+rect.get_width()/2., 1.05*height, ('%0.2f $\pm$ %0.2f' % (height, stdevs[idx])) + " " + unit, ha='center', va='bottom', fontsize='smaller')

def labelTimes(ax, rects, stdevs, unit):
    for (idx, rect) in enumerate(rects):
            height = rect.get_height()
            width = rect.get_width()
#            ax.annotate(('%3.0f $\pm$ %3.1f' % (height, stdevs[idx])), xy=(rect.get_x() + width/2., 1.05 * height), xytext=(50, 50), textcoords='offset points', arrowprops=dict(arrowstyle='->', facecolor='black'))
            ax.text(rect.get_x() + width / 2. + 0.05, height + 30, ('%3.0f $\pm$ %3.1f' % (height, stdevs[idx])) + " " + unit, ha='center', va='bottom', fontsize='smaller')

labelTimes(time_ax, time_bars, time_stdevs, "ms")
labelLosses(loss_ax, loss_bars, loss_stdevs, "")

print "Times"
print times
print np.std(times)
print "Times with mask"
print masktimes
print np.std(masktimes)
print "Losses"
print losses
print np.std(losses)
print "Losses with mask"
print masklosses
print np.std(masklosses)


# Save.
savefig(path + '/mask.png')
savefig(path + '/mask.pdf')
