#!/usr/bin/python

from pylab import *
import matplotlib.pyplot as plt
from matplotlib import rc
import numpy as np
import sys
import os
import common_plotting as cp
import matplotlib.cm as cm

            
if(len(sys.argv) != 3):
    print "Usage: plot_timing.py AVS_DIR VGA_DIR"
    sys.exit(1)

# -- Config
cp.setup()

# -- Load data
avspath = sys.argv[1];
vgapath = sys.argv[2];
tmpfile = ".python-asotunsatoheuthaoneut"

os.system("cat " + vgapath + "/*-vga.txt | grep 'Total feature computation' | grep milliseconds | awk '{sum += $4} END {print sum / NR}' > " + tmpfile);
vga_feature = np.loadtxt(tmpfile)
os.system("cat " + vgapath + "/*-vga.txt | grep maxflow | grep milliseconds | awk '{sum += $2} END {print sum / NR}' > " + tmpfile)
vga_maxflow = np.loadtxt(tmpfile)

os.system("grep 'Total feature computation' `find " + avspath + " -name testing_results.txt-log.txt | grep mask1_skip02` | awk '{sum += $4} END {print sum / NR}' > " + tmpfile)
qqvga_feature = np.loadtxt(tmpfile)
os.system("grep maxflow `find " + avspath + " -name testing_results.txt-log.txt | grep mask1_skip02` | awk '{sum += $2} END {print sum / NR}' > " + tmpfile)
qqvga_maxflow = np.loadtxt(tmpfile)

os.system("rm " + tmpfile)

# -- Plot VGA results.
figw = 15
figh = 5
fig = plt.figure(figsize = (figw, figh))

def timingPlot(ax, feat, gc):
    times = np.array([feat, gc])
    fracs = times / (feat + gc);
    labels = [("%0.0f ms" % x) for x in times]
    explode = [0.05, 0.05]
    print fracs
    pie = ax.pie(fracs, explode=explode, labels=labels, autopct='', shadow=False)
    for w in pie[0]:
        w.set_alpha(0.5)
    return pie

vga_ax = fig.add_subplot(1, 3, 1, aspect='equal')
timingPlot(vga_ax, vga_feature, vga_maxflow)
vga_ax.set_title("640 $\\times$ 480", bbox={'facecolor':'0.8', 'pad':10}, position=(0.5, -0.1))

# -- Plot QQVGA results.
qqvga_ax = fig.add_subplot(1, 3, 3, aspect='equal')
pie = timingPlot(qqvga_ax, qqvga_feature, qqvga_maxflow)
qqvga_ax.set_title("160 $\\times$ 120", bbox={'facecolor':'0.8', 'pad':10}, position=(0.5, -0.1))

# -- Legend.
leg = fig.legend(pie[0], ("Features", "Graph cuts"), loc='center', bbox_to_anchor = (0.5, 0.5))

# -- Save.
savefig(avspath + '/timing.png')
savefig(avspath + '/timing.pdf')


