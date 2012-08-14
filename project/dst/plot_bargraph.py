#!/usr/bin/python

from pylab import *
import matplotlib.pyplot as plt
import numpy as np
import sys
import os
import common_plotting as cp

if(len(sys.argv) != 1):
    print "Usage: plot_bargraph.py.  Run in the dir with testing_results.txt."
    sys.exit(1)


# -- Config
cp.setup()

# -- Load data
tmpfile = ".python-aosetuhsaotehusaoeu"
os.system("cat dst/evaluation.txt | grep 'Mean Hamming loss' | awk '{print $NF}' > " + tmpfile)
dst_mhl = np.loadtxt(tmpfile)
os.system("cat dst/evaluation.txt | grep 'Mean capped normalized accuracy' | awk '{print $NF}' > " + tmpfile)
dst_mcna = np.loadtxt(tmpfile)
os.system("cat houghtrack/evaluation.txt | grep 'Mean Hamming loss' | awk '{print $NF}' > " + tmpfile)
ht_mhl = np.loadtxt(tmpfile)
os.system("cat houghtrack/evaluation.txt | grep 'Mean capped normalized accuracy' | awk '{print $NF}' > " + tmpfile)
ht_mcna = np.loadtxt(tmpfile)
os.system("rm " + tmpfile)

# -- Get overall performances
average_dst_mcna = np.sum(dst_mcna) / len(dst_mcna)
average_ht_mcna = np.sum(ht_mcna) / len(ht_mcna)
print "HT: " + str(average_ht_mcna)
print "DST: " + str(average_dst_mcna)
print "Error reduction: " + str(((1-average_ht_mcna)-(1-average_dst_mcna)) / (1-average_ht_mcna))
dst_mcna = hstack([dst_mcna, 0, 0, average_dst_mcna, 0])
ht_mcna = hstack([ht_mcna, 0, 0, average_ht_mcna, 0])

average_dst_mhl = np.sum(dst_mhl) / len(dst_mhl)
average_ht_mhl = np.sum(ht_mhl) / len(ht_mhl)
dst_mhl = hstack([dst_mhl, 0, 0, average_dst_mhl, 0])
ht_mhl = hstack([ht_mhl, 0, 0, average_ht_mhl, 0])

# -- Set up figure
indices = np.arange(len(dst_mcna))
width = 0.35
fig = figure(figsize=(15, 5))

# -- Capped Normalized Accuracy
ax1 = plt.subplot2grid((2, 5), (0, 1), colspan=4)
for loc, spine in ax1.spines.iteritems():
    if loc in ['right','top']:
        spine.set_color('none')
ax1.yaxis.set_ticks_position('left')
ax1.xaxis.set_ticks([])
rects1dst = ax1.bar(indices, dst_mcna, width, color='g')
rects1ht = ax1.bar(indices + width, ht_mcna, width, color='gray')
ax1.yaxis.set_ticks([ax1.yaxis.get_majorticklocs()[0], ax1.yaxis.get_majorticklocs()[-1]])

# -- Hamming Loss
ax2 = plt.subplot2grid((2, 5), (1, 1), colspan=4)
ax2.xaxis.set_ticks_position('bottom')
ax2.xaxis.set_ticklabels([str(x) for x in range(1, len(indices) - 3)] + ['', '', 'Overall', ''])
ax2.xaxis.set_ticks(indices + width)
for loc, spine in ax2.spines.iteritems():
    if loc in ['right','top']:
        spine.set_color('none')
rects2dst = ax2.bar(indices, dst_mhl, width, color='g')
rects2ht = ax2.bar(indices + width, ht_mhl, width, color='gray')
ax2.yaxis.set_ticks_position('left')
ax2.yaxis.set_ticks([ax2.yaxis.get_majorticklocs()[0], ax2.yaxis.get_majorticklocs()[-1]])

# -- Annotate and save
xlabel('Sequence Number')
txt1 = figtext(0.1, 0.7, "Normalized\naccuracy", size='xx-large')
txt2 = figtext(0.1, 0.2, "Hamming\nloss", size='xx-large')
fl = figlegend((rects2dst[0], rects2ht[0]), ('Ours', 'HoughTrack'), loc=(0.0, 0.45))

pad = 0.1
bbi = 'tight'
bbea = [txt1]
savefig('bargraph.pdf', bbox_inches=bbi, pad_inches=pad, bbox_extra_artists=bbea)
savefig('bargraph.eps', bbox_inches=bbi, pad_inches=pad, bbox_extra_artists=bbea)
savefig('bargraph.png', bbox_inches=bbi, pad_inches=pad, bbox_extra_artists=bbea)

