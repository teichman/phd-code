#!/usr/bin/python

from pylab import *
import matplotlib.pyplot as plt
import numpy as np
import sys
import os
import common_plotting as cp

if(len(sys.argv) != 1):
    print "Usage: plot_bargraph.py.  Run in the workspace dir."
    sys.exit(1)


# -- Config
cp.setup()

# -- Load data
tmpfile = ".python-aosetuhsaotehusaoeu"
os.system("grep Angle evaluations/configuration*.txt | awk '{print $NF}' > " + tmpfile)
angles = np.loadtxt(tmpfile)
os.system("grep Euclidean evaluations/configuration*.txt | awk '{print $NF}' > " + tmpfile)
translations = np.loadtxt(tmpfile);

# -- Get overall performances
average_angle = np.sum(angles) / len(angles)
average_translation = np.sum(translations) / len(translations)

angles = hstack([angles, 0, 0, average_angle, 0])
translations = hstack([translations, 0, 0, average_translation, 0])

print angles
print translations

# -- Set up figure
indices = np.arange(len(angles))
width = 0.35
fig = figure(figsize=(15, 5))

# -- Angles
ax1 = plt.subplot2grid((2, 5), (0, 1), colspan=4)
for loc, spine in ax1.spines.iteritems():
    if loc in ['right','top']:
        spine.set_color('none')
ax1.yaxis.set_ticks_position('left')
ax1.xaxis.set_ticks([])
angle_rects = ax1.bar(indices, angles, width, color='g')
ax1.yaxis.set_ticks([ax1.yaxis.get_majorticklocs()[0], ax1.yaxis.get_majorticklocs()[-1]])

# -- Translations
ax2 = plt.subplot2grid((2, 5), (1, 1), colspan=4)
ax2.xaxis.set_ticks_position('bottom')
ax2.xaxis.set_ticklabels([str(x) for x in range(1, len(indices) - 3)] + ['', '', 'Overall', ''])
ax2.xaxis.set_ticks(indices + width)
for loc, spine in ax2.spines.iteritems():
    if loc in ['right','top']:
        spine.set_color('none')
translation_rects = ax2.bar(indices, translations, width, color='g')
ax2.yaxis.set_ticks_position('left')
ax2.yaxis.set_ticks([ax2.yaxis.get_majorticklocs()[0], ax2.yaxis.get_majorticklocs()[-1]])

# -- Annotate and save
xlabel('Sequence Number')
txt1 = figtext(0.1, 0.7, "Angle error\n(radians)", size='xx-large')
txt2 = figtext(0.1, 0.2, "Translation error\n(m)", size='xx-large')
#fl = figlegend(angle_rects, translation_rects), ('Angle error (radians)', 'Translation error (m)'), loc=(0.0, 0.45))

pad = 0.1
bbi = 'tight'
bbea = [txt1]
savefig('bargraph.pdf', bbox_inches=bbi, pad_inches=pad, bbox_extra_artists=bbea)
savefig('bargraph.eps', bbox_inches=bbi, pad_inches=pad, bbox_extra_artists=bbea)
savefig('bargraph.png', bbox_inches=bbi, pad_inches=pad, bbox_extra_artists=bbea)

