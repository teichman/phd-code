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
os.system("grep Angle ../evaluations/configuration*.txt | awk '{print $NF}' > " + tmpfile)
angles = np.loadtxt(tmpfile)
os.system("grep Euclidean ../evaluations/configuration*.txt | awk '{print $NF}' > " + tmpfile)
translations = np.loadtxt(tmpfile);
os.system("grep 'Sync error' ../evaluations/configuration*.txt | awk '{print $NF}' > " + tmpfile)
syncs = np.loadtxt(tmpfile);

# -- Get overall performances
num_examples = np.prod(np.size(angles))
if num_examples <= 1:
    angles = np.array([np.asscalar(angles)])
    translations = np.array([np.asscalar(translations)])
    syncs = np.array([np.asscalar(syncs)])
average_angle = np.sum(angles) / num_examples
average_translation = np.sum(translations) / num_examples
average_sync = np.sum(syncs) / num_examples

# Save tex of the table while we're at it
std_trans = np.std(translations)
std_angle = np.std(angles)
std_sync = np.std(syncs)
texfile = open('results_table.tex','w')
texfile.write('\\begin{tabular}{ l | r | r | r }\n')
texfile.write('Sequence & Translation & Rotation & Sync \\\ \n')
texfile.write('\\hline\n')
for i in range(num_examples):
    texfile.write('%.2d & %.2f & %.2f & %.2f \\\ \n'%(i+1, translations[i], angles[i], syncs[i]))
texfile.write('\\hline\n')
texfile.write('Avg & %.2f$\pm$%.2f & %.2f$\pm$%.2f & %.2f$\pm$%.2f \\\ \n'%(average_translation, std_trans, average_angle,std_angle, average_sync, std_sync))
texfile.write('\end{tabular}\n')
texfile.close()


angles = hstack([angles, 0, 0, average_angle, 0])
translations = hstack([translations, 0, 0, average_translation, 0])
syncs = hstack([syncs, 0, 0, average_sync, 0])

print "Angles"
print angles
print "Translations"
print translations
print "Syncs"
print syncs

# -- Set up figure
indices = np.arange(len(angles))
width = 0.35
fig = figure(figsize=(15, 5))

# -- Angles
ax1 = plt.subplot2grid((3, 5), (0, 1), colspan=4)
for loc, spine in ax1.spines.iteritems():
    if loc in ['right','top']:
        spine.set_color('none')
ax1.yaxis.set_ticks_position('left')
ax1.xaxis.set_ticks([])
angle_rects = ax1.bar(indices, angles, width, color='g')
ax1.yaxis.set_ticks([ax1.yaxis.get_majorticklocs()[0], ax1.yaxis.get_majorticklocs()[-1]])

# -- Translations
ax2 = plt.subplot2grid((3, 5), (1, 1), colspan=4)
for loc, spine in ax2.spines.iteritems():
    if loc in ['right','top']:
        spine.set_color('none')
ax2.yaxis.set_ticks_position('left')
ax2.xaxis.set_ticks([])
translation_rects = ax2.bar(indices, translations, width, color='g')
ax2.yaxis.set_ticks([ax2.yaxis.get_majorticklocs()[0], ax2.yaxis.get_majorticklocs()[-1]])

# -- Syncs
ax3 = plt.subplot2grid((3, 5), (2, 1), colspan=4)
ax3.xaxis.set_ticks_position('bottom')
ax3.xaxis.set_ticklabels([str(x) for x in range(1, len(indices) - 3)] + ['', '', 'Overall', ''])
#ax3.xaxis.set_ticks(indices + width)
ax3.xaxis.set_ticks(indices + width/2.0)
for loc, spine in ax3.spines.iteritems():
    if loc in ['right','top']:
        spine.set_color('none')
translation_rects = ax3.bar(indices, syncs, width, color='g')
ax3.yaxis.set_ticks_position('left')
ax3.yaxis.set_ticks([ax3.yaxis.get_majorticklocs()[0], ax3.yaxis.get_majorticklocs()[-1]])

# -- Annotate and save
xlabel('Sequence Number')
txt1 = figtext(0.1, 0.75, "Angle error\n(radians)", size='large')
txt2 = figtext(0.1, 0.5, "Translation error\n(m)", size='large')
txt3 = figtext(0.1, 0.25, "Sync error\n(s)", size='large')
#fl = figlegend(angle_rects, translation_rects), ('Angle error (radians)', 'Translation error (m)'), loc=(0.0, 0.45))

pad = 0.1
bbi = 'tight'
bbea = [txt1]
savefig('bargraph.pdf', bbox_inches=bbi, pad_inches=pad, bbox_extra_artists=bbea)
savefig('bargraph.eps', bbox_inches=bbi, pad_inches=pad, bbox_extra_artists=bbea)
savefig('bargraph.png', bbox_inches=bbi, pad_inches=pad, bbox_extra_artists=bbea)


