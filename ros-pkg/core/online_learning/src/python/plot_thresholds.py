#!/usr/bin/python

import sys
import os
import argparse
import numpy as np
import matplotlib.pyplot as plt
import common_plotting as cp

parser = argparse.ArgumentParser()
parser.add_argument('cl', help='Which class to show for.')
parser.add_argument('--out', help='Output filename base, no extension.', default='thresholds')

args = parser.parse_args()
print 'Displaying results for class: ' + args.cl
basename = args.out + '-' + args.cl
print 'Saving to ' + basename

# -- TODO: Handle this more gracefully.
cid = -1
if args.cl == 'car':
    cid = 0
elif args.cl == 'pedestrian':
    cid = 1
elif args.cl == 'bicyclist':
    cid = 2
        
# -- Config
cp.setup()

# -- Load data
tmpfile = '.python-astoehusnatohesutnhaosenu'
os.system("grep 'Total unlabeled frames' `find -name 'learner_status.txt' | sort` | awk '{print $NF}' > " + tmpfile)
num_unl = np.loadtxt(tmpfile)
os.system("grep 'Total unlabeled frames' $(find -name track_results.txt | sort | awk -F/ '{print $2 \"/learner_status.txt\"}') | awk '{print $NF}' > " + tmpfile)
eval_num_unl = np.loadtxt(tmpfile)

os.system("grep 'Num tracks annotated as " + args.cl + "' `find -name 'learner_status.txt' | sort` | awk '{print $NF}' > " + tmpfile)
num_ann_neg_cum = np.loadtxt(tmpfile)
num_ann_neg = np.diff(np.insert(num_ann_neg_cum, 0, 0))
os.system("grep 'Num tracks annotated as " + args.cl + "' `find -name 'learner_status.txt' | sort` | awk '{print $(NF-1)}' > " + tmpfile)
num_ann_pos_cum = np.loadtxt(tmpfile)
num_ann_pos = np.diff(np.insert(num_ann_pos_cum, 0, 0))

os.system("grep 'Upper .* threshold' -A1 `find -name 'learner_status.txt' | sort` | grep -v ind | grep -v '\-\-' | awk '{print $" + str(cid+2) + "}' > " + tmpfile)
upper = np.loadtxt(tmpfile)
os.system("grep 'Lower .* threshold' -A1 `find -name 'learner_status.txt' | sort` | grep -v ind | grep -v '\-\-' | awk '{print $" + str(cid+2) + "}' > " + tmpfile)
lower = np.loadtxt(tmpfile)

os.system("rm " + tmpfile)

# -- Clean out infs.
upper[upper > 1e20] = 0
lower[lower < -1e20] = 0

# -- Plot 
fig = plt.figure(figsize=(13, 5))
plt.grid(True)
ax1 = fig.add_subplot(111)  # ax1 = fig.add_subplot(1, 1, 1) fails comically
plt.plot(num_unl, upper, label='Upper threshold')
plt.plot(num_unl, lower, label='Lower threshold')
ax1.set_ylabel('Threshold')

ax2 = ax1.twinx()
width = num_unl[-1] / 50.
ax2.bar(num_unl, num_ann_neg, width, color=(0.85, 0.85, 0.85))
ax2.bar(num_unl, num_ann_pos, width, color=(0, 0.3, 0), bottom=num_ann_neg)
ax2.set_ylabel('Number of hand-labeled tracks')
ymin, ymax = plt.ylim()
plt.ylim(ymin, ymax * 2)

plt.xlabel('Number unlabeled frames looked at')
plt.savefig(basename + '.pdf')
plt.savefig(basename + '.png')



