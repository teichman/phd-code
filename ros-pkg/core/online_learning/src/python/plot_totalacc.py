#!/usr/bin/python

import sys
import os
import argparse
import numpy as np
import matplotlib.pyplot as plt
import common_plotting as cp

parser = argparse.ArgumentParser()
parser.add_argument('--type', help='Show annotations or inductions.  "ann" or "ind", default "ann"', default='ann')
parser.add_argument('--out', help='Output filename base, no extension.', default='plot')

args = parser.parse_args()
assert(args.type == 'ann' or args.type == 'ind')
if args.type == 'ann':
    print 'Showing annotations'
else:
    print 'Showing inductions'
basename = args.out + '-' + args.type + '-total'
print 'Saving to ' + basename

# -- Config
cp.setup()

# -- Load data
tmpfile = ".python-astoehusathoesutahoneuth"

# -- Load global statistics.
os.system("grep 'Total acc' `find -name 'track_results.txt' | grep -v annotated | sort` | awk '{print $NF}' > " + tmpfile)
accuracies = np.loadtxt(tmpfile)
os.system("grep 'Total acc' `find -name 'track_results.txt' | grep -v annotated | sort` | awk -F/ '{print $2}' | grep -o '[0-9]*' > " + tmpfile)
accuracies_idx = np.loadtxt(tmpfile)
os.system("grep elapsed $(find -name track_results.txt | grep -v annotated | sort | awk -F/ '{print $2 \"/learner_status.txt\"}') | awk '{print $NF}' > " + tmpfile)
accuracies_times = np.loadtxt(tmpfile)
os.system("grep 'Total unlabeled frames' $(find -name track_results.txt | grep -v annotated | sort | awk -F/ '{print $2 \"/learner_status.txt\"}') | awk '{print $NF}' > " + tmpfile)
eval_num_unl = np.loadtxt(tmpfile)
os.system("grep 'Total unlabeled frames' `find -name 'learner_status.txt' | sort` | awk '{print $NF}' > " + tmpfile)
num_unl = np.loadtxt(tmpfile)
os.system("grep 'Total tracks annotated' `find -name 'learner_status.txt' | sort` | awk '{print $NF}' > " + tmpfile)
num_ann_cum = np.loadtxt(tmpfile)
#num_ann = np.diff(np.insert(num_ann_cum, 0, 0))
num_ann = np.append(np.diff(num_ann_cum), 0)
os.system("grep 'Total elapsed' `find -name 'learner_status.txt' | sort` | awk '{print $NF}' > " + tmpfile)
times = np.loadtxt(tmpfile)
#os.system("grep 'Total frames inducted' `find -name 'learner_status.txt' | sort` | awk '{print $NF}' > " + tmpfile)
#num_inducted_cum = np.loadtxt(tmpfile)
#num_inducted = np.append(np.diff(num_inducted_cum), 0)
os.system("rm " + tmpfile)

# -- Plot number of annotated or inducted.
fig = plt.figure(figsize=(13, 5))
width = num_unl[-1] / 100.
ax1 = fig.add_subplot(111)
plot1 = 0
if args.type == 'ann': 
    plot1 = ax1.bar(num_unl, num_ann, width, color=(0.4, 0.4, 0.4), label='Num hand-labeled')
    ax1.set_ylabel('Number of hand-labeled tracks')
#else:
#    plot1 = ax1.bar(num_unl, num_inducted, width, color=(0.4, 0.4, 0.4), label='Num inducted')
#    ax1.set_ylabel('Number of inducted frames')
ymin, ymax = plt.ylim()
plt.ylim(ymin, ymax * 3)
plt.grid(True)

# -- Plot accuracy or PR
ax2 = ax1.twinx()
ax2.set_xlabel('Number unlabeled frames looked at')
plot2 = ax2.plot(eval_num_unl, accuracies, cp.linestyles[1], label='Accuracy')
ax2.set_ylabel('Accuracy')

ymin, ymax = plt.ylim()
plt.ylim(ymin, 1.0)

plt.savefig(basename + '.pdf')
plt.savefig(basename + '.png')



