#!/usr/bin/python

import sys
import os
import argparse
import numpy as np
import matplotlib.pyplot as plt
import common_plotting as cp

parser = argparse.ArgumentParser()
parser.add_argument('cl', help='Which class to show for.')
parser.add_argument('--type', help='Show annotations or inductions.  "ann" or "ind", default "ann"', default='ann')
parser.add_argument('--out', help='Output filename base, no extension.', default='plot')

args = parser.parse_args()
assert(args.type == 'ann' or args.type == 'ind')
print 'Displaying results for class: ' + args.cl
if args.type == 'ann':
    print 'Showing annotations'
else:
    print 'Showing inductions'
basename = args.out + '-' + args.type + '-' + args.cl + '-pr'
print 'Saving to ' + basename

# -- Config
cp.setup()

# -- Load data
tmpfile = '.python-astoehusathoesutahoneuth'

# -- Load global statistics.
os.system("grep 'Total acc' `find -name 'track_results.txt' | grep test_results | sort` | awk '{print $NF}' > " + tmpfile)
accuracies = np.loadtxt(tmpfile)
os.system("grep 'Total acc' `find -name 'track_results.txt' | grep test_results | sort` | awk -F/ '{print $2}' | grep -o '[0-9]*' > " + tmpfile)
accuracies_idx = np.loadtxt(tmpfile)
os.system("grep elapsed $(find -name track_results.txt | grep test_results | sort | awk -F/ '{print $2 \"/learner_status.txt\"}') | awk '{print $NF}' > " + tmpfile)
accuracies_times = np.loadtxt(tmpfile)
os.system("grep 'Total unlabeled frames' $(find -name track_results.txt | grep test_results | sort | awk -F/ '{print $2 \"/learner_status.txt\"}') | awk '{print $NF}' > " + tmpfile)
eval_num_unl = np.loadtxt(tmpfile)
os.system("grep 'Total unlabeled frames' `find -name 'learner_status.txt' | sort` | awk '{print $NF}' > " + tmpfile)
num_unl = np.loadtxt(tmpfile)
os.system("grep 'Total tracks annotated' `find -name 'learner_status.txt' | sort` | awk '{print $NF}' > " + tmpfile)
num_ann_cum = np.loadtxt(tmpfile)
num_ann = np.diff(np.insert(num_ann_cum, 0, 0))
os.system("grep 'Total elapsed' `find -name 'learner_status.txt' | sort` | awk '{print $NF}' > " + tmpfile)
times = np.loadtxt(tmpfile)

# -- Load class-only statistics
os.system("grep -A12 'Class: " + args.cl + "' `find -name 'track_results.txt' | grep test_results | sort` | grep 'Per-class recall' | awk '{print $NF}' > " + tmpfile)
cl_recall = np.loadtxt(tmpfile)
os.system("grep -A12 'Class: " + args.cl + "' `find -name 'track_results.txt' | grep test_results | sort` | grep 'Per-class precision' | awk '{print $NF}' > " + tmpfile)
cl_precision = np.loadtxt(tmpfile)
os.system("grep -A10 'Class: " + args.cl + "' `find -name 'track_results.txt' | grep test_results | sort` | grep Accuracy | awk '{print $NF}' > " + tmpfile)
cl_accuracy = np.loadtxt(tmpfile)

os.system("grep 'Num tracks annotated as " + args.cl + "' `find -name 'learner_status.txt' | sort` | awk '{print $NF}' > " + tmpfile)
cl_num_ann_neg_cum = np.loadtxt(tmpfile)
cl_num_ann_neg = np.append(np.diff(cl_num_ann_neg_cum), 0)
os.system("grep 'Num tracks annotated as " + args.cl + "' `find -name 'learner_status.txt' | sort` | awk '{print $(NF-1)}' > " + tmpfile)
cl_num_ann_pos_cum = np.loadtxt(tmpfile)
cl_num_ann_pos = np.diff(cl_num_ann_pos_cum)
cl_num_ann_pos = np.append(cl_num_ann_pos, 0)

os.system("rm " + tmpfile)

# -- Plot number of annotated or inducted.
fig = plt.figure(figsize=(13, 5))
plt.grid(True)
width = num_unl[-1] / 100.
ax1 = fig.add_subplot(111)
plot1 = 0

if args.type == 'ann': 
    ax1.bar(num_unl, cl_num_ann_neg, width, color=(0.85, 0.85, 0.85), zorder=9)
    ax1.bar(num_unl, cl_num_ann_pos, width, color=(0, 0.3, 0), bottom=cl_num_ann_neg, zorder=10)
    ax1.set_ylabel('Number of hand-labeled tracks')
ymin, ymax = plt.ylim()
plt.ylim(ymin, ymax * 3)
plt.xlabel('Number unlabeled frames looked at')
# -- Plot accuracy or PR
ax2 = ax1.twinx()

plot3 = ax2.plot(eval_num_unl + width, cl_precision, 'kh:', label='Precision', zorder=1)
plot4 = ax2.plot(eval_num_unl + width, cl_recall, 'bs:', label='Recall', zorder=1)
ax2.set_ylabel('Precision and recall')

ymin, ymax = plt.ylim()
plt.ylim(ymin, 1.0)

plt.legend(loc='center right')

plt.savefig(basename + '.pdf')
plt.savefig(basename + '.png')



