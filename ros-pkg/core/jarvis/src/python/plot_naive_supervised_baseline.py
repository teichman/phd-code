#!/usr/bin/python

# Run this script inside a regression test dir.

import sys
import argparse
import itertools
import numpy as np
import matplotlib.pyplot as plt
import math

import roslib
roslib.load_manifest('jarvis')
import common_plotting as cp
from regression_testing import * 

def getPlotTitle():
    return os.path.basename(os.getcwd())[:-2].capitalize().replace("_", " ")

parser = argparse.ArgumentParser()
parser.add_argument("-v", "--print-vals", action='store_true')
parser.add_argument("-o", "--output-name", help="output filename (without extension)", type=str, default='nsb')
args = parser.parse_args()

# -- Config
cp.setup()

# -- Read in the pairs of (num_tr_ex, acc).
subsample_names = loadBash("find -type d -name 'subsample*' | sort | awk -F'/' '{print $NF}' | sort | uniq")
print subsample_names

means = np.zeros((len(subsample_names)))
stderrs = np.zeros((len(subsample_names)))
ntrs = np.zeros((len(subsample_names)))

for idx, subsample_name in enumerate(subsample_names):
    accs = 100 * npLoadBash("grep 'Total acc' `find -name track_results.txt | grep naive_supervised_baseline | grep quantity | grep " + subsample_name + " | sort` | awk '{print $NF}'")
    ntr = npLoadBash("grep -h '[0-9] tracks\.' `find -name training_stats.txt | grep " + subsample_name + " | sort` | awk '{print $1}' | sort | uniq")
    assert(ntr.size == 1)  # All runs in each subsample should have the same number of tracks.

    means[idx] = np.mean(accs)
    stderrs[idx] = np.std(accs) / math.sqrt(len(accs))
    ntrs[idx] = ntr


# -- Read in the accuracies of group induction.
gi_accs = 100 * npLoadBash("grep 'Total acc' `find -L -wholename '*/induction/final_track_results.txt' | sort` | awk '{print $NF}'")
#gi_ntrs = npLoadBash("for dir in `find -L -maxdepth 1 -mindepth 1 -type d`; do grep -A2 'Hand-annotated' `find $dir -wholename '*/induction/*/learner_status.txt' | sort | tail -n1`; done | grep tracks | awk '{print $1}'")
gi_ntrs = npLoadBash("for dir in `find -name induction | sort`; do echo $dir; grep -A2 'Hand-annotated' `find $dir -name 'learner_status.txt' | sort | tail -n1`; done | grep tracks | awk '{print $1}'")

print gi_accs
print gi_ntrs

gi_acc_mean = np.mean(gi_accs)
gi_acc_stderr = np.std(gi_accs) / math.sqrt(len(gi_accs))
gi_ntr_mean = np.mean(gi_ntrs)
gi_ntr_stderr = np.std(gi_ntrs) / math.sqrt(len(gi_ntrs))

# -- Make the plot.
fig, ax = plt.subplots()
plt.errorbar(ntrs, means, yerr=stderrs, fmt='-o', color='black', markersize=5, linewidth=1, linestyle=':', label='Supervised baseline')
# Plot group induction results as a single point with error bars in both directions.
#ax.errorbar(gi_ntr_mean, gi_acc_mean, yerr=gi_acc_stderr, xerr=gi_ntr_stderr, fmt='', color='red', elinewidth=2, capsize=3, label='Group induction')
# Plot all group induction results individually.
ax.plot(gi_ntrs, gi_accs, 'x', color='red', markersize=6, markeredgewidth=2, label='Group induction')

plt.xlabel('Number of supervised training examples')
plt.ylabel('Accuracy (\%)')
plt.title(getPlotTitle())

plt.legend(loc='lower right')

plt.savefig(args.output_name + '-' + os.path.basename(os.getcwd()) + '.pdf')
plt.savefig(args.output_name + '-' + os.path.basename(os.getcwd()) + '.png')


