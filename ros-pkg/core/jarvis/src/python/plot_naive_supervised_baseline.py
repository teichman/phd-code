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
#ntrs = npLoadBash("grep -h '[0-9] tracks\.' `find -name training_stats.txt | grep naive_supervised_baseline | sort` | awk '{print $1}'")
#accs = npLoadBash("grep 'Total acc' `find -name track_results.txt | grep naive_supervised_baseline | grep quantity_experiment | sort` | awk '{print $NF}'")

means = np.zeros((len(subsample_names)))
stderrs = np.zeros((len(subsample_names)))
ntrs = np.zeros((len(subsample_names)))

for idx, subsample_name in enumerate(subsample_names):
    accs = npLoadBash("grep 'Total acc' `find -name track_results.txt | grep naive_supervised_baseline | grep quantity | grep " + subsample_name + " | sort` | awk '{print $NF}'")
    ntr = npLoadBash("grep -h '[0-9] tracks\.' `find -name training_stats.txt | grep " + subsample_name + " | sort` | awk '{print $1}' | sort | uniq")
    assert(ntr.size == 1)  # All runs in each subsample should have the same number of tracks.

    means[idx] = np.mean(accs)
    stderrs[idx] = np.std(accs) / math.sqrt(len(accs))
    ntrs[idx] = ntr


# -- Read in the accuracies of group induction.
gi_accs = npLoadBash("grep 'Total acc' `find -L -wholename '*/induction/final_track_results.txt' | sort` | awk '{print $NF}'")
gi_ntrs = npLoadBash("for dir in `find -L -maxdepth 1 -mindepth 1 -type d`; do grep -A2 'Hand-annotated' `find $dir -wholename '*/induction/*/learner_status.txt' | sort | tail -n1`; done | grep tracks | awk '{print $1}'")

print gi_accs
print gi_ntrs

gi_mean = np.mean(gi_accs)
gi_stderr = np.std(gi_accs) / math.sqrt(len(gi_accs))
gi_ntr = np.mean(gi_ntrs)

# -- Make the plot.
fig, ax = plt.subplots()
ax.errorbar(ntrs, means, yerr=stderrs, fmt='-o', color='black', markersize=5, linewidth=1, linestyle=':', label='Supervised baseline')
ax.plot(gi_ntrs, gi_accs, 'x', color='red', markersize=6, markeredgewidth=2, label='Group induction')

plt.xlabel('Number of supervised training examples')
plt.ylabel('Accuracy')
plt.title(getPlotTitle())

plt.legend(loc='lower right')

plt.savefig(args.output_name + '.pdf')
plt.savefig(args.output_name + '.png')

