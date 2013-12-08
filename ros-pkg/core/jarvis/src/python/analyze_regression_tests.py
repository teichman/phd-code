#!/usr/bin/python

import sys
import os
import argparse
import itertools
import tempfile
import numpy as np
import matplotlib.pyplot as plt


def npLoadBash(command):
    tmpfile = '.python-asotuesntahoensuthnat'  # TODO: Use tempfile library.
    os.system(command + " > " + tmpfile)
    array = np.loadtxt(tmpfile)
    os.system('rm ' + tmpfile)
    return array

if len(sys.argv) == 1:
    print 'Usage: ' + sys.argv[0] + ' DIR [ DIR ... ]'
    print '  where DIRS are the run directories.'
    exit(0)


dirs = sys.argv[1:]

gi_accs = npLoadBash("grep 'Total acc' `find -name 'final_track_results.txt' | sort` | awk '{print $NF}'")
baseline_accs = npLoadBash("grep 'Total acc' `find -wholename '*iter009/track_results.txt' | sort` | awk '{print $NF}'")

print 'Baseline mean accuracy: ' + str(np.mean(baseline_accs))
print 'Group induction mean accuracy: ' + str(np.mean(gi_accs))
improvement = np.mean(gi_accs) - np.mean(baseline_accs)
print 'Improvement: ' + str(improvement)

all = np.concatenate((gi_accs, baseline_accs))

num_perm_better = 0.
num_samples = 100000
for _ in range(num_samples):
    perm = np.random.permutation(all)
    gi = perm[0:len(gi_accs)]
    baseline = perm[len(baseline_accs):]
    improvement_perm = np.mean(gi) - np.mean(baseline)
    if improvement_perm >= improvement:
        num_perm_better += 1

p = num_perm_better / num_samples
print "p: " + str(p)


