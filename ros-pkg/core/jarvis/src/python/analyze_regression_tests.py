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

# pre and post are np arrays.
def permutationTest(pre, post):
    improvement = np.mean(post) - np.mean(pre)
    all = np.concatenate((pre, post))
    num_perm_better = 0.
    num_samples = 10000
    for _ in range(num_samples):
        perm = np.random.permutation(all)
        sample_post = perm[0:len(post)]
        sample_pre = perm[len(post):]
        sample_improvement = np.mean(sample_post) - np.mean(sample_pre)
        if sample_improvement >= improvement:
            num_perm_better += 1

    p = num_perm_better / num_samples
    return p

# pre and post are lists of np arrays.
def stratifiedPermutationTest(pre, post):
    improvement = np.mean(np.array(post)) - np.mean(np.array(pre))
    
    num_perm_better = 0.
    num_samples = 10000
    for _ in range(num_samples):

        # Generate a scrambled version of pre and post.
        sample_pre = []
        sample_post = []
        for (idx, __) in enumerate(pre):
            all = np.concatenate((pre[idx], post[idx]))
            perm = np.random.permutation(all)
            sample_post.append(perm[0:len(post[idx])])
            sample_pre.append(perm[len(post[idx]):])

        # Increment if better than the actual version.
        sample_improvement = np.mean(np.array(sample_post)) - np.mean(np.array(sample_pre))
        if sample_improvement >= improvement:
            num_perm_better += 1

    p = num_perm_better / num_samples
    return p
    

if len(sys.argv) != 2:
    print 'Usage: ' + sys.argv[0] + ' DIR'
    print '  where DIR is the output directory for a full suite of regression tests.'
    exit(0)


output_dir = sys.argv[1]
test_names = os.walk('.').next()[1]

gi_accs_all = []
baseline_accs_all = []
for test_name in test_names:
    print "============================================================"
    print "Test: " + test_name
    gi_accs = npLoadBash("grep 'Total acc' `find " + os.path.join(output_dir, test_name) + " -name 'final_track_results.txt' | sort` | awk '{print $NF}'")
    gi_accs_all.append(gi_accs)
    baseline_accs = npLoadBash("grep 'Total acc' `find " + os.path.join(output_dir, test_name) + " -wholename '*iter009/track_results.txt' | sort` | awk '{print $NF}'")
    baseline_accs_all.append(baseline_accs)
    print 'Baseline mean accuracy: ' + str(np.mean(baseline_accs))
    print 'Group induction mean accuracy: ' + str(np.mean(gi_accs))
    improvement = np.mean(gi_accs) - np.mean(baseline_accs)
    print 'Improvement: ' + str(improvement)
    pvalue = permutationTest(baseline_accs, gi_accs)
    print 'P < ' + str(pvalue)
    print
    print gi_accs - baseline_accs

    
#    pvalue2 = stratifiedPermutationTest([baseline_accs], [gi_accs])
#    print 'P < ' + str(pvalue2)


print "============================================================"
print "Overall results"
improvement = np.mean(np.array(gi_accs_all)) - np.mean(np.array(baseline_accs_all))
print 'Baseline mean accuracy: ' + str(np.mean(baseline_accs_all))
print 'Group induction mean accuracy: ' + str(np.mean(gi_accs_all))
print 'Improvement: ' + str(improvement)
pvalue = stratifiedPermutationTest(baseline_accs_all, gi_accs_all)
print 'P < ' + str(pvalue)
