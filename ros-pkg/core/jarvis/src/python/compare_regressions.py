#!/usr/bin/python

import sys
import os
import argparse
import itertools
import numpy as np
import matplotlib.pyplot as plt

if len(sys.argv) != 3:
    print 'Usage: compare_regressions.py PRE POST'
    print '  where PRE and POST are directories containing regression runs.'
    exit(0)

pre_dir = sys.argv[1]
post_dir = sys.argv[2]

pre_test_names = sorted(os.walk(pre_dir).next()[1])
post_test_names = sorted(os.walk(post_dir).next()[1])

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

print "============================================================"
print "Overall results"
improvement = np.mean(np.array(gi_accs_all)) - np.mean(np.array(baseline_accs_all))
print 'Baseline mean accuracy: ' + str(np.mean(baseline_accs_all))
print 'Group induction mean accuracy: ' + str(np.mean(gi_accs_all))
print 'Improvement: ' + str(improvement)
pvalue = stratifiedPermutationTest(baseline_accs_all, gi_accs_all)
print 'P < ' + str(pvalue)


tmpfile = '.python-asotnehusnatoheuntahoeu'

print "PRE: " + pre_dir
os.system("grep 'Total acc' `find " + pre_dir + " -name final_track_results.txt | sort` | awk '{print $NF}' > " + tmpfile)
pre_acc = np.loadtxt(tmpfile)
print "Mean: " + str(np.mean(pre_acc))
print pre_acc

print "POST: " + post_dir
os.system("grep 'Total acc' `find " + post_dir + " -name final_track_results.txt | sort` | awk '{print $NF}' > " + tmpfile)
post_acc = np.loadtxt(tmpfile)
print "Mean: " + str(np.mean(post_acc))
print post_acc

improvement = np.mean(post_acc) - np.mean(pre_acc)
print "Improvement: " + str(improvement)
all = np.concatenate((pre_acc, post_acc))

#print "permutations"
num_perm_better = 0.
num_samples = 1000
for _ in range(num_samples):
    perm = np.random.permutation(all)
    pre = perm[0:len(pre_acc)]
    post = perm[len(pre_acc):]
    improvement_perm = np.mean(post) - np.mean(pre)
#    print "Improvement: " + str(improvement_perm)
    if improvement_perm >= improvement:
        num_perm_better += 1

p = num_perm_better / num_samples
print "p: " + str(p)
    
    
# Hah, there's a lot.  n!.
#for permutation in itertools.permutations(all):
#    print permutation


