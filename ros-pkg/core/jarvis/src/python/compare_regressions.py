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


