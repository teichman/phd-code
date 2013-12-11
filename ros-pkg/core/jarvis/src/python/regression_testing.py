#!/usr/bin/python

import os
import tempfile
import numpy as np

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
