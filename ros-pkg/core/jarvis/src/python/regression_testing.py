#!/usr/bin/python

import os
import tempfile
import numpy as np
import random

def npLoadBash(command):
    tmpfile = '.python-asotuesntahoensuthnat'  # TODO: Use tempfile library.
    os.system(command + " > " + tmpfile)
    array = np.loadtxt(tmpfile)
    os.system('rm ' + tmpfile)
    return array

# pre and post are lists of np arrays.  Swaps can occur within matched pre / post arrays,
# but not between unmatched arrays.
def stratifiedPermutationTest(pre, post, num_samples = 10000):
    assert(len(pre) == len(post))  # Number of strata must match.
    mean_change = np.mean(np.concatenate(post)) - np.mean(np.concatenate(pre))
    sign = 1.0
    if mean_change == 0:
        return (0, 0.5)
    elif mean_change < 0:
        sign = -1.0
    
    num_more_extreme = 0.
    for _ in range(num_samples):

        # Generate a scrambled version of pre and post.
        sample_pre = list(pre)  # Copy the list.
        sample_post = list(post)
        for i in range(len(sample_pre)):
            permutation = np.random.permutation(np.concatenate([sample_pre[i], sample_post[i]]))
            sample_pre[i] = permutation[:len(pre[i])]
            sample_post[i] = permutation[len(pre[i]):]
            assert(len(sample_pre[i]) == len(pre[i]))
            assert(len(sample_post[i]) == len(post[i]))

        # Increment if more extreme than the actual change.
        sample_change = np.mean(np.concatenate(sample_post)) - np.mean(np.concatenate(sample_pre))
        if sample_change * sign >= mean_change * sign:
            num_more_extreme += 1

    p = num_more_extreme / num_samples
    return (mean_change, p)

# pre and post are np arrays.
# Returns (change, p value).
# Two-tailed test.  The p value is the probability that, if there
# was no difference between pre and post, you'd see a change at
# least as extreme *in the direction of the observed change* at random.
# A change of exactly zero is not well-defined.
def swapTest(pre, post, num_samples = 10000):
    assert(pre.size == post.size)
    mean_change = np.mean(post) - np.mean(pre)

    sign = 1.0
    if mean_change == 0:
        return (0, 0.5)
    elif mean_change < 0:
        sign = -1.0

    sample_pre = list(pre)  # Copy the list.
    sample_post = list(post)
    num_more_extreme = 0.
    for _ in range(num_samples):
        for i in range(pre.size):
            if random.randint(0, 1) == 0:
                tmp = sample_pre[i]
                sample_pre[i] = sample_post[i]
                sample_post[i] = tmp
        sample_change = np.mean(sample_post) - np.mean(sample_pre)
        if sample_change * sign >= mean_change * sign:
            num_more_extreme += 1

    p = num_more_extreme / num_samples
    return (mean_change, p)

def splitNum(num, signed = False):
    numstr = '%.2f' % num
    if(signed):
        numstr = '%+.2f' % num
    strs = numstr.split('.')
    whole = strs[0]
    decimal = ''
    if len(strs) == 2:
        decimal = strs[1]
    return(whole, decimal)

# pre and post are lists of np arrays.
# If paired == True, then use pairwise swapping only in the statistical significance test.
def analyze(pre, post, pre_name = 'Pre', post_name = 'Post', test_names = [], quantity_name = 'quantity', num_samples = 10000, paired = False):
    if len(test_names) == 0:
        test_names = ["Test%03d" % i for i in range(len(pre))]

    assert(len(pre) == len(post))
    assert(len(test_names) == len(pre))

    print '================================================================================'
    print 'Regression test: ' + quantity_name
    print
    print '{0:20s}  {1:12s} {2:12s}  {3:12s} {4:12s}'.format('Test', pre_name, post_name, 'Change', 'Significance')
    print "--------------------------------------------------------------------------------"
    for (idx, test_name) in enumerate(test_names):
        if paired:
            (change, p) = swapTest(pre[idx], post[idx], num_samples)
        else:
            (change, p) = stratifiedPermutationTest([pre[idx]], [post[idx]], num_samples)
        
        print "{name:16s} {pre_mean_vals[0]:>6}.{pre_mean_vals[1]:<6} {post_mean_vals[0]:>6}.{post_mean_vals[1]:<6} {change_vals[0]:>6}.{change_vals[1]:<6}    p < {p:<10.3f}".format(name=test_name, pre_mean_vals=splitNum(np.mean(pre[idx])), post_mean_vals=splitNum(np.mean(post[idx])), change_vals=splitNum(change, True), p=p)


    print "--------------------------------------------------------------------------------"
    if paired:
        (change, p) = swapTest(np.concatenate(pre), np.concatenate(post), num_samples)
    else:
        (change, p) = stratifiedPermutationTest(pre, post, num_samples)
    pre_mean = np.mean(np.concatenate(pre))
    post_mean = np.mean(np.concatenate(post))
    print "{name:16s} {pre_mean_vals[0]:>6}.{pre_mean_vals[1]:<6} {post_mean_vals[0]:>6}.{post_mean_vals[1]:<6} {change_vals[0]:>6}.{change_vals[1]:<6}    p < {p:<10.3f}".format(name='all', pre_mean_vals=splitNum(pre_mean), post_mean_vals=splitNum(post_mean), change_vals=splitNum(change, True), p=p)

    print

