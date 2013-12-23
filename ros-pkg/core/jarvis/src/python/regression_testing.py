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

# pre and post are np arrays.
def permutationTest(pre, post, num_samples = 10000):
    assert(pre.size == post.size)
    improvement = np.mean(post) - np.mean(pre)

    sample_pre = list(pre)  # Copy the list.
    sample_post = list(post)
    assert(len(sample_pre) == len(sample_post))
    num_perm_better = 0.
    for _ in range(num_samples):
        for (i, _) in enumerate(sample_pre):
            if random.randint(0, 1) == 0:
                tmp = sample_pre[i]
                sample_pre[i] = sample_post[i]
                sample_post[i] = tmp
        sample_improvement = np.mean(sample_post) - np.mean(sample_pre)
        if sample_improvement >= improvement:
            num_perm_better += 1

    p = num_perm_better / num_samples
    return p

# pre and post are lists of np arrays.  Swaps can occur within matched pre / post arrays,
# but not between unmatched arrays.
def stratifiedPermutationTest(pre, post, num_samples = 10000):
    improvement = np.mean(np.array(post)) - np.mean(np.array(pre))
    
    num_perm_better = 0.
    for _ in range(num_samples):

        # Generate a scrambled version of pre and post.
        sample_pre = list(pre)  # Copy the list.
        sample_post = list(post)
        for (i, __) in enumerate(sample_pre):
            assert(len(sample_pre[i]) == len(sample_post[i]))
            for (j, ___) in enumerate(sample_pre[i]):
                if random.randint(0, 1) == 0:
                    tmp = sample_pre[i][j]
                    sample_pre[i][j] = sample_post[i][j]
                    sample_post[i][j] = tmp

        # Increment if better than the actual version.
        sample_improvement = np.mean(np.array(sample_post)) - np.mean(np.array(sample_pre))
        if sample_improvement >= improvement:
            num_perm_better += 1

    p = num_perm_better / num_samples
    return p


# pre_scores and post_scores are lists of np arrays.
def compareTests(pre_scores, post_scores, pre_name = 'Pre', post_name = 'Post', test_names = [], score_name = 'score', num_samples = 10000):
    if len(test_names) == 0:
        test_names = ["%03d" % i for i in range(len(pre_scores))]

    assert(len(pre_scores) == len(post_scores))
    assert(len(test_names) == len(pre_scores))
    
    for (idx, test_name) in enumerate(test_names):
        print "================================================================================"
        print "Test: " + test_name
        print pre_name + ' mean ' + score_name + ': ' + str(np.mean(pre_scores[idx]))
        print post_name + ' mean ' + score_name + ': ' + str(np.mean(post_scores[idx]))
        improvement = np.mean(post_scores[idx]) - np.mean(pre_scores[idx])
        print 'Improvement: ' + str(improvement)
        if score_name == 'accuracy':
            error_reduction = (1 - (1 - np.mean(post_scores[idx])) / (1 - np.mean(pre_scores[idx]))) * 100
            print 'Error reduction: %3.1f%%' % (error_reduction)
        print pre_scores[idx]
        print post_scores[idx]
        pvalue = permutationTest(pre_scores[idx], post_scores[idx], num_samples)
        print 'P < ' + str(pvalue)
        print
        print score_name + " changes:"
        print post_scores[idx] - pre_scores[idx]

    print "================================================================================"
    print "Overall results"
    print pre_name + ' mean ' + score_name + ': ' + str(np.mean(pre_scores))
    print post_name + ' mean ' + score_name + ': ' + str(np.mean(post_scores))
    improvement = np.mean(np.array(post_scores)) - np.mean(np.array(pre_scores))
    print 'Improvement: ' + str(improvement)
    if score_name == 'accuracy':
        error_reduction = (1 - (1 - np.mean(np.array(post_scores))) / (1 - np.mean(np.array(pre_scores)))) * 100
        print 'Error reduction: %3.1f%%' % (error_reduction)

    pvalue = stratifiedPermutationTest(pre_scores, post_scores, num_samples)
    print 'P < ' + str(pvalue)
