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
def permutationTest(pre, post, num_samples = 10000):
    improvement = np.mean(post) - np.mean(pre)
    all = np.concatenate((pre, post))
    num_perm_better = 0.
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
def stratifiedPermutationTest(pre, post, num_samples = 10000):
    improvement = np.mean(np.array(post)) - np.mean(np.array(pre))
    
    num_perm_better = 0.
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
        pvalue = permutationTest(pre_scores[idx], post_scores[idx], num_samples)
        print 'P < ' + str(pvalue)
        print
        print post_scores[idx] - pre_scores[idx]

    print "================================================================================"
    print "Overall results"
    improvement = np.mean(np.array(post_scores)) - np.mean(np.array(pre_scores))
    print pre_name + ' mean ' + score_name + ': ' + str(np.mean(pre_scores))
    print post_name + ' mean ' + score_name + ': ' + str(np.mean(post_scores))
    print 'Improvement: ' + str(improvement)
    pvalue = stratifiedPermutationTest(pre_scores, post_scores, num_samples)
    print 'P < ' + str(pvalue)
