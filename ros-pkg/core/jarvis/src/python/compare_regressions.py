#!/usr/bin/python

import sys
import os
import argparse
import itertools
import numpy as np

import roslib
roslib.load_manifest('jarvis')
from regression_testing import * 


parser = argparse.ArgumentParser()
parser.add_argument("pre_dir", help="pre-condition regression test dir")
parser.add_argument("post_dir", help="post-condition regression test dir")
parser.add_argument("-t", "--type", help="comparison type", choices=["acc", "ann"], default="acc")
parser.add_argument("-v", "--print-vals", action='store_true')
parser.add_argument("-n", "--num-permutations", type=int, default=10000)
parser.add_argument("--include-tests", help="limit to these tests", type=str, nargs='+', default=[])
parser.add_argument("--exclude-tests", help="limit to these tests", type=str, nargs='+', default=[])
args = parser.parse_args()

pre_test_names = sorted(os.walk(args.pre_dir).next()[1])
post_test_names = sorted(os.walk(args.post_dir).next()[1])
test_names = sorted(list(set(pre_test_names) & set(post_test_names)))  # Only consider tests that appear in both directories.
if len(args.include_tests) > 0:
    assert(len(args.exclude_tests) == 0)
    for name in args.include_tests:
        assert(name in test_names)
    test_names = args.include_tests
if len(args.exclude_tests) > 0:
    assert(len(args.include_tests) == 0)
    for name in args.exclude_tests:
       test_names.remove(name)


pre_vals = []
post_vals = []

if args.type == 'acc':
    label = 'Accuracy (%)'
    for test_name in test_names:
        pre_vals.append(100 * npLoadBash("grep 'Total acc' `find -L " + os.path.join(args.pre_dir, test_name) + " -wholename '*/induction/final_track_results.txt' | sort` | awk '{print $NF}'"))
        post_vals.append(100 * npLoadBash("grep 'Total acc' `find -L " + os.path.join(args.post_dir, test_name) + " -wholename '*/induction/final_track_results.txt' | sort` | awk '{print $NF}'"))

elif args.type == 'ann':
    label = 'Annotations'
    for test_name in test_names:
        pre_vals.append(npLoadBash("for dir in `find -L " + os.path.join(args.pre_dir, test_name) + " -maxdepth 1 -mindepth 1 -type d`; do grep -A2 'Hand-annotated' `find $dir -name learner_status.txt | sort | tail -n1`; done | grep tracks | awk '{print $1}'"))
        post_vals.append(npLoadBash("for dir in `find -L " + os.path.join(args.post_dir, test_name) + " -maxdepth 1 -mindepth 1 -type d`; do grep -A2 'Hand-annotated' `find $dir -name learner_status.txt | sort | tail -n1`; done | grep tracks | awk '{print $1}'"))

if args.print_vals:
    print
    for i in range(len(test_names)):
        print test_names[i]
        print str(pre_vals[i]) + "\t" + str(np.mean(pre_vals[i]))
        print str(post_vals[i]) + "\t" + str(np.mean(post_vals[i]))
        print

analyze(pre_vals, post_vals,
        'Pre', 'Post',
        test_names, label,
        args.num_permutations, paired=False)



# time
    # pre_time.append(npLoadBash("for dir in `find -L " + os.path.join(pre_dir, test_name) + " -maxdepth 1 -mindepth 1 -type d`; do grep 'Total elapsed time' `find $dir -name learner_status.txt | sort | tail -n1`; done | awk '{print $NF}'"))
    # post_time.append(npLoadBash("for dir in `find -L " + os.path.join(post_dir, test_name) + " -maxdepth 1 -mindepth 1 -type d`; do grep 'Total elapsed time' `find $dir -name learner_status.txt | sort | tail -n1`; done | awk '{print $NF}'"))
 
