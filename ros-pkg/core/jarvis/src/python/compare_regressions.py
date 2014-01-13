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
parser.add_argument("-t", "--type", help="comparison type", choices=["accuracy", "annotations"], default="accuracy")
parser.add_argument("pre_dir", help="pre-condition regression test dir")
parser.add_argument("post_dir", help="post-condition regression test dir")
args = parser.parse_args()

pre_test_names = sorted(os.walk(args.pre_dir).next()[1])
post_test_names = sorted(os.walk(args.post_dir).next()[1])
assert(pre_test_names == post_test_names)
test_names = pre_test_names
pre_vals = []
post_vals = []

if args.type == 'accuracy':
    label = 'Accuracy (%)'
    for test_name in test_names:
        pre_vals.append(100 * npLoadBash("grep 'Total acc' `find -L " + os.path.join(args.pre_dir, test_name) + " -name 'final_track_results.txt' | sort` | awk '{print $NF}'"))
        post_vals.append(100 * npLoadBash("grep 'Total acc' `find -L " + os.path.join(args.post_dir, test_name) + " -name 'final_track_results.txt' | sort` | awk '{print $NF}'"))

elif args.type == 'annotations':
    label = 'Annotations'
    for test_name in test_names:
        pre_vals.append(npLoadBash("for dir in `find -L " + os.path.join(args.pre_dir, test_name) + " -maxdepth 1 -mindepth 1 -type d`; do grep -A2 'Hand-annotated' `find $dir -name learner_status.txt | sort | tail -n1`; done | grep tracks | awk '{print $1}'"))
        post_vals.append(npLoadBash("for dir in `find -L " + os.path.join(args.post_dir, test_name) + " -maxdepth 1 -mindepth 1 -type d`; do grep -A2 'Hand-annotated' `find $dir -name learner_status.txt | sort | tail -n1`; done | grep tracks | awk '{print $1}'"))

analyze(pre_vals, post_vals,
        'Pre', 'Post',
        test_names, label, 10000)

