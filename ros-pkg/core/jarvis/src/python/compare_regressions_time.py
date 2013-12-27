#!/usr/bin/python

import sys
import os
import argparse
import itertools
import numpy as np

import roslib
roslib.load_manifest('jarvis')
from regression_testing import * 

if len(sys.argv) != 3:
    print 'Usage: compare_regressions.py PRE POST'
    print '  where PRE and POST are directories containing regression runs.'
    exit(0)

pre_dir = sys.argv[1]
post_dir = sys.argv[2]
pre_test_names = sorted(os.walk(pre_dir).next()[1])
post_test_names = sorted(os.walk(post_dir).next()[1])
pre_time = []
post_time = []

assert(pre_test_names == post_test_names)
test_names = pre_test_names

for test_name in test_names:
    pre_time.append(npLoadBash("for dir in `find -L " + os.path.join(pre_dir, test_name) + " -maxdepth 1 -mindepth 1 -type d`; do grep 'Total elapsed time' `find $dir -name learner_status.txt | sort | tail -n1`; done | awk '{print $NF}'"))
    post_time.append(npLoadBash("for dir in `find -L " + os.path.join(post_dir, test_name) + " -maxdepth 1 -mindepth 1 -type d`; do grep 'Total elapsed time' `find $dir -name learner_status.txt | sort | tail -n1`; done | awk '{print $NF}'"))

analyze(pre_time, post_time,
        'Pre', 'Post',
        test_names, 'Seconds', 10000)

