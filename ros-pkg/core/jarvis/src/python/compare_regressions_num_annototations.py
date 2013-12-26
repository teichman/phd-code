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
pre_numann = []
post_numann = []

assert(pre_test_names == post_test_names)
test_names = pre_test_names

for test_name in test_names:
    pre_numann.append(-npLoadBash("for dir in `find -L " + os.path.join(pre_dir, test_name) + " -maxdepth 1 -mindepth 1 -type d`; do grep -A2 'Hand-annotated' `find $dir -name learner_status.txt | sort | tail -n1`; done | grep tracks | awk '{print $1}'"))
    post_numann.append(-npLoadBash("for dir in `find -L " + os.path.join(post_dir, test_name) + " -maxdepth 1 -mindepth 1 -type d`; do grep -A2 'Hand-annotated' `find $dir -name learner_status.txt | sort | tail -n1`; done | grep tracks | awk '{print $1}'"))

compareTests(pre_numann, post_numann,
             'Pre', 'Post',
             test_names, 'negative num user annotations', 10000)

