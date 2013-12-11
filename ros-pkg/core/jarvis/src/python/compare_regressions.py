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
pre_accs = []
post_accs = []

assert(pre_test_names == post_test_names)
test_names = pre_test_names

for test_name in test_names:
    pre_accs.append(npLoadBash("grep 'Total acc' `find " + os.path.join(pre_dir, test_name) + " -name 'final_track_results.txt' | sort` | awk '{print $NF}'"))
    post_accs.append(npLoadBash("grep 'Total acc' `find " + os.path.join(post_dir, test_name) + " -name 'final_track_results.txt' | sort` | awk '{print $NF}'"))

compareTests(pre_accs, post_accs,
             'Pre', 'Post',
             test_names, 'accuracy', 10000)

