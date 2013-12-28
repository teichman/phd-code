#!/usr/bin/python

import sys
import argparse
import itertools

import roslib
roslib.load_manifest('jarvis')
from regression_testing import * 

if len(sys.argv) != 2:
    print 'Usage: ' + sys.argv[0] + ' DIR'
    print '  where DIR is the output directory for a full suite of regression tests.'
    exit(0)

# -- Load data.
output_dir = sys.argv[1]
test_names = sorted(os.walk(output_dir).next()[1])
gi_accs = []
baseline_accs = []
for test_name in test_names:
    gi_accs.append(100 * npLoadBash("grep 'Total acc' `find -L " + os.path.join(output_dir, test_name) + " -name 'final_track_results.txt' | sort` | awk '{print $NF}'"))
    baseline_accs.append(100 * npLoadBash("grep 'Total acc' `find -L " + os.path.join(output_dir, test_name) + " -wholename '*iter009/track_results.txt' | sort` | awk '{print $NF}'"))

# -- Run test.
analyze(baseline_accs, gi_accs,
        'Baseline', 'GI',
        test_names, 'Accuracy (%)', 10000)
