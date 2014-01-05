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
accs = npLoadBash("grep 'Total acc' `find " + output_dir + " -name final_track_results.txt | sort` | awk '{print $NF}'")
healths = npLoadBash("grep 'Total acc' $(for run in `find -L " + output_dir + " -maxdepth 2 -mindepth 2 -type d | sort`; do find -L $run -name track_results.txt | grep validation | sort | tail -n1; done) | awk '{print $NF}'")

print accs
print healths
