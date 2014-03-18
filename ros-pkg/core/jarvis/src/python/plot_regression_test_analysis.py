#!/usr/bin/python

import sys
import argparse
import itertools

import roslib
roslib.load_manifest('jarvis')
from regression_testing import * 

parser = argparse.ArgumentParser()
parser.add_argument("dir", help="regression test dir")
parser.add_argument("-n", "--num-permutations", type=int, default=10000)
parser.add_argument("-v", "--print-vals", action='store_true')
parser.add_argument("--include-tests", help="limit to these tests", type=str, nargs='+', default=[])
parser.add_argument("--exclude-tests", help="limit to these tests", type=str, nargs='+', default=[])
args = parser.parse_args()

# -- Get the test names to use.
test_names = sorted(os.walk(args.dir).next()[1])
if len(args.include_tests) > 0:
    for name in args.include_tests:
        assert(name in test_names)
    test_names = args.include_tests
if len(args.exclude_tests) > 0:
    assert(len(args.include_tests) == 0)
    for name in args.exclude_tests:
       test_names.remove(name)
test_names = sorted(test_names)

# -- Get the test types to use and set up a dictionary for results to live in.
test_types = ['Naive supervised', 'Matched supervised', 'Active learning', 'Group induction']
vals_dict = {test_type: [] for test_type in test_types}

for test_type in test_types:
    print 'Reading data for test type ' + test_type

    if test_type == 'Matched supervised':
        for test_name in test_names:
            vals_dict[test_type].append(100 * npLoadBash("grep 'Total acc' `find -L " + os.path.join(args.dir, test_name) + " -wholename '*baseline_unfair/*iter009/track_results.txt' | sort` | awk '{print $NF}'"))

    elif test_type == 'Naive supervised':
        for test_name in test_names:
            vals_dict[test_type].append(100 * npLoadBash("grep 'Total acc' `find -L " + os.path.join(args.dir, test_name) + " -wholename '*/naive_supervised_baseline/average_results/track_results.txt' | sort` | awk '{print $NF}'"))
    
    elif test_type == 'Active learning':
        for test_name in test_names:
            vals_dict[test_type].append(100 * npLoadBash("grep 'Total acc' `find -L " + os.path.join(args.dir, test_name) + " -wholename '*/active_learning/final_track_results.txt' | sort` | awk '{print $NF}'"))
    
    elif test_type == 'Group induction':
        for test_name in test_names:
            vals_dict[test_type].append(100 * npLoadBash("grep 'Total acc' `find -L " + os.path.join(args.dir, test_name) + " -wholename '*/induction/final_track_results.txt' | sort` | awk '{print $NF}'"))
        
# -- Debugging printout.
if args.print_vals:
    print
    for test_type, vals in vals_dict.iteritems():
        print test_type
        for i in range(len(test_names)):
            print test_names[i] + "\t" + str(vals[i]) + "\t" + str(np.mean(vals[i]))

