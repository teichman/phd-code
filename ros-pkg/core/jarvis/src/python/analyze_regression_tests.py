#!/usr/bin/python

import sys
import argparse
import itertools

import roslib
roslib.load_manifest('jarvis')
from regression_testing import * 

parser = argparse.ArgumentParser()
parser.add_argument("-t", "--type", help="comparison type", choices=["msb", "nsb", "act", "ann"], required=True)
parser.add_argument("dir", help="regression test dir")
parser.add_argument("-n", "--num-permutations", type=int, default=10000)
parser.add_argument("-v", "--print-vals", action='store_true')
parser.add_argument("--include-tests", help="limit to these tests", type=str, nargs='+', default=[])
parser.add_argument("--exclude-tests", help="limit to these tests", type=str, nargs='+', default=[])
args = parser.parse_args()

# -- Load data.
output_dir = sys.argv[1]
test_names = sorted(os.walk(output_dir).next()[1])
gi_vals = []
baseline_vals = []
if len(args.include_tests) > 0:
    for name in args.include_tests:
        assert(name in test_names)
    test_names = args.include_tests
if len(args.exclude_tests) > 0:
    assert(len(args.include_tests) == 0)
    for name in args.exclude_tests:
       test_names.remove(name)

test_names = sorted(test_names)

if args.type == "msb":
    baseline_label = "MSB"
    label = 'Accuracy (%)'
    for test_name in test_names:
        gi_vals.append(100 * npLoadBash("grep 'Total acc' `find -L " + os.path.join(output_dir, test_name) + " -wholename '*/induction/final_track_results.txt' | sort` | awk '{print $NF}'"))
        baseline_vals.append(100 * npLoadBash("grep 'Total acc' `find -L " + os.path.join(output_dir, test_name) + " -wholename '*baseline_unfair/*iter009/track_results.txt' | sort` | awk '{print $NF}'"))

elif args.type == "nsb":
    baseline_label = "NSB"
    label = 'Accuracy (%)'
    for test_name in test_names:
        gi_vals.append(100 * npLoadBash("grep 'Total acc' `find -L " + os.path.join(output_dir, test_name) + " -wholename '*/induction/final_track_results.txt' | sort` | awk '{print $NF}'"))
        baseline_vals.append(100 * npLoadBash("grep 'Total acc' `find -L " + os.path.join(output_dir, test_name) + " -wholename '*/naive_supervised_baseline/average_results/track_results.txt' | sort` | awk '{print $NF}'"))

elif args.type == "act":
    baseline_label = "ACT"
    label = 'Accuracy (%)'
    for test_name in test_names:
        gi_vals.append(100 * npLoadBash("grep 'Total acc' `find -L " + os.path.join(output_dir, test_name) + " -wholename '*/induction/final_track_results.txt' | sort` | awk '{print $NF}'"))
        baseline_vals.append(100 * npLoadBash("grep 'Total acc' `find -L " + os.path.join(output_dir, test_name) + " -wholename '*/active_learning/final_track_results.txt' | sort` | awk '{print $NF}'"))


elif args.type == "ann":
    baseline_label = "ACT"
    label = 'Num annotations'
    for test_name in test_names:
        gi_vals.append(npLoadBash("for dir in `find -L " + os.path.join(output_dir, test_name) + " -maxdepth 1 -mindepth 1 -type d`; do grep -A2 'Hand-annotated' `find $dir -wholename '*/induction/*/learner_status.txt' | sort | tail -n1`; done | grep tracks | awk '{print $1}'"))
        baseline_vals.append(npLoadBash("for dir in `find -L " + os.path.join(output_dir, test_name) + " -maxdepth 1 -mindepth 1 -type d`; do grep -A2 'Hand-annotated' `find $dir -wholename '*/active_learning/*/learner_status.txt' | sort | tail -n1`; done | grep tracks | awk '{print $1}'"))

if args.print_vals:
    print
    for i in range(len(test_names)):
        print test_names[i]
        print str(baseline_vals[i]) + "\t" + str(np.mean(baseline_vals[i]))
        print str(gi_vals[i]) + "\t" + str(np.mean(gi_vals[i]))
        print

# -- Run test.
analyze(baseline_vals, gi_vals,
        baseline_label, 'GI',
        test_names, label, args.num_permutations, paired = True)
