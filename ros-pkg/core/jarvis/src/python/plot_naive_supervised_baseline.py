#!/usr/bin/python

import sys
import argparse
import itertools
import numpy as np
import matplotlib.pyplot as plt
import math

import roslib
roslib.load_manifest('jarvis')
import common_plotting as cp
from regression_testing import * 

parser = argparse.ArgumentParser()
parser.add_argument("-v", "--print-vals", action='store_true')
parser.add_argument("-d", "--dir", help='regression test results directory to run on', type=str, default='.')
parser.add_argument("-o", help="output file (without extension)", type=str, default='nsb')
args = parser.parse_args()

# -- Read in the pairs of (num_tr_ex, acc).
subsample_names = loadBash("find -type d -name 'subsample*' | sort | awk -F'/' '{print $NF}' | sort | uniq")
print subsample_names
#ntrs = npLoadBash("grep -h '[0-9] tracks\.' `find -name training_stats.txt | grep naive_supervised_baseline | sort` | awk '{print $1}'")
#accs = npLoadBash("grep 'Total acc' `find -name track_results.txt | grep naive_supervised_baseline | grep quantity_experiment | sort` | awk '{print $NF}'")

means = np.zeros((len(subsample_names)))
stderrs = np.zeros((len(subsample_names)))
ntrs = np.zeros((len(subsample_names)))

for idx, subsample_name in enumerate(subsample_names):
    accs = npLoadBash("grep 'Total acc' `find -name track_results.txt | grep naive_supervised_baseline | grep quantity | grep " + subsample_name + " | sort` | awk '{print $NF}'")
    ntr = npLoadBash("grep -h '[0-9] tracks\.' `find -name training_stats.txt | grep " + subsample_name + " | sort` | awk '{print $1}' | sort | uniq")
    assert(ntr.size == 1)  # All runs in each subsample should have the same number of tracks.

    means[idx] = np.mean(accs)
    stderrs[idx] = np.std(accs) / math.sqrt(len(accs))
    ntrs[idx] = ntr


# -- Read in the accuracies of group induction.
gi_accs = npLoadBash("grep 'Total acc' `find -L -wholename '*/induction/final_track_results.txt' | sort` | awk '{print $NF}'")
gi_ntrs = npLoadBash("for dir in `find -L -maxdepth 1 -mindepth 1 -type d`; do grep -A2 'Hand-annotated' `find $dir -wholename '*/induction/*/learner_status.txt' | sort | tail -n1`; done | grep tracks | awk '{print $1}'")

print gi_accs
print gi_ntrs

gi_mean = np.mean(gi_accs)
gi_stderr = np.std(gi_accs) / math.sqrt(len(gi_accs))
gi_ntr = np.mean(gi_ntrs)

# -- Make the scatter plot.
fig, ax = plt.subplots()
ax.errorbar(ntrs, means, yerr=stderrs, fmt='-o')
ax.errorbar(gi_ntr, gi_mean, yerr=gi_stderr, fmt='-o')
plt.show()

exit(0)


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

# -- Set up for plotting.
cp.setup()
ind = np.arange(len(test_names))  # the x locations for the groups
width = 1.0 / (len(test_types) + 1)       # the width of the bars

fig = plt.figure(figsize=(15, 5))
ax = fig.add_axes([0.07, 0.07, 0.75, 0.8])
all_rects = []

colors = {'Naive supervised': 'black', 'Matched supervised': 'green', 'Active learning': 'blue', 'Group induction': 'red'}

for idx, test_type in enumerate(test_types):
    means = [np.mean(vals) for vals in vals_dict[test_type]]
    stdevs = [np.std(vals) for vals in vals_dict[test_type]]
    print test_type
    print test_names
    print means
    print stdevs
    rects = ax.bar(ind + width * idx, means, width, color=colors[test_type], yerr=stdevs, label=test_type)
    all_rects.append(rects)

ax.set_ylabel('Accuracy (\%)')
ax.set_title('')
ax.set_xticks(ind + 2 * width)
sanitized_test_names = [sanitizeTestName(test_name) for test_name in test_names]
print sanitized_test_names
ax.set_xticklabels(sanitized_test_names)

ax.set_ylim([70, 100])

from matplotlib.font_manager import FontProperties
font_prop = FontProperties()
font_prop.set_size('small')
ax.legend(loc='center left', bbox_to_anchor=(1, 0.5), prop = font_prop)

plt.show()
                                        
