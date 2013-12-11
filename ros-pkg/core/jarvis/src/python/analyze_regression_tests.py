#!/usr/bin/python

import sys
import argparse
import itertools
import matplotlib.pyplot as plt

import roslib
roslib.load_manifest('jarvis')
from regression_testing import * 
    
if len(sys.argv) != 2:
    print 'Usage: ' + sys.argv[0] + ' DIR'
    print '  where DIR is the output directory for a full suite of regression tests.'
    exit(0)

output_dir = sys.argv[1]
test_names = sorted(os.walk('.').next()[1])

gi_accs_all = []
baseline_accs_all = []
for test_name in test_names:
    print "============================================================"
    print "Test: " + test_name
    gi_accs = npLoadBash("grep 'Total acc' `find " + os.path.join(output_dir, test_name) + " -name 'final_track_results.txt' | sort` | awk '{print $NF}'")
    gi_accs_all.append(gi_accs)
    baseline_accs = npLoadBash("grep 'Total acc' `find " + os.path.join(output_dir, test_name) + " -wholename '*iter009/track_results.txt' | sort` | awk '{print $NF}'")
    baseline_accs_all.append(baseline_accs)
    print 'Baseline mean accuracy: ' + str(np.mean(baseline_accs))
    print 'Group induction mean accuracy: ' + str(np.mean(gi_accs))
    improvement = np.mean(gi_accs) - np.mean(baseline_accs)
    print 'Improvement: ' + str(improvement)
    pvalue = permutationTest(baseline_accs, gi_accs)
    print 'P < ' + str(pvalue)
    print
    print gi_accs - baseline_accs

print "============================================================"
print "Overall results"
improvement = np.mean(np.array(gi_accs_all)) - np.mean(np.array(baseline_accs_all))
print 'Baseline mean accuracy: ' + str(np.mean(baseline_accs_all))
print 'Group induction mean accuracy: ' + str(np.mean(gi_accs_all))
print 'Improvement: ' + str(improvement)
pvalue = stratifiedPermutationTest(baseline_accs_all, gi_accs_all)
print 'P < ' + str(pvalue)
