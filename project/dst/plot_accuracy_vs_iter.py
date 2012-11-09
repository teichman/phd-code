#!/usr/bin/python

from pylab import *
import numpy
import sys
import os
import common_plotting as cp

if(len(sys.argv) != 2):
    print "Usage: python plot_accuracy_vs_iter.py AVI_DIR"
    sys.exit(1)

# -- Config.
cp.setup()

# -- Load data and draw.
fig = figure()
ax = fig.add_subplot(1, 1, 1)
ax.set_xlabel('Learning Iteration')
ax.set_ylabel('Accuracy')
grid(True)
tmpfile = ".python-aosethusnatohesutahoesnut"
os.system("find " + sys.argv[1] + " -name 'weights0*-results.txt' | sort | awk -F/ '{print $4}' | sed 's/[^0-9]//g' | sort | tail -n1 > " + tmpfile)
maxidx = numpy.loadtxt(tmpfile)
print "maxidx:", maxidx
org = [np.array([]) for x in np.arange(0, maxidx + 1)]
assert(len(org) == maxidx + 1)
for path in os.listdir(sys.argv[1]):
    if os.path.isdir(path):
        os.system("cat `find " + path + " -name '*-results.txt' | sort` | grep 'Overall mean' | awk '{print $NF}' > " + tmpfile)
        vals = numpy.loadtxt(tmpfile)
        vals = np.ones(len(vals)) - vals
        os.system("find " + path + " -name '*-results.txt' | sort | awk -F/ '{print $2}' | sed 's/weights//' > " + tmpfile)
        indices = numpy.loadtxt(tmpfile)
        for i, index in enumerate(indices):
            org[int(index)] = np.append(org[int(index)], vals[i])

print "lengths:"
print [len(x) for x in org]
indices = [idx for idx, x in enumerate(org) if len(x) > 0]
means = [np.mean(x) for x in org if len(x) > 0]
stdevs = [np.std(x) for x in org if len(x) > 0]
print "indices:"
print indices
print "means:"
print means
print "stdevs:"
print stdevs

ax.plot(indices, means, zorder=1, color='green')
ax.errorbar(indices, means, yerr=stdevs, fmt=None, ecolor='gray', elinewidth=2, ecapsize=2, zorder=10)

os.system("rm " + tmpfile)
ylim(0, 1)

savefig('accuracy_vs_iter.pdf')
savefig('accuracy_vs_iter.png')


