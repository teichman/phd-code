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
xlabel('Learning Iteration')
ylabel('Accuracy')
grid(True)
tmpfile = ".python-aosethusnatohesutahoesnut"
indices = []
vals = []
maxidx = 0
for path in os.listdir(sys.argv[1]):
    if os.path.isdir(path):
#        print path
        os.system("cat `find " + path + " -name '*-results.txt' | sort` | grep 'Overall mean' | awk '{print $NF}' > " + tmpfile)
        vals.append(numpy.loadtxt(tmpfile))
        os.system("find " + path + " -name '*-results.txt' | sort | awk -F/ '{print $2}' | sed 's/weights//' > " + tmpfile)
        indices.append(numpy.loadtxt(tmpfile))
        maxidx = max(np.amax(indices[-1]), maxidx)
#        scatter(indices, ones(len(vals)) - vals, marker=cp.markers[0], s=40)

print 'max: ' + str(int(maxidx))
org = []
for i in np.arange(0, int(maxidx)+1):
    org.append(np.array([]))
    for j, index in enumerate(indices):
        for ind in index:
            if ind == i:
                org[i].append(vals[j])

print org
#totals = np.array()




os.system("rm " + tmpfile)
ylim(0, 1)

savefig('accuracy_vs_iter.pdf')
savefig('accuracy_vs_iter.png')


