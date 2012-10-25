#!/usr/bin/python

from pylab import *
import numpy
import sys
import os
import common_plotting as cp

if(len(sys.argv) < 2):
    print "Usage: python plot_accuracy_vs_speed.py VALS [VALS ...]"
    sys.exit(1)

# -- Config.
cp.setup()

# -- Load and process data.
times = []
accuracies = []
names = []
slowest = 0
for valspath in sys.argv[1:]:
    print 'loading ' + valspath
    vals = numpy.loadtxt(valspath)
    times.append(vals[:,0])
    accuracies.append(ones(len(vals[:,1])) - vals[:,1])
    names.append(valspath[2:14].replace('_', " "))
    print names[-1]
    print times[-1]
    print accuracies[-1]
    if times[-1].max(0) > slowest:
        slowest = times[-1].max(0)

speedups = []
for t in times:
    speedups.append((t / slowest)**-1)
    
# -- Draw.
fig = figure()
xlabel('Speedup')
ylabel('Accuracy')
grid(True)
for idx, s in enumerate(speedups):
    name = names[idx]
    lab = ""
    skip = float(name[10:])
    print name
    print 'skip is ' + name[10:] + " " + str(skip)
    pct = 100.0 * (1.0 - 1.0 / skip)
    lab += str(pct) + '\% downsampling'
#    if name[6:11] == 'skip1':
#        lab += '00\% downsampling'
#    elif name[6:11] == 'skip2':
#        lab += '50\% downsampling'
#    elif name[6:11] == 'skip4':
#        lab += '75\% downsampling'
#    elif name[6:11] == 'skip7':
#        lab += '86\% downsampling'

    if name[0:5] == "mask0":
        lab += ', no mask'
    else:
        lab += ', with mask'
        
    scatter(s, accuracies[idx], marker=cp.markers[idx], c=cp.colors[idx], s=40, label=lab)


ylim(0, 1)
legend(loc='lower left')
savefig('accuracy_vs_speed.pdf')
savefig('accuracy_vs_speed.png')


