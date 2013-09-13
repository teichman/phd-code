#!/usr/bin/python

import sys
import os
import argparse
import numpy as np
import matplotlib.pyplot as plt
import common_plotting as cp

parser = argparse.ArgumentParser()
parser.add_argument('--out', help='Output filename base, no extension.', default='baseline_plot')
parser.add_argument('--dir', help='Baseline experiment dir.', default='.')

args = parser.parse_args()
basename = args.dir + '/plot'
print 'Saving to ' + basename

# -- Config
cp.setup()

# -- Load data
rundirs = [name for name in os.listdir(args.dir) if os.path.isdir(os.path.join(args.dir, name))]
accs = []
ntracks = []
tmpfile = ".python-aosetuhsatoehuntahoesuth"
for rundir in rundirs:
    os.system("grep 'Total acc' `find " + os.path.join(args.dir, rundir) + " -name track_results.txt | grep -v iter | sort` | awk '{print $NF}' > " + tmpfile)
    accs.append(np.loadtxt(tmpfile))
    os.system("ls " + os.path.join(args.dir, rundir) + " | grep tracks | awk -Ft '{print $1}' > " + tmpfile)
    ntracks.append(np.loadtxt(tmpfile))

for idx, acc in enumerate(accs):
    plt.plot(ntracks[idx], acc)

ymin, ymax = plt.ylim()
plt.ylim(ymin, 1.0)
plt.ylabel('Accuracy')
plt.xlabel('Number of hand-labeled tracks')
plt.title('Fully-supervised baseline')

plt.savefig(basename + '.pdf')
plt.savefig(basename + '.png')

