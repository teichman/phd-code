#!/usr/bin/python

from pylab import *
import numpy
import sys
import os
from matplotlib import rc

linestyles = ['ko-', 'rv--', 'bs:', 'b*--', 'g+-', 'kh:']
colors = ['#0B6184', '#1B1BB3', '#FFBF00', '#FF9200', '#560EAD', '#A67E00', '#0B6184', '#FFBF00', '#FF9200', '#560EAD', '#A67E00', '#1B1BB3']
markers = ['d', 'o', 'h', 's', '^', 'x', 'd', 'o', 'h', 's', '^', 'x']

def setup():
    fig_width_pt = 500.0  # Get this from LaTeX using \showthe\columnwidth
    inches_per_pt = 1.0/72.27               # Convert pt to inch
    golden_mean = (sqrt(5)-1.0)/2.0         # Aesthetic ratio
    fig_width = fig_width_pt*inches_per_pt  # width in inches
    fig_height = fig_width*golden_mean      # height in inches
    fig_size =  [fig_width,fig_height]
    params = {'figure.figsize': fig_size}
    rcParams['lines.linewidth'] = 3
    rcParams['font.size'] = 14
    rcParams['text.usetex'] = True
    rcParams['font.family'] = 'serif'
    rcParams.update(params)
