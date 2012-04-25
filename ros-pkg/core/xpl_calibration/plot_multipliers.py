#!/usr/bin/python
from matplotlib.ticker import FuncFormatter

def multFormatter(x, pos):
    return '%1.2f' % (max_mult - (x / num_bins) * (max_mult - min_mult))
def radiusFormatter(x, pos):
    return '%1.1f' % (x / num_bins)

plt.clf()
fig = plt.figure()
ax = fig.add_subplot(111)
ax.xaxis.set_major_formatter(FuncFormatter(radiusFormatter))
ax.yaxis.set_major_formatter(FuncFormatter(multFormatter))
plt.xlabel('Radius')
plt.ylabel('Range multiplier')
img = plt.imshow(hist, interpolation='nearest')
cbar = fig.colorbar(img, ticks=[0, 0.25, 0.5, 0.75, 1])
plt.savefig('heatmap.png')
plt.savefig('heatmap.pdf')
print 'Saved plot to heatmap.{png,pdf}.'
