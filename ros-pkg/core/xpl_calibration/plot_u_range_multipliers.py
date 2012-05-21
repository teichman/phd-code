#!/usr/bin/python
from matplotlib.ticker import FuncFormatter

def rangeFormatter(x, pos):
    return '%1.1f' % (max_range - x * bin_size)

plt.clf()
fig = plt.figure(figsize=(20, 5), dpi=100)

ax1 = fig.add_subplot(1, 3, 1)
ax1.set_title('Depth multiplier mean')
ax1.set_xlabel('u')
ax1.set_ylabel('range')
ax1.yaxis.set_major_formatter(FuncFormatter(rangeFormatter))
img1 = plt.imshow(mean, interpolation='nearest')
cbar1 = fig.colorbar(img1, orientation='vertical')

ax2 = fig.add_subplot(1, 3, 2)
ax2.set_title('Depth multiplier stdev')
ax2.set_xlabel('u')
ax2.set_ylabel('range')
ax2.yaxis.set_major_formatter(FuncFormatter(rangeFormatter))
img2 = plt.imshow(stdev, interpolation='nearest')
cbar2 = fig.colorbar(img2, orientation='vertical')

ax3 = fig.add_subplot(1, 3, 3)
ax3.set_title('Counts')
ax3.set_xlabel('u')
ax3.set_ylabel('range')
ax3.yaxis.set_major_formatter(FuncFormatter(rangeFormatter))
img3 = plt.imshow(counts, interpolation='nearest')
cbar3 = fig.colorbar(img3, orientation='vertical')

plt.savefig('u_range_image.png')
plt.savefig('u_range_image.pdf')
print 'Saved plot to u_range_image.{png,pdf}.'
