#!/usr/bin/python
#from matplotlib.ticker import FuncFormatter

#def multFormatter(x, pos):
#    return '%1.2f' % (max_mult - (x / num_bins) * (max_mult - min_mult))
#def radiusFormatter(x, pos):
#    return '%1.1f' % (x / num_bins)

plt.clf()
fig = plt.figure(figsize=(20, 5), dpi=100)

ax1 = fig.add_subplot(1, 3, 1)
ax1.set_title('Depth multiplier mean')
img1 = plt.imshow(mean, interpolation='nearest')
cbar1 = fig.colorbar(img1, orientation='vertical')

ax2 = fig.add_subplot(1, 3, 2)
ax2.set_title('Depth multiplier stdev')
img2 = plt.imshow(stdev, interpolation='nearest')
cbar2 = fig.colorbar(img2, orientation='vertical')

ax3 = fig.add_subplot(1, 3, 3)
ax3.set_title('Counts')
img3 = plt.imshow(counts, interpolation='nearest')
cbar3 = fig.colorbar(img3, orientation='vertical')

plt.savefig('multipliers_image.png')
plt.savefig('multipliers_image.pdf')
print 'Saved plot to multipliers_image.{png,pdf}.'
