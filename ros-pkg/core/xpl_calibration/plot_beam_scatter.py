#!/usr/bin/python
plt.clf()
fig = plt.figure()

plt.plot([0, 13], [0, 13], 'g', zorder=0)
plt.scatter(velo, asus, zorder=1)
plt.xlabel('Velodyne range')
plt.ylabel('Asus range')
plt.title('Pixel u=%d, v=%d in %d x %d depth image' % (u, v, width, height))
plt.savefig('beam_u%d_v%d_scatter.png' % (u, v))
plt.savefig('beam_u%d_v%d_scatter.pdf' % (u, v))

print 'Saved plot to beam_u%d_v%d_scatter.png' % (u, v)
