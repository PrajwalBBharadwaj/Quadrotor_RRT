import matplotlib.pyplot as plt
from shapely.geometry import LinearRing
from shapely.plotting import plot_line, plot_points


fig = plt.figure(1, dpi=90)

# 1: valid ring
ax = fig.add_subplot(11)
ring = LinearRing([(0, 0), (0, 2), (1, 1), (2, 2), (2, 0), (1, 0.8), (0, 0)])

plot_line(ring, ax=ax, add_points=False, alpha=0.7)
plot_points(ring, ax=ax, alpha=0.7)

ax.set_title('a) valid')

ax.set_xlim(-3, 3)
ax.set_ylim(-3, 3)

# #2: invalid self-touching ring
# ax = fig.add_subplot(122)
# ring2 = LinearRing([(0, 0), (0, 2), (1, 1), (2, 2), (2, 0), (1, 1), (0, 0)])

# plot_line(ring2, ax=ax, add_points=False, alpha=0.7)
# plot_points(ring2, ax=ax, alpha=0.7)

# ax.set_title('b) invalid')

# ax.set_xlim(-3, 3)
# ax.set_ylim(-3, 3)

plt.show()
