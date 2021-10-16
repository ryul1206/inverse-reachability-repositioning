from mpl_toolkits.mplot3d import axes3d
import matplotlib.pyplot as plt
import numpy as np

# Make the grid
x, y, z = np.meshgrid(np.arange(-0.8, 1, 0.2),
                      np.arange(-0.8, 1, 0.2),
                      np.arange(-0.8, 1, 0.8))
print(x.shape)
print(x)

# Make the direction data for the arrows
u = np.sin(np.pi * x) * np.cos(np.pi * y) * np.cos(np.pi * z)
v = -np.cos(np.pi * x) * np.sin(np.pi * y) * np.cos(np.pi * z)
w = (np.sqrt(2.0 / 3.0) * np.cos(np.pi * x) * np.cos(np.pi * y) *
     np.sin(np.pi * z))
print(u.shape)

# Color by azimuthal angle
c = np.arctan2(v, u)
# Flatten and normalize
c = (c.ravel() - c.min()) / c.ptp()
# Repeat for each body line and two head lines
print(c.shape)
print(c[:6])
c = np.concatenate((c, np.repeat(c, 2)))
# Colormap
print(c.shape)
print(c[:6])
c = plt.cm.hsv(c)

fig = plt.figure()
ax = fig.gca(projection='3d')
ax.quiver(x, y, z, u, v, w, colors=c, length=0.1, normalize=True)
plt.show()