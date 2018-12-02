import numpy as np
import matplotlib.pyplot as plt
from grid import create_grid

plt.rcParams["figure.figsize"] = [12, 12]

filename = 'colliders.csv'
# Read in the data skipping the first two lines.
# Note: the first line contains the latitude and longitude of map center
# Where is this??
data = np.loadtxt(filename,delimiter=',',dtype='Float64',skiprows=2)
print(data)

# Static drone altitude (metres)
drone_altitude = 5

# Minimum distance required to stay away from an obstacle (metres)
# Think of this as padding around the obstacles.
safe_distance = 3

grid = create_grid(data, drone_altitude, safe_distance)

# equivalent to
# plt.imshow(np.flip(grid, 0))
# NOTE: we're placing the origin in the lower lefthand corner here
# so that north is up, if you didn't do this north would be positive down
plt.imshow(grid, origin='lower')

plt.xlabel('EAST')
plt.ylabel('NORTH')
plt.show()