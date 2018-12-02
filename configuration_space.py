import numpy as np
import matplotlib.pyplot as plt



def create_grid(data, drone_altitude, safety_distance):
    """
    Returns a grid representation of a 2D configuration space
    based on given obstacle data, drone altitude and safety distance
    arguments.
    """

    # minimum and maximum north coordinates
    north_min = np.floor(np.amin(data[:, 0] - data[:, 3]))
    north_max = np.ceil(np.amax(data[:, 0] + data[:, 3]))

    # minimum and maximum east coordinates
    east_min = np.floor(np.amin(data[:, 1] - data[:, 4]))
    east_max = np.ceil(np.amax(data[:, 1] + data[:, 4]))

    # given the minimum and maximum coordinates we can
    # calculate the size of the grid.
    north_size = int(np.ceil(north_max - north_min))
    east_size = int(np.ceil(east_max - east_min))
    # Initialize an empty grid
    grid = np.zeros((north_size, east_size))

    # Center offset for grid
    north_min_center = np.min(data[:, 0])
    east_min_center = np.min(data[:, 1])
    # Populate the grid with obstacles
    for i in range(data.shape[0]):
        north, east, alt, d_north, d_east, d_alt = data[i, :]
        if alt + d_alt + safety_distance > drone_altitude:
            obstacle = [
                int(np.clip(north - d_north - safety_distance - north_min, 0, north_size - 1)),
                int(np.clip(north + d_north + safety_distance - north_min, 0, north_size - 1)),
                int(np.clip(east - d_east - safety_distance - east_min, 0, east_size - 1)),
                int(np.clip(east + d_east + safety_distance - east_min, 0, east_size - 1)),
            ]
            grid[obstacle[0]:obstacle[1] + 1, obstacle[2]:obstacle[3] + 1] = 1

    return grid


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