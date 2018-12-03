import numpy as np
import matplotlib.pyplot as plt
from grid import create_grid
from skimage.morphology import medial_axis
from skimage.util import invert
from planning import a_star
from heuristics import *

plt.rcParams['figure.figsize'] = 12, 12

# This is the same obstacle data from the previous lesson.
filename = 'colliders.csv'
data = np.loadtxt(filename, delimiter=',', dtype='Float64', skiprows=2)


def find_start_goal(skel, start, goal):
    # TODO: find start and goal on skeleton
    # Some useful functions might be:
    # np.nonzero()
    # np.transpose()
    # np.linalg.norm()
    # np.argmin()
    skel_cells = np.transpose(skel.nonzero())
    start_min_dist = np.linalg.norm(np.array(start) - np.array(skel_cells), axis=1).argmin()
    near_start = skel_cells[start_min_dist]
    goal_min_dist = np.linalg.norm(np.array(goal) - np.array(skel_cells), axis=1).argmin()
    near_goal = skel_cells[goal_min_dist]

    return near_start, near_goal


start_ne = (25, 100)
goal_ne = (650, 500)

# TODO: Your start and goal location defined above
# will not necessarily be on the skeleton so you
# must first identify the nearest cell on the
# skeleton to start and goal

# Static drone altitude (meters)
drone_altitude = 5
safety_distance = 2

grid = create_grid(data, drone_altitude, safety_distance)
skeleton = medial_axis(invert(grid))

# equivalent to
# plt.imshow(np.flip(grid, 0))
plt.imshow(grid, cmap='Greys', origin='lower')
plt.imshow(skeleton, cmap='Greys', origin='lower', alpha=0.7)

plt.plot(start_ne[1], start_ne[0], 'rx')
plt.plot(goal_ne[1], goal_ne[0], 'x')

plt.xlabel('EAST')
plt.ylabel('NORTH')
plt.show()

skel_start, skel_goal = find_start_goal(skeleton, start_ne, goal_ne)
print(start_ne, goal_ne)
print(skel_start, skel_goal)

path, cost = a_star(invert(skeleton).astype(np.int), euclidean, tuple(skel_start), tuple(skel_goal))
print(cost)
# adding start and goal to path
path.insert(0, tuple(start_ne))
path.append(tuple(goal_ne))

# Compare to regular A* on the grid
path2, cost2 = a_star(grid, euclidean, start_ne, goal_ne)
print(cost2)

plt.imshow(grid, cmap='Greys', origin='lower')
plt.imshow(skeleton, cmap='Greys', origin='lower', alpha=0.7)

# For the purposes of the visual the east coordinate lay along
# the x-axis and the north coordinates long the y-axis.
plt.plot(start_ne[1], start_ne[0], 'rx')
plt.plot(goal_ne[1], goal_ne[0], 'x')

pp = np.array(path)
plt.plot(pp[:, 1], pp[:, 0], 'g')
pp2 = np.array(path2)
plt.plot(pp2[:, 1], pp2[:, 0], 'r')

plt.xlabel('EAST')
plt.ylabel('NORTH')
plt.show()
