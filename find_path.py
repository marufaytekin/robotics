import numpy as np
import matplotlib.pyplot as plt
from grid import create_grid
from planning import *
#from bresenham import bresenham
from heuristics import *


def point(p):
    return np.array([p[0], p[1], 1.]).reshape(1, -1)


def collinearity_check(p1, p2, p3, epsilon=1e-6):
    m = np.concatenate((p1, p2, p3), 0)
    det = np.linalg.det(m)
    return abs(det) < epsilon


def prune_path(path):
    _pruned_path = [p for p in path]
    # TODO: prune the path!

    i = 0
    while i < len(_pruned_path) - 2:
        p1 = point(_pruned_path[i])
        p2 = point(_pruned_path[i + 1])
        p3 = point(_pruned_path[i + 2])

        if collinearity_check(p1, p2, p3):
            _pruned_path.remove(_pruned_path[i + 1])
        else:
            i += 1
    return _pruned_path


plt.rcParams['figure.figsize'] = 12, 12

# This is the same obstacle data from the previous lesson.
filename = 'colliders.csv'
data = np.loadtxt(filename, delimiter=',', dtype='Float64', skiprows=2)
print(data)

# Static drone altitude (meters)
drone_altitude = 5

# Minimum distance stay away from obstacle (meters)
safe_distance = 3

# TODO: Use `create_grid` to create a grid configuration space of
# the obstacle data.
grid = create_grid(data, drone_altitude, safe_distance)

# equivalent to
plt.imshow(np.flip(grid, 0))
plt.imshow(grid, origin='lower')

plt.xlabel('EAST')
plt.ylabel('NORTH')
plt.show()

start_ne = (25,  100)
goal_ne = (750., 370.)
path, cost = a_star(grid, euclidean, start_ne, goal_ne)
pruned_path = prune_path(path)
print(pruned_path)
print(len(pruned_path))

plt.imshow(grid, cmap='Greys', origin='lower')

pp = np.array(pruned_path)
plt.plot(pp[:, 1], pp[:, 0], 'r')
plt.scatter(pp[:, 1], pp[:, 0], color = 'r')

plt.plot(start_ne[1], start_ne[0], 'x',  markersize=12)
plt.plot(goal_ne[1], goal_ne[0], 'x',  markersize=12)

plt.xlabel('EAST')
plt.ylabel('NORTH')

plt.show()
