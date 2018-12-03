from planning import *
from heuristics import *

start = (0, 0)
goal = (4, 4)

grid = np.array([
    [0, 1, 0, 0, 0, 0],
    [0, 0, 0, 0, 0, 0],
    [0, 1, 0, 0, 0, 0],
    [0, 0, 0, 1, 1, 0],
    [0, 0, 0, 1, 0, 0],
])

path, cost = a_star(grid, manhattan, start, goal)
print(path, cost)

# S -> start, G -> goal, O -> obstacle
#print(visualize_path(grid, path, start))
