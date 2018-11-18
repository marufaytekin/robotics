# Import numpy, Enum and Queue
import numpy as np
from enum import Enum
from queue import Queue, PriorityQueue
import math


class Action(Enum):
    """
    An action is represented by a 3 element tuple.

    The first 2 values are the delta of the action relative
    to the current grid position. The third and final value
    is the cost of performing the action.
    """
    LEFT = (0, -1, 1)
    RIGHT = (0, 1, 1)
    UP = (-1, 0, 1)
    DOWN = (1, 0, 1)
    UP_RIGHT = (-1, 1, math.sqrt(2))
    UP_LEFT = (-1, -1, math.sqrt(2))
    DOWN_RIGHT = (1, 1, math.sqrt(2))
    DOWN_LEFT = (1, -1, math.sqrt(2))

    def __str__(self):
        if self == self.LEFT:
            return '<'
        elif self == self.RIGHT:
            return '>'
        elif self == self.UP:
            return '^'
        elif self == self.DOWN:
            return 'v'
        elif self == self.UP_RIGHT:
            return '/'
        elif self == self.UP_LEFT:
            return '\\'
        elif self == self.DOWN_RIGHT:
            return '\\'
        elif self == self.DOWN_LEFT:
            return '/'

    @property
    def cost(self):
        return self.value[2]

    @property
    def delta(self):
        return self.value[0], self.value[1]


def valid_actions(grid, current_node):
    """
    Returns a list of valid actions given a grid and current node.
    """
    valid = [Action.UP, Action.LEFT, Action.RIGHT, Action.DOWN, Action.UP_LEFT, Action.UP_RIGHT, Action.DOWN_LEFT, Action.DOWN_RIGHT]
    n, m = grid.shape[0] - 1, grid.shape[1] - 1
    x, y = current_node

    # check if the node is off the grid or
    # it's an obstacle

    if x - 1 < 0 or grid[x - 1, y] == 1:
        valid.remove(Action.UP)
    if x + 1 > n or grid[x + 1, y] == 1:
        valid.remove(Action.DOWN)
    if y - 1 < 0 or grid[x, y - 1] == 1:
        valid.remove(Action.LEFT)
    if y + 1 > m or grid[x, y + 1] == 1:
        valid.remove(Action.RIGHT)

    # diagonal actions

    if x - 1 < 0 or y + 1 > m or grid[x - 1, y + 1] == 1:
        valid.remove(Action.UP_RIGHT)
    if x - 1 < 0 or y - 1 < 0 or grid[x - 1, y - 1] == 1:
        valid.remove(Action.UP_LEFT)
    if x + 1 > n or y + 1 > m or grid[x + 1, y + 1] == 1:
        valid.remove(Action.DOWN_RIGHT)
    if x + 1 > n or y - 1 < 0 or grid[x + 1, y - 1] == 1:
        valid.remove(Action.DOWN_LEFT)

    return valid


def visualize_path(grid, path, start):
    sgrid = np.zeros(np.shape(grid), dtype=np.str)
    sgrid[:] = ' '
    sgrid[grid[:] == 1] = 'O'

    pos = start

    for a in path:
        da = a.value
        sgrid[pos[0], pos[1]] = str(a)
        pos = (pos[0] + da[0], pos[1] + da[1])
    sgrid[pos[0], pos[1]] = 'G'
    sgrid[start[0], start[1]] = 'S'
    return sgrid


def uniform_cost(grid, start, goal):
    # Initialize the starting variables
    path = []
    path_cost = 0
    queue = PriorityQueue()
    visited = set()

    branch = {}
    found = False
    queue.put((0, start))
    visited.add(start)

    while not queue.empty():
        # Remove the first element from the queue
        current_cost, current_node = queue.get()

        # Check if the current vertex corresponds to the goal state
        if current_node == goal:
            print('Found a path.')
            found = True
            break
        else:
            for action in valid_actions(grid, current_node):
                # determine the next_node using the action delta
                next_node = (current_node[0] + action.delta[0], current_node[1] + action.delta[1])
                # compute the new cost
                new_cost = current_cost + action.cost

                # Check if the new vertex has not been visited before.
                # If the node has not been visited you will need to
                # 1. Mark it as visited
                # 2. Add it to the queue
                if next_node not in visited:
                    visited.add(next_node)
                    queue.put((new_cost, next_node))
                    branch[next_node] = (new_cost, current_node, action)

    if found:
        # retrace steps
        path = []
        n = goal
        path_cost = branch[n][0]
        while branch[n][1] != start:
            path.append(branch[n][2])
            n = branch[n][1]
        path.append(branch[n][2])

    return path[::-1], path_cost


start = (0, 0)
goal = (4, 4)

grid = np.array([
    [0, 1, 0, 0, 0, 0],
    [0, 1, 0, 1, 0, 0],
    [0, 1, 0, 0, 0, 0],
    [0, 1, 0, 0, 1, 0],
    [0, 0, 0, 1, 0, 0],
])

path, path_cost = uniform_cost(grid, start, goal)
#print(path_cost, path)

# S -> start, G -> goal, O -> obstacle
vis_path = visualize_path(grid, path, start)
print(vis_path)


