# Import numpy, Enum and Queue
import numpy as np
from enum import Enum
from queue import Queue


class Action(Enum):
    # Actions are tuples corresponding to movements in (i, j)
    LEFT = (0, -1)
    RIGHT = (0, 1)
    UP = (-1, 0)
    DOWN = (1, 0)

    # Define string characters for each action
    def __str__(self):
        if self == self.LEFT:
            return '<'
        elif self == self.RIGHT:
            return '>'
        elif self == self.UP:
            return '^'
        elif self == self.DOWN:
            return 'v'


# Define a function that returns a list of valid actions
# through the grid from the current node
def valid_actions(grid, current_node):
    """
    Returns a list of valid actions given a grid and current node.
    """
    # First define a list of all possible actions
    valid = [Action.UP, Action.LEFT, Action.RIGHT, Action.DOWN]
    # Retrieve the grid shape and position of the current node
    n, m = grid.shape[0] - 1, grid.shape[1] - 1
    x, y = current_node

    # check if the node is off the grid or it's an obstacle
    # If it is either, remove the action that takes you there
    if x - 1 < 0 or grid[x - 1, y] == 1:
        valid.remove(Action.UP)
    if x + 1 > n or grid[x + 1, y] == 1:
        valid.remove(Action.DOWN)
    if y - 1 < 0 or grid[x, y - 1] == 1:
        valid.remove(Action.LEFT)
    if y + 1 > m or grid[x, y + 1] == 1:
        valid.remove(Action.RIGHT)

    return valid


# Define a function to visualize the path
def visualize_path(grid, path, start):
    """
    Given a grid, path and start position
    return visual of the path to the goal.

    'S' -> start
    'G' -> goal
    'O' -> obstacle
    ' ' -> empty
    """
    # Define a grid of string characters for visualization
    sgrid = np.zeros(np.shape(grid), dtype=np.str)
    sgrid[:] = ' '
    sgrid[grid[:] == 1] = 'O'

    pos = start
    # Fill in the string grid
    for a in path:
        da = a.value
        sgrid[pos[0], pos[1]] = str(a)
        pos = (pos[0] + da[0], pos[1] + da[1])
    sgrid[pos[0], pos[1]] = 'G'
    sgrid[start[0], start[1]] = 'S'
    return sgrid


def breadth_first(grid, start, goal):
    q = Queue()
    branch = {}
    visited = set()
    found = False
    q.put(start)
    visited.add(start)

    while not q.empty():
        current_node = q.get()
        if current_node == goal:
            print('Found a path.')
            found = True
            break
        else:
            actions = valid_actions(grid, current_node)
            for action in actions:
                da = action.value
                next_node = (current_node[0] + da[0], current_node[1] + da[1])
                if next_node not in visited:
                    visited.add(next_node)
                    q.put(next_node)
                    branch[next_node] = (current_node, action)

    # Now, if you found a path, retrace your steps through
    # the branch dictionary to find out how you got there!
    path = []
    if found:
        # retrace steps
        path = []
        n = goal
        while branch[n][0] != start:
            path.append(branch[n][1])
            n = branch[n][0]
        path.append(branch[n][1])

    return path[::-1]


# Define a start and goal location
start = (0, 0)
goal = (4, 4)

# Define your grid-based state space of obstacles and free space
grid = np.array([
    [0, 1, 0, 0, 0, 0],
    [0, 1, 0, 0, 0, 0],
    [0, 1, 0, 1, 0, 0],
    [0, 0, 0, 1, 1, 0],
    [0, 0, 0, 1, 0, 0],
])

path = breadth_first(grid, start, goal)

# S -> start, G -> goal, O -> obstacle
visual_path = visualize_path(grid, path, start)
print(visual_path)