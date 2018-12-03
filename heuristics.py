import numpy as np


# Manhattan distance.
def manhattan(position, goal_position):
    h = abs(position[0] - goal_position[0]) + abs(position[1] - goal_position[1])
    return h


# Euclidean distance.
def euclidean(position, goal_position):
    h = np.sqrt((position[0] - goal_position[0]) ** 2 + (position[1] - goal_position[1]) ** 2)
    return h
