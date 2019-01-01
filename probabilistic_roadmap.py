# Probabilistic Roadmap
# In this notebook you'll expand on previous random sampling exercises by creating a graph from the points and running A*.
#
# Load the obstacle map data
# Sample nodes (use KDTrees here)
# Connect nodes (use KDTrees here)
# Visualize graph
# Define heuristic
# Define search method
# Execute and visualize

import time
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from shapely.geometry import Polygon, Point
from grid import create_grid
import networkx as nx
import numpy.linalg as LA
from sklearn.neighbors import KDTree
from shapely.geometry import Polygon, Point, LineString
from sampling import *
from queue import PriorityQueue

plt.rcParams['figure.figsize'] = 12, 12

# This is the same obstacle data from the previous lesson.
filename = 'colliders.csv'
data = np.loadtxt(filename, delimiter=',', dtype='Float64', skiprows=2)
print(data)

from sampling import Sampler

# TODO: sample points randomly
# then use KDTree to find nearest neighbor polygon
# and test for collision
sampler = Sampler(data)
polygons = sampler._polygons

nodes = sampler.sample(300)
print(len(nodes))


# TODO: connect nodes
# Suggested method
# 1) cast nodes into a graph called "g" using networkx
# 2) write a method "can_connect()" that:
# casts two points as a shapely LineString() object
# tests for collision with a shapely Polygon() object
# returns True if connection is possible, False otherwise
# 3) write a method "create_graph()" that:
# defines a networkx graph as g = Graph()
# defines a tree = KDTree(nodes)
# test for connectivity between each node and
# k of it's nearest neighbors
# if nodes are connectable, add an edge to graph
# Iterate through all candidate nodes!

def heuristic(n1, n2):
    h = LA.norm(np.array(n2) - np.array(n1))
    return h


def can_connect(n1, n2, polygons):
    l = LineString([n1, n2])
    for p in polygons:
        if p.crosses(l) and p.height >= min(n1[2], n2[2]):
            return False
    return True


def create_graph(nodes, polygons, k):
    g = nx.Graph()
    tree = KDTree(nodes)
    g.root_graph.edges
    for n1 in nodes:
        # for each node connect try to connect to k nearest nodes
        idxs = tree.query([n1], k, return_distance=False)[0]

        for idx in idxs:
            n2 = nodes[idx]
            if n2 == n1:
                continue

            if can_connect(n1, n2, polygons):
                g.add_edge(n1, n2, weight=1)
    return g


# 1. Find the closest point in the graph to our current location,
# same thing for the goal location.
def closest_point(graph, current_point):
    d_min = 1000000
    curr_closest_point = None
    for point in graph.nodes:
        d = LA.norm(np.array(point) - np.array(current_point))
        if d < d_min:
            curr_closest_point = point
            d_min = d
    return curr_closest_point


def a_star(graph, heuristic, start, goal):
    """Modified A* to work with NetworkX graphs."""

    # TODO: complete

    path = []
    queue = PriorityQueue()
    queue.put((0, start))
    visited = set(start)

    branch = {}
    found = False

    while not queue.empty():
        item = queue.get()
        current_cost = item[0]
        current_node = item[1]

        if current_node == goal:
            print('Found a path.')
            found = True
            break
        else:
            for next_node in graph[current_node]:
                cost = graph.edges[current_node, next_node]['weight']
                new_cost = current_cost + cost + heuristic(next_node, goal)

                if next_node not in visited:
                    visited.add(next_node)
                    queue.put((new_cost, next_node))

                    branch[next_node] = (new_cost, current_node)
    path = []
    path_cost = 0
    if found:

        # retrace steps
        path = []
        n = goal
        path_cost = branch[n][0]
        while branch[n][1] != start:
            path.append(branch[n][1])
            n = branch[n][1]
        path.append(branch[n][1])

    return path[::-1], path_cost


# start = list(g.nodes)[0]
# print ("start: ", start)
# k = np.random.randint(len(g.nodes))
# print(k, len(g.nodes))
# goal = list(g.nodes)[k]
# print("goal: ", goal)
#

import time

t0 = time.time()
g = create_graph(nodes, polygons, 10)
print('graph took {0} seconds to build'.format(time.time() - t0))
print("Number of edges", len(g.edges))

grid = create_grid(data, sampler._zmax, 1)

#start = (25, 100)
#goal = (750., 370.)
# start = (-122.39745, 37.79248, 0)
# goal = (-122.401427, 37.797488, 0.045)
#
#
# start_closest = closest_point(g, start)
# print("start: ", start, "start closest: ", start_closest)
# goal_closest = closest_point(g, goal)
# print("goal: ", goal, "goal closest: ", goal_closest)

start = list(g.nodes)[0]
print("start: ", start)
k = np.random.randint(len(g.nodes))
goal = list(g.nodes)[k]
print("goal: ", goal)


fig = plt.figure()

plt.imshow(grid, cmap='Greys', origin='lower')

nmin = np.min(data[:, 0])
emin = np.min(data[:, 1])

# draw start and goal
plt.plot(start[1] - emin, start[0] - nmin, 'rx', markersize=8)
plt.plot(goal[1] - emin, goal[0] - nmin, 'gx', markersize=8)

# draw edges
for (n1, n2) in g.edges:
    plt.plot([n1[1] - emin, n2[1] - emin], [n1[0] - nmin, n2[0] - nmin], 'black', alpha=0.5)

# draw all nodes
for n1 in nodes:
    plt.scatter(n1[1] - emin, n1[0] - nmin, c='blue')

# draw connected nodes
for n1 in g.nodes:
    plt.scatter(n1[1] - emin, n1[0] - nmin, c='red')

plt.xlabel('NORTH')
plt.ylabel('EAST')

plt.show()

# search for path
path, cost = a_star(g, heuristic, start, goal)
print(len(path), path)

path_pairs = zip(path[:-1], path[1:])
for (n1, n2) in path_pairs:
    print(n1, n2)


fig = plt.figure()

plt.imshow(grid, cmap='Greys', origin='lower')

nmin = np.min(data[:, 0])
emin = np.min(data[:, 1])

# draw nodes
for n1 in g.nodes:
    plt.scatter(n1[1] - emin, n1[0] - nmin, c='red')

# draw edges
for (n1, n2) in g.edges:
    plt.plot([n1[1] - emin, n2[1] - emin], [n1[0] - nmin, n2[0] - nmin], 'black')

# DONE: add code to visualize the path
path_pairs = zip(path[:-1], path[1:])
for (n1, n2) in path_pairs:
    plt.plot([n1[1] - emin, n2[1] - emin], [n1[0] - nmin, n2[0] - nmin], 'green', linewidth=2)

# draw start and goal
plt.plot(start[1] - emin, start[0] - nmin, 'rx', markersize=8)
plt.plot(goal[1] - emin, goal[0] - nmin, 'gx', markersize=8)

plt.xlabel('NORTH')
plt.ylabel('EAST')

plt.show()
