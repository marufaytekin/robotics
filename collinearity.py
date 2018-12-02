import time
import numpy as np


def point(p):
    return np.array([p[0], p[1], 1.])


def collinearity_float(p1, p2, p3, epsilon=1e-2):
    collinear = False
    # TODO: Add a third dimension of z=1 to each point
    p1 = point(p1)
    p2 = point(p2)
    p3 = point(p3)
    # TODO: Create the matrix out of three points
    mat = np.vstack((p1, p2, p3))
    # TODO: Calculate the determinant of the matrix.
    det = np.linalg.det(mat)
    # TODO: Set collinear to True if the determinant is less than epsilon

    if det < epsilon:
        collinear = True

    return collinear


def collinearity_int(p1, p2, p3):
    collinear = False
    # TODO: Calculate the determinant of the matrix using integer arithmetic
    x1 = p1[0]
    y1 = p1[1]
    x2 = p2[0]
    y2 = p2[1]
    x3 = p3[0]
    y3 = p3[1]

    det = x1 * (y2 - y3) + x2 * (y3 - y1) + x3 * (y1 - y2)

    # TODO: Set collinear to True if the determinant is equal to zero
    if det > 0:
        collinear = True

    return collinear


p1 = np.array([1, 2])
p2 = np.array([2, 3])
p3 = np.array([3, 4])

t1 = time.time()
collinear = collinearity_float(p1, p2, p3)
t_3D = time.time() - t1

t1 = time.time()
collinear = collinearity_int(p1, p2, p3)
t_2D = time.time() - t1
print(t_3D/t_2D)