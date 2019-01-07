#!/usr/bin/env python
# coding: utf-8

# # Non Linear Controller

import matplotlib.pylab as pylab

from drone import Drone2D
from controllers import NonLinearCascadingController

import trajectories
import simulate
import plotting

pylab.rcParams['figure.figsize'] = 10, 10


# #### TODO - Implement 2D controller with NON-LINEAR control equations.
# 
# The non-linear equations of motion are shown below.
# 
# $$\begin{align}
# \ddot{z} &= g - \frac{u_1}{m}\cos\phi
# \\
# \\
# \ddot{y} &= \frac{u_1}{m}\sin\phi
# \\
# \\
# \ddot{\phi} &= \frac{u_2}{I_x}
# \end{align}$$
# 
# Recalling that $$ \bar{u_1} ==\ddot{z} ; \bar{u_2}  == \ddot{\phi} ;  \phi_{\text{command}} == \phi $$ 
# $$
# $$
# Then these equations can be solved for $u_1$, $\phi_{\text{command}}$, and $u_2$. 
# 
# $$\begin{align}
# u_1 &= \frac{m(g - \bar{u_1})}{\cos\phi}
# \\
# \\
# \phi_{\text{command}} &= \sin^{-1}\left(\frac{m\ddot{y}_{\text{target}}}{u_1}\right)
# \\
# \\
# u_2 &= I_x \bar{u}_2 
# \end{align}$$
# 
# The first equation will be useful when implementing `altitude_controller`, the second when implementing
# `lateral_controller`, and the third when implementing `attitude_controller`.
# 
# Note that $\ddot{y}_{\text{target}}$ is like $\bar{u}_1$ or $\bar{u}_2$. It comes from PD control on the
# controller's inputs.


# The flight path we'll use to test our controller is a figure 8 described as follows:
# $$
# \begin{align}
# z & = a_z \sin{\omega_z t} \\
# y & = a_y \cos{\omega_y t}
# \end{align}
# $$
# 
# where $\omega_y = \omega_z / 2$. 
# 
# **NOTE on tuning:**
# <br></br>
# 
# A good approach is to start with the inner loop, the ATTITUDE controller parameters (phi_k_p, phi_k_d) and the move
#  forward with the LATERAL and ALTITUDE controller. Also, the instruction for the "linear" implementation can be
# useful here.


# TESTING CELL 
# 
# Note - this cell will only give nice output when your code
#  is working AND you've tuned your parameters correctly.
#  you might find it helpful to come up with a strategy
#  for first testing the inner loop controller and THEN 
#  testing the outer loop.
#
# Run this cell when you think your controller is ready!
#
# You'll have to tune the controller gains to get good results.

#### CONTROLLER GAINS (TUNE THESE) ######

z_k_p = 0.1
z_k_d = 10.0
y_k_p = 1
y_k_d = 10.0
phi_k_p = 150.0
phi_k_d = 50.0

#########################################

drone = Drone2D()

# INSTANTIATE CONTROLLER
non_linear_controller = NonLinearCascadingController(
    drone.m,
    drone.I_x,
    z_k_p=z_k_p,
    z_k_d=z_k_d,
    y_k_p=y_k_p,
    y_k_d=y_k_d,
    phi_k_p=phi_k_p,
    phi_k_d=phi_k_d
)

# TRAJECTORY PARAMETERS (you don't need to change these)
total_time = 30.0
omega_z = 1.0  # angular frequency of figure 8

# GENERATE FIGURE 8
z_traj, y_traj, t = trajectories.figure_8(omega_z, total_time, dt=0.02)
z_path, z_dot_path, z_dot_dot_path = z_traj
y_path, y_dot_path, y_dot_dot_path = y_traj

# SIMULATE MOTION
linear_history = simulate.zy_flight(z_traj,
                                    y_traj,
                                    t,
                                    non_linear_controller,
                                    inner_loop_speed_up=10)
# PLOT RESULTS
pylab.rcParams['figure.figsize'] = 10, 10
plotting.plot_zy_flight_path(z_path, y_path, linear_history)
