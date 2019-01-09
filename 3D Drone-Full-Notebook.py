#!/usr/bin/env python
# coding: utf-8

# # 3D Drone 
# ---
# In this lesson, we will develop a controller for a drone in a 3D environment. The lesson is divided into two parts. In the first part, you will develop a class which will capture and describe the drone dynamics. In the second part, you will develop a control system for the drone. At the end of the exercise, you will perform a simple three-dimensional flight and will observe how your controller performs and if the drone follows the desired path. 
# 
# <div class="container" style="width: 100%;">
#  <div class="D1 col-sm-6" style="width: 40%;">
#    <img src="drone1_1.png" height="100">
#  </div>
#  <div class="D2 col-sm-6" style="width: 50%;">
#    <img src="Drone2.png" height="300">
#  </div>
# </div>
# 
# ### Drone dynamics
# Let's remember the movement that a drone can perform. It can move along the three position axis $x$, $y$ and $z$. We choose the $z$ axis to be directed downward as in previous exercises. The drone also can roll along the $x$ axis, pitch along the $y$ axis and yaw along the $z$ axis. The directions of the rotations are depicted in the images above. 
# In this exercise, we will have to track the drone's change in attitude in two coordinate systems. One in an inertial frame (world frame) relative to the surroundings and the second in the body frame attached to the drone itself. You will be responsible for implementing the proper conversion functions and tracking the necessary quantities in the appropriate frames. 

# In[51]:




import numpy as np 
import math
from math import sin, cos, tan, sqrt
import matplotlib.pyplot as plt
import matplotlib.pylab as pylab
from mpl_toolkits.mplot3d import Axes3D
import random

from solution import UDACITYDroneIn3D, UDACITYController
import testing

pylab.rcParams['figure.figsize'] = 10, 10



# # Flight planning
# 
# In order to test the developed drone dynamics and the controller, we will execute simple three-dimensional flight with changing yaw angle. 
# 
# _Keep in mind that the desired flight path needs to have an analytical form (differentiable for the entirety of the path)._
# 
# The selected test path is a figure 8 in three dimensions with yaw that keeps the drone always facing along the motion direction. 
# 
# $$
# \begin{align}
# x &= \sin{\omega_x t} \\
# y &= \cos{\omega_y t} \\
# z &= \cos{\omega_z t} \\
# \end{align}
# $$
# $\omega_z = \omega_y = \omega_x/2$.

# In[80]:


total_time = 20.0
dt = 0.01

t=np.linspace(0.0,total_time,int(total_time/dt))

omega_x = 0.8
omega_y = 0.4
omega_z = 0.4

a_x = 1.0 
a_y = 1.0
a_z = 1.0

x_path= a_x * np.sin(omega_x * t) 
x_dot_path= a_x * omega_x * np.cos(omega_x * t)
x_dot_dot_path= -a_x * omega_x**2 * np.sin(omega_x * t)

y_path= a_y * np.cos(omega_y * t)
y_dot_path= -a_y * omega_y * np.sin(omega_y * t)
y_dot_dot_path= -a_y * omega_y**2 * np.cos(omega_y * t)

z_path= a_z * np.cos(omega_z * t)
z_dot_path= -a_z * omega_z * np.sin(omega_z * t)
z_dot_dot_path= - a_z * omega_z**2 * np.cos(omega_z * t)

psi_path= np.arctan2(y_dot_path,x_dot_path)

fig = plt.figure()
ax = fig.gca(projection='3d')
ax.plot(x_path, y_path, z_path)
plt.title('Flight path').set_fontsize(20)
ax.set_xlabel('$x$ [$m$]').set_fontsize(20)
ax.set_ylabel('$y$ [$m$]').set_fontsize(20)
ax.set_zlabel('$z$ [$m$]').set_fontsize(20)
plt.legend(['Planned path'],fontsize = 14)
plt.figure(figsize=(10,10))
plt.show()


# In[81]:


fig = plt.figure()
ax = fig.gca(projection='3d')

u = np.cos(psi_path)
v = np.sin(psi_path)
w = np.zeros(psi_path.shape)
for i in range(0,z_path.shape[0],20):
    ax.quiver(x_path[i], y_path[i], z_path[i], u[i], v[i], w[i], length=0.2, normalize=True,color='green')

plt.title('Flight path').set_fontsize(20)
ax.set_xlabel('$x$ [$m$]').set_fontsize(20)
ax.set_ylabel('$y$ [$m$]').set_fontsize(20)
ax.set_zlabel('$z$ [$m$]').set_fontsize(20)
plt.legend(['Planned yaw',],fontsize = 14)

plt.show()


# ### Plotting the drone's headings

# # Executing the flight
# 
# In this section, we will set up the entire system and will connect the drone object with the controller object. Next, execute the flight and compare the desired path with the executed one. 

# In[82]:


# how fast the inner loop (Attitude controller) performs calculations 
# relative to the outer loops (altitude and position controllers).
inner_loop_relative_to_outer_loop = 10

# creating the drone object
drone = UDACITYDroneIn3D()
 
# creating the control system object 

control_system = UDACITYController(z_k_p=2.0,
                            z_k_d=1.0, 
                            x_k_p=6.0,
                            x_k_d=4.0,
                            y_k_p=6.0,
                            y_k_d=4.0,
                            k_p_roll=8.0,
                            k_p_pitch=8.0,
                            k_p_yaw=8.0,
                            k_p_p=20.0,
                            k_p_q=20.0,
                            k_p_r=20.0)



# declaring the initial state of the drone with zero
# height and zero velocity 

drone.X = np.array([x_path[0],
                               y_path[0],
                               z_path[0],
                               0.0,
                               0.0,
                               psi_path[0],
                               x_dot_path[0],
                               y_dot_path[0],
                               z_dot_path[0],
                               0.0,
                               0.0,
                               0.0])

# arrays for recording the state history, 
# propeller angular velocities and linear accelerations
drone_state_history = drone.X
omega_history = drone.omega
accelerations = drone.linear_acceleration()
accelerations_history= accelerations
angular_vel_history = drone.get_euler_derivatives()

# executing the flight
for i in range(0,z_path.shape[0]):
    
    rot_mat = drone.R()

    c = control_system.altitude_controller(z_path[i],
                                           z_dot_path[i],
                                           z_dot_dot_path[i],
                                           drone.X[2],
                                           drone.X[8],
                                           rot_mat)
    
    b_x_c, b_y_c = control_system.lateral_controller(x_path[i],
                                                     x_dot_path[i],
                                                     x_dot_dot_path[i],
                                                     drone.X[0],
                                                     drone.X[6],
                                                     y_path[i],
                                                     y_dot_path[i],
                                                     y_dot_dot_path[i],
                                                     drone.X[1],
                                                     drone.X[7],
                                                     c) 
    
    for _ in range(inner_loop_relative_to_outer_loop):
        
        rot_mat = drone.R()
        
        angular_vel = drone.get_euler_derivatives()
        
        u_bar_p, u_bar_q, u_bar_r = control_system.attitude_controller(
            b_x_c,
            b_y_c,
            psi_path[i],
            drone.psi,
            drone.X[9],
            drone.X[10],
            drone.X[11],
            rot_mat)
        
        drone.set_propeller_angular_velocities(c, u_bar_p, u_bar_q, u_bar_r)
        
        drone_state = drone.advance_state(dt/inner_loop_relative_to_outer_loop)
        
    # generating a history of the state history, propeller angular velocities and linear accelerations
    drone_state_history = np.vstack((drone_state_history, drone_state))
    
    omega_history=np.vstack((omega_history,drone.omega))
    accelerations = drone.linear_acceleration()
    accelerations_history= np.vstack((accelerations_history,accelerations))
    angular_vel_history = np.vstack((angular_vel_history,drone.get_euler_derivatives()))
    


fig = plt.figure()
ax = fig.gca(projection='3d')
ax.plot(x_path, y_path, z_path,linestyle='-',marker='.',color='red')
ax.plot(drone_state_history[:,0],
         drone_state_history[:,1],
         drone_state_history[:,2],
         linestyle='-',color='blue')

plt.title('Flight path').set_fontsize(20)
ax.set_xlabel('$x$ [$m$]').set_fontsize(20)
ax.set_ylabel('$y$ [$m$]').set_fontsize(20)
ax.set_zlabel('$z$ [$m$]').set_fontsize(20)
plt.legend(['Planned path','Executed path'],fontsize = 14)

ax.set_xlim(-1, 1)
ax.set_ylim(-1, 1)
ax.set_zlim(-1, 1)

plt.show()


# # Flight path comparison 
# 
# Comparing the desired heading and the actual heading (Yaw angle).

# In[47]:


fig = plt.figure()
ax = fig.gca(projection='3d')

u = np.cos(psi_path)
v = np.sin(psi_path)
w = np.zeros(psi_path.shape)

drone_u = np.cos(drone_state_history[:,5])
drone_v = np.sin(drone_state_history[:,5])
drone_w = np.zeros(psi_path.shape)

for i in range(0,z_path.shape[0],20):
    ax.quiver(x_path[i], y_path[i], z_path[i], u[i], v[i], w[i], length=0.2, normalize=True,color='red')
    ax.quiver(drone_state_history[i,0], 
              drone_state_history[i,1], 
              drone_state_history[i,2], 
              drone_u[i], drone_v[i], drone_w[i], 
              length=0.2, normalize=True,color='blue')
ax.set_ylim(-1, 1)
ax.set_zlim(-1, 1)
plt.title('Flight path').set_fontsize(20)
ax.set_xlabel('$x$ [$m$]').set_fontsize(20)
ax.set_ylabel('$y$ [$m$]').set_fontsize(20)
ax.set_zlabel('$z$ [$m$]').set_fontsize(20)
plt.legend(['Planned yaw','Executed yaw'],fontsize = 14)

plt.show()


# Calculating the error in position $\epsilon^2(t) = \left(x_t(t)-x_a(t)\right)^2 + \left(y_t(t)-y_a(t)\right)^2+ \left(z_t(t)-z_a(t)\right)^2$.

# In[48]:


err= np.sqrt((x_path-drone_state_history[:-1,0])**2 
             +(y_path-drone_state_history[:-1,1])**2 
             +(y_path-drone_state_history[:-1,2])**2)


plt.plot(t,err)
plt.title('Error in flight position').set_fontsize(20)
plt.xlabel('$t$ [$s$]').set_fontsize(20)
plt.ylabel('$e$ [$m$]').set_fontsize(20)
plt.show()


# Plotting the angular velocities of the propellers in time. 

# In[49]:


plt.plot(t,-omega_history[:-1,0],color='blue')
plt.plot(t,omega_history[:-1,1],color='red')
plt.plot(t,-omega_history[:-1,2],color='green')
plt.plot(t,omega_history[:-1,3],color='black')

plt.title('Angular velocities').set_fontsize(20)
plt.xlabel('$t$ [$s$]').set_fontsize(20)
plt.ylabel('$\omega$ [$rad/s$]').set_fontsize(20)
plt.legend(['P 1','P 2','P 3','P 4' ],fontsize = 14)

plt.grid()
plt.show()


# Plotting the Yaw angle of the drone in time. 

# In[50]:


plt.plot(t,psi_path,marker='.')
plt.plot(t,drone_state_history[:-1,5])
plt.title('Yaw angle').set_fontsize(20)
plt.xlabel('$t$ [$s$]').set_fontsize(20)
plt.ylabel('$\psi$ [$rad$]').set_fontsize(20)
plt.legend(['Planned yaw','Executed yaw'],fontsize = 14)
plt.show()


# [Solution](/notebooks/solution.py)

# In[ ]:




