# minimum snap trajectory generation
# give some waypoints and generate piecewise polynomials
# waypoints -> [0,0,10], [2,2,10], [4,6,10],[15,15,10]

# generate minimum snap trajectory
# states -> [x,y,z,phi] -> sig
# sig(t) = [x(t), y(t), z(t), phi(t)]
# set initial velocity -> 10 m/s
# time = distance/time = 5 seconds


# Study machine learning and deep learning, use keras for semantic segmentation
# various things are there -> Object detection, Instance Segmentation, Panoptic Segmentation
# Study machine learning from scratch as well
# see how to use machine learning in various applications -> localization, motion planning, trajectory planning

import numpy as np
import matplotlib.pyplot as plt

# Firstly create minimum jerk trajectory 

# Enable obstacle detection using depth camera over ROS -> Create ESDF Map -> Plan waypoints using OMPL -> Generate Polynomial Trajectory using numerical optimization

# Localization -> Visual Inertial Odometry -> Prefereably monocular + IMU, stereo + IMU.

# Solve for each axis independently

# pass the velocities and yaw

def minJerkTraj(Xinit, Xgoal):  # Xinit = [x0,y0,z0,v0], Ygoal = [xf,yf,zf,vf]
    