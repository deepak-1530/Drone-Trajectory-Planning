# minimum jerk trajectory generation

# minimum jerk trajectory planning
# x = xi + (xf - xi)(10(t/d)**3 - 15(t/d)**4 + 6(t/d)**5)
# y = yi + (yf - yi)(10(t/d)**3 - 15(t/d)**4 + 6(t/d)**5)
# x_dot = (1/d)(xf - xi)(30(t/d)**2 - 60(t/d)**3 + 30(t/d)**4)
# y_dot = (1/d)(yf - yi)(30(t/d)**2 - 60(t/d)**3 + 30(t/d)**4)
# yaw = math.atan2(y_dot,x_dot)
# time (t) = [t0, t0 + dt, t0 + 2*dt, t0 + 3*dt ... t0 + n*dt]
# t is the current time index (set dt time interval)
# d is the total time to reach the goal location

import numpy as np
import math

def genTraj(start, goal, t_mission):
    x = []
    y = []
    z = []
    V =[]
    yaw = []
    dt = 0.1
    t = np.arange(0,t_mission+dt,dt)
    s2D = np.array([start[0],start[1],start[2]])
    g2D = np.array([goal[0],goal[1], goal[2]])
    
    for i in range(len(t)):
        pos = s2D + (g2D - s2D)*(10*(t[i]/t_mission)**3  - 15*(t[i]/t_mission)**4 + 6*(t[i]/t_mission)**5)
        print(pos)
        vel = (1.0/t_mission)*(g2D - s2D)*(30*(t[i]/t_mission)**2 - 60*(t[i]/t_mission)**3 + 30*(t[i]/t_mission)**4)
        yaw_ = math.atan2(vel[1], vel[0])*57.30
        print(yaw_)
        x.append(pos[0])
        y.append(pos[1])
        z.append(pos[2])
        V.append(math.sqrt((vel[0])**2 + (vel[1])**2))
        yaw.append(yaw_)

    return x,y,z,V,yaw,t