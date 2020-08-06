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
    V =[]
    yaw = []
    dt = 0.1
    t = np.arange(0,t_mission+dt,dt)
    for i in range(len(t)):
        x_ = start[0] + (goal[0] - start[0])*(10*(t[i]/t_mission)**3  - 15*(t[i]/t_mission)**4 + 6*(t[i]/t_mission)**5)
        y_ = start[1] + (goal[1] - start[1])*(10*(t[i]/t_mission)**3  - 15*(t[i]/t_mission)**4 + 6*(t[i]/t_mission)**5)
        xV_ = (1.0/t_mission)*(goal[0] - start[0])*(30*(t[i]/t_mission)**2 - 60*(t[i]/t_mission)**3 + 30*(t[i]/t_mission)**4)
        yV_ = (1.0/t_mission)*(goal[1] - start[1])*(30*(t[i]/t_mission)**2 - 60*(t[i]/t_mission)**3 + 30*(t[i]/t_mission)**4)
        print([xV_, yV_])
        yaw_ = math.atan2(yV_, xV_)*57.30
        print(yaw_)
        x.append(x_)
        y.append(y_)
        V.append(math.sqrt((xV_)**2 + (yV_)**2))
        yaw.append(yaw_)
    
    return x,y,V,yaw,t