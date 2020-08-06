# minimum jerk trajectory planning
# x = xi + (xf - xi)(10(t/d)**3 - 15(t/d)**4 + 6(t/d)**5)
# y = yi + (yf - yi)(10(t/d)**3 - 15(t/d)**4 + 6(t/d)**5)
# x_dot = (1/d)(xf - xi)(30(t/d)**2 - 60(t/d)**3 + 30(t/d)**4)
# y_dot = (1/d)(yf - yi)(30(t/d)**2 - 60(t/d)**3 + 30(t/d)**4)
# yaw = math.atan2(y_dot,x_dot)
# time (t) = [t0, t0 + dt, t0 + 2*dt, t0 + 3*dt ... t0 + n*dt]
# t is the current time index (set dt time interval)
# d is the total time to reach the goal location
import mavsdk 
import math
from mavsdk import System
import asyncio
from mavsdk.offboard import (OffboardError, PositionNedYaw)
import numpy as np
import minJerk
import matplotlib.pyplot as plt
from mpl_toolkits import mplot3d
fig = plt.figure()
ax = plt.axes(projection='3d')


async def run():
    # start = [x,y,v,yaw]
    start = [0,0,2,np.deg2rad(0)]

    # goal = [x,y,v,yaw]
    goal = [37,95,0,np.deg2rad(30)]

    dist = math.sqrt((goal[0]-start[0])**2 + (goal[1]-start[1])**2)

    avgVel = start[2]

    # time to reach the goal
    t_mission = dist/avgVel   # seconds
    print(f"mission time is: {t_mission}")

    xP,yP,vP,yawP,time = minJerk.genTraj(start,goal,t_mission)

    ax.plot3D(xP, yP, time, 'gray')
    plt.show()
    #fig.show()
    
    plt.plot(time,xP)
    plt.plot(time,yP)
    plt.plot(time,yawP)
    plt.show()
    plt.plot(time,vP)
    plt.show()

    drone = System()

    await drone.connect(system_address="udp://:14540")
    await drone.action.arm()
    await drone.offboard.set_position_ned(PositionNedYaw(0.0, 0.0, 0.0, 0.0))
    await drone.offboard.start()

    for t in range(len(time)-1):
        await drone.offboard.set_position_ned(PositionNedYaw(xP[t], yP[t], -10.0, yawP[t]))
        await asyncio.sleep(time[t+1] - time[t])

if __name__=="__main__":
    asyncio.run(run())


