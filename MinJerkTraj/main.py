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
from mavsdk.offboard import (OffboardError, VelocityNedYaw)


async def run():
    # start = [x,y,z,v,yaw]
    start = [0,0,0,10,np.deg2rad(0)]

    # goal = [x,y,z,v,yaw]
    goal = [94,76,20,0,np.deg2rad(30)]

    dist = math.sqrt((goal[0]-start[0])**2 + (goal[1]-start[1])**2 + (goal[2]-start[2])**2)

    avgVel = start[3]

    # time to reach the goal
    t_mission = dist/avgVel   # seconds
    print(f"mission time is: {t_mission}")

    xP,yP,zP,vP,yawP,time = minJerk.genTraj(start,goal,t_mission)

    ax.plot3D(xP, yP, zP, 'gray')
    plt.show()
    #fig.show()
    
    plt.plot(time,xP)
    plt.plot(time,yP)
    plt.plot(time,zP)
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
        await drone.offboard.set_position_ned(PositionNedYaw(xP[t],yP[t],-zP[t],yawP[t]))
        await asyncio.sleep(time[t+1] - time[t])

if __name__=="__main__":
    asyncio.run(run())


