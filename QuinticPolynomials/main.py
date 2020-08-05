# quintic polynomial trajectory plannerr
# given start and end conditions -> calculate the polynomial
# use the given time window to calculate the parameters
# s(t) = a0 + a1*t + a2*t**2 + a3*t***3 + a4*t****4 + a5*t*****5

# generate multiple trajectories and select the one with minimum jerk and acceleration within limits

import numpy as np
import mavsdk
from mavsdk import System
import matplotlib.pyplot as plt
from QuiPoly import QuinticPolynomial
import math
import asyncio
from mavsdk.offboard import (OffboardError, PositionNedYaw)


# minimum and maxium times to reach the goal
minT = 5.0 # seconds
maxT = 50.0 # seconds
stepT = minT  # check in the intervals of 5 seconds i.e. best traj in 5 seconds, 10 seconds, 15 seconds etc.


# states -> position, velocity, yaw, acceleration
# starting -> sP, sV, sYaw, sAccel
# goal     -> gP, gV, gYaw, gAccel
# Time to reach -> (5,50)
# goal --- > minimize jerk

def quintic_polynomials_planner(sX, sY, sYaw, sV, sA, gX, gY,  gYaw, gV, gA, maxA, maxJerk, dt):
    # it returns -> time array, x array, y array, yaw array, velocity array, acceleration array, jerk array

    # get the velocity components
    sVx = sV*math.cos(sYaw)
    sVy = sV*math.sin(sYaw)
    sAx = sA*math.cos(sYaw)
    sAy = sA*math.sin(sYaw)

    gVx = gV*math.cos(gYaw)
    gVy = gV*math.sin(gYaw)
    gAx = gA*math.cos(gYaw)
    gAy = gA*math.sin(gYaw)

    
    # now calculate the quintic polynomial trajectory 
    for T in np.arange(minT, maxT, stepT):
        xqp = QuinticPolynomial(sX,sVx,sAx,gX,gVx,gAx,T)
        yqp = QuinticPolynomial(sY,sVy,sAy,gY,gVy,gAy,T)

        time, rx, ry, ryaw, rv, ra, rj = [], [], [], [], [], [], []

        # trajectories calculated.... now generate all the variables
        for t in np.arange(0.0,T+dt,dt):
            time.append(t)
            rx.append(xqp.calc_point(t))
            ry.append(yqp.calc_point(t))

            vx = xqp.calc_first_derivative(t)
            vy = yqp.calc_first_derivative(t)
            rv.append(math.sqrt(vx**2 + vy**2))
            ryaw.append(math.atan2(vy,vx))

            ax = xqp.calc_second_derivative(t)
            ay = yqp.calc_second_derivative(t)
            a = math.sqrt(ax**2 + ay**2)
            if len(rv) >= 2 and rv[-1] - rv[-2] < 0.0:
                a = a*(-1)
            ra.append(a)

            jx = xqp.calc_third_derivative(t)
            jy = yqp.calc_third_derivative(t)
            j = np.hypot(jx, jy)
            if len(ra) >= 2 and ra[-1] - ra[-2] < 0.0:
                j *= -1
            rj.append(j)

        if max([abs(i) for i in ra]) <= maxA and max([abs(i) for i in rj]) <= maxJerk:
            print("find path!!")
            break
        #else:
            #time, rx, ry, ryaw, rv, ra, rj = [], [], [], [], [], [], []

    return time, rx, ry, ryaw, rv, ra, rj


async def execTraj(drone,x,y,yaw,v,a,time):
    print("drone in offboard mode now")

    for t in range(len(time)-1):
        await drone.offboard.set_position_ned(PositionNedYaw(x[t], y[t], -10.0, yaw[t]*57.3))
        print(x[t],y[t],yaw[t])
        await asyncio.sleep(time[t+1]-time[t])
    return

    
async def main():
    drone = System()
    await drone.connect(system_address="udp://:14540")
    await drone.action.arm()
    await drone.offboard.set_position_ned(PositionNedYaw(0.0, 0.0, 0.0, 0.0))
    await drone.offboard.start()

    sX = 0
    sY = 0
    sYaw = np.deg2rad(0)
    sV = 10.0
    sA = 0.1

    gX = 100
    gY = 50
    gYaw = np.deg2rad(60)  # 30 degrees of yaw with respect to the starting point
    gV = 0.0               # 1m/s goal velocity
    gA = 0.0           # 0 acceleration upon reaching the goal

    maxA = 2.5
    maxJerk = 0.1

    dt = 0.1  # time step

    time, x, y, yaw, v, a, j = quintic_polynomials_planner(
        sX, sY, sYaw, sV, sA, gX, gY, gYaw, gV, gA, maxA, maxJerk, dt)
    plt.plot(x,y)
    plt.show()
    plt.plot(time,j)
    plt.show()
    plt.plot(time,v)
    plt.show()
    plt.plot(time,yaw)
    plt.show()
    plt.plot(time,a)
    plt.show()
    print(len(x))
    print("quintic spline calculated")

    await execTraj(drone,x,y,yaw,v,a,time)
    return

if __name__=="__main__":
    asyncio.run(main())


