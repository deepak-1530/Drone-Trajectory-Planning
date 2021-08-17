import mavsdk
import asyncio
import numpy as np
from mavsdk import System
import math
from mavsdk.offboard import (OffboardError, PositionNedYaw)
import matplotlib.pyplot as plt

####################################################
# DRONE_STATE = [Xd,Yd,Zd,Vd_X,Vd_Y,Vd_Z]          #
# GOAL_STATE = [Xg,Yg,Zg,Vg_X,Vg_Y,Vg_Z]           #
# T = DISTANCE/VELOCITY                            #
# Velocity at goal location = 0                    #
# Velocity at the beginning of FTC = 5m/s          #       
####################################################

DRONE_STATE = [] # [[x, y, z], [vx, vy, vz]]

GOAL_STATE =  [] # [[x, y, z], [vx, vy, vz]]

COORDINATES = [[[50,100,-10],[0,0,0]], [[100,50,-10],[0,0,0]], [[71,20,-10],[0,0,0]]]

#############################################################
# Calculate trajectory -> Each axis is treated independently
#############################################################

def calcPath(Sx, Sv, Gx, Gv, T):
    ##########################################################
    # Equation of the curve is a0 + a1*t + a2*t**2 + a3*t**3 #
    # x(t) = a0 + a1*t + a2*t^2 + a3*t^3                     #    
    # x'(t) = a1 + 2*a2*t + 3*a3*t^2                         #
    # x(0) = a0, x'(0) = a1 or a0 = x(0) & a1 = x'(0)        #
    # x(T) - a0 - a1*T = a2*T**2 + a3*T**3                   #
    # x(T) - a1        = 2*a2*T  + 3*a3*T**2                 #
    # B = [x(T) - a0 - a1*T, x(T) - a1]  (2 X 1)             #
    # A = [[T**2, T**3],[2*T, 3*T**2]]   (2 X 2)             #
    # z = [a2,a3]                                            #
    # z = inverse(A)*B                                       #
    ##########################################################
    print(f"Starting and goal states are: {Sx, Sv, Gx, Gv}")
    a0 = Sx
    a1 = Sv
    A = np.array([[T**2, T**3],[2*T, 3*T**2]])
    B = np.array([Gx - a0 - a1*T, Gv - a1])
    z = np.linalg.solve(A, B)
    a2,a3 = z[0],z[1]
    pos, time = [],[]
    dt = 0.5
    for t in np.arange(0.0,T+dt,dt):
        pos.append(a0 + a1*t + a2*t**2 + a3*t**3)
        time.append(t)

    return pos, time


##################################
# Publish user's location in NED #
##################################
async def publish_target_Coord():
    global GOAL_STATE
    global COORDINATES
    for i in range(len(COORDINATES)):
        GOAL_STATE = COORDINATES[i]
        await asyncio.sleep(12)


#####################################################
# Calculate trajectory using Drone position, velocity 
# and Goal position, velocity
#####################################################
async def getTrajectory(droneState, goalState):
    path = []
    t = []
    distance = math.sqrt((droneState[0][0] - goalState[0][0])**2 + (droneState[0][1] - goalState[0][1])**2 + (droneState[0][2] - goalState[0][2])**2)

    #####################################################################
    # check if the drone velocity is 0, meaning first target location   #    
    # In this case -> set the drone velocity to 5 m/s and calculate the # 
    # time required to reach the goal                                   #
    ######################################################################
     
    if (math.sqrt(droneState[1][0]**2 + droneState[1][1]**2 + droneState[1][2]**2) < 0.5):
        droneVel = 5.0
    else:
        droneVel = math.sqrt(droneState[1][0]**2 + droneState[1][1]**2 + droneState[1][2]**2)
    
    T = distance/droneVel

    for i in range(2):
        p,t = calcPath(droneState[0][i],droneState[1][i],goalState[0][i], goalState[1][i], T)
        path.append(p)

    return path,t
        
##########################################
# Get Drone Position and Velocity in NED #
##########################################
async def getDroneState(drone):
    global DRONE_STATE
    async for state in drone.telemetry.position_velocity_ned():
        DRONE_STATE = [[state.position.north_m, state.position.east_m, state.position.down_m], [state.velocity.north_m_s, state.velocity.east_m_s, state.velocity.down_m_s]]
        return DRONE_STATE


##################################################################################
# Check difference between current goal location and new published goal location #
##################################################################################
async def checkDiff(goalState):
    global GOAL_STATE
    diff = np.array(goalState) - np.array(GOAL_STATE)
    diff = math.sqrt(diff[0][1]**2 + diff[0][1]**2 + diff[0][2]**2)
    if(diff>10):
        print(f"Difference is: {diff}")
        return True
    else:
        return False

#################################################################
# Execute calculated trajectory using OFFBOARD POSITION CONTROL #
#################################################################
async def execTrajectory(drone):
    global DRONE_STATE, GOAL_STATE
    count = 0  
    prevGoal = None

    while True:
        await asyncio.sleep(0.25)

        if count == 0:
            currGoal = GOAL_STATE
            droneState = await getDroneState(drone)
            path, t = await getTrajectory(droneState, currGoal)

        count += 1

        if currGoal == prevGoal: # If new goal is same as current -> Wait for the goal to get updated
            continue
        else:
            print(f"current goal is: {currGoal}")
            prevGoal = currGoal
            for index in range(len(t)-1):
                flag = await checkDiff(currGoal) # Check if new user location is distant from the previous one
                
                if flag is False:
                    await drone.offboard.set_position_ned(PositionNedYaw(path[0][index], path[1][index], -10, 0)) # Currently no yaw and fixed height
                    await asyncio.sleep(t[index+1] - t[index])
                    print(t[index])
                else:
                    print(f"Goal Updated : {GOAL_STATE}")
                    prevGoal = currGoal
                    currGoal = GOAL_STATE
                    droneState = await getDroneState(drone)
                    print(f"Updated Drone state: {droneState}")
                    path,t = await getTrajectory(droneState, currGoal)
                    break
     


async def main():
    drone = System()
    
    await drone.connect(system_address="udp://:14540")
    await drone.action.arm()
    await drone.action.takeoff()
    await drone.offboard.set_position_ned(PositionNedYaw(0.0, 0.0, 0.0, 0.0))
    await drone.offboard.start()

    print("Drone in offboard mode ...")

    t1 = asyncio.ensure_future(execTrajectory(drone))
    t3 = asyncio.ensure_future(publish_target_Coord())
    await t3, t1

if __name__=="__main__":
    asyncio.run(main())