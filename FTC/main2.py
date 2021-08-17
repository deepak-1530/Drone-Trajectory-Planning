import numpy as np
import mavsdk
from mavsdk import System
import matplotlib.pyplot as plt
from QuiPoly import QuinticPolynomial
import math
import asyncio
from mavsdk.offboard import (OffboardError, PositionNedYaw)

############################
### Follow the customer ### 
############################

###############################################################
# Coroutines -> Publish the target coordinate, calculate Traj #
# Global Variables -> Trajectory (set of waypoints)           #
# Waypoints -> At fixed height (10 metress)                   #
###############################################################

####################################
# async def publish_target_Coord() #
# async def generateTraj()         #
# async def executeTraj()          #
####################################

WAYPOINTS_X = []
WAYPOINTS_Y = []
TIME        = []
YAW         = []


TARGET_STATE = []

COORDINATES = [[70,50,0,0,0], [75,56,0,0,10], [80,64,0,0,25], [74,45,0,0,45]]  # x, y,v,a yaw (degrees)

DRONE_STATE = []   # x,y,z, vx, vy, vz

async def publish_target_Coord():
    global TARGET_LOC
    for i in range(5):
        TARGET_STATE = COORDINATES[i]
        await asyncio.sleep(10)

async def getDroneState(drone):
    global DRONE_STATE
    async for state in drone.telemetry.position_velocity_ned():
        DRONE_STATE = [state.position.north_m, state.position.east_m, state.position.down_m, state.velocity.velocity.north_m_s, state.velocity.east_m_s, state.velocity.down_m_s]
        await asyncio.sleep(0.25)


async def generateTraj():
    global WAYPOINTS_X, WAYPOINTS_Y
    global TARGET_STATE

    # get the drone's current and target locations
    # use the telemetry function to update the drone's location
    # when the trajectory gets updated, change the waypoints array
    # get the drone's current state and target state
    # update the trajectory

async def executeTraj():
    global WAYPOINTS_X, WAYPOINTS_Y