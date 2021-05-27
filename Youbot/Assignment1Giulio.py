# -*- coding: utf-8 -*-
"""
The aim of this code is to show small examples of controlling the displacement of the robot in V-REP.

(C) Copyright Renaud Detry 2013, Mathieu Baijot 2017, Norman Marlier 2019.
Distributed under the GNU General Public License.
(See http://www.gnu.org/copyleft/gpl.html)
"""
# VREP
import sim as vrep

# Useful import

import math
import numpy as np
import numpy
import pickle
import sys
import time
import string
from sympy import *
from scipy.optimize import fsolve, root
import matplotlib.pyplot as plt
from sklearn.cluster import KMeans
# import pandas as pd

from skimage import measure
from skimage.draw import ellipse
from skimage.measure import label, regionprops, regionprops_table
from skimage.transform import rotate

from cleanup_vrep import cleanup_vrep
from spatialmath.base import homtrans
from vrchk import vrchk
from youbot_init import youbot_init
from youbot_drive import youbot_drive
from youbot_hokuyo_init import youbot_hokuyo_init
from youbot_hokuyo import youbot_hokuyo
from youbot_xyz_sensor import youbot_xyz_sensor
from beacon import beacon_init, youbot_beacon
from utils_sim import angdiff

from trans_rot_matrix import trans_rot_matrix
from matplotlib.path import Path
import matplotlib.pyplot as plt

from Functions import rotate, stopWheels, rotate2

# Test the python implementation of a youbot
# Initiate the connection to the simulator.
print('Program started')
# Use the following line if you had to recompile remoteApi
# vrep = remApi('remoteApi', 'extApi.h')
# vrep = remApi('remoteApi')

# Close the connection in case if a residual connection exists
vrep.simxFinish(-1)
clientID = vrep.simxStart('127.0.0.1',  19997, True, True, 2000, 5)

# The time step the simulator is using (your code should run close to it).
timestep = .05

# Synchronous mode
returnCode = vrep.simxSynchronous(clientID, True)

# If you get an error like:
#   Remote API function call returned with error code: 64.
# Explanation: simxStart was not yet called.
# Make sure your code is within a function!
# You cannot call V-REP from a script.
if clientID < 0:
    sys.exit('Failed connecting to remote API server. Exiting.')

print('Connection ' + str(clientID) + ' to remote API server open')

# Make sure we close the connection whenever the script is interrupted.
# cleanup_vrep(vrep, id)

# This will only work in "continuous remote API server service".
# See http://www.v-rep.eu/helpFiles/en/remoteApiServerSide.htm
vrep.simxStartSimulation(clientID, vrep.simx_opmode_blocking)

# Send a Trigger to the simulator: this will run a time step for the physics engine
# because of the synchronous mode. Run several iterations to stabilize the simulation

for i in range(int(1./timestep)):
    vrep.simxSynchronousTrigger(clientID)
    vrep.simxGetPingTime(clientID)

# Retrieve all handles, mostly the Hokuyo.
h = youbot_init(vrep, clientID)
h = youbot_hokuyo_init(vrep, h)
beacons_handle = beacon_init(vrep, clientID, h)

# Send a Trigger to the simulator: this will run a time step for the physics engine
# because of the synchronous mode. Run several iterations to stabilize the simulation
for i in range(int(1./timestep)):
    vrep.simxSynchronousTrigger(clientID)
    vrep.simxGetPingTime(clientID)



##############################################################################
#                                                                            #
#                          INITIAL CONDITIONS                                #
#                                                                            #
##############################################################################
# Define all the variables which will be used through the whole simulation.
# Important: Set their initial values.

# Get the position of the beacons in the world coordinate frame (x, y)
beacons_world_pos = np.zeros((len(beacons_handle), 3))
for i, beacon in enumerate(beacons_handle):
    res, beacons_world_pos[i] = vrep.simxGetObjectPosition(clientID, beacon, -1,
                                                           vrep.simx_opmode_buffer)

# Parameters for controlling the youBot's wheels: at each iteration,
# those values will be set for the wheels.
# They are adapted at each iteration by the code.
forwBackVel = 0  # Move straight ahead.
rightVel = 0  # Go sideways.
rotateRightVel = 0  # Rotate.
prevOrientation = 0  # Previous angle to goal (easy way to have a condition on the robot's angular speed).
prevPosition = [0, 0]  # Previous distance to goal (easy way to have a condition on the robot's speed).

# Set the arm to its starting configuration. id
res = vrep.simxPauseCommunication(clientID, True)  # Send order to the simulator through vrep object.
vrchk(vrep, res)  # Check the return value from the previous V-REP call (res) and exit in case of error.

res = vrep.simxPauseCommunication(clientID, False)
vrchk(vrep, res)

# Define Constants
# Definition of the starting pose of the arm (the angle to impose at each joint to be in the rest position).
startingJoints = [0, 30.91 * math.pi / 180, 52.42 * math.pi / 180, 72.68 * math.pi / 180, 0]
resolution = 0.25
dim = 7.5   # house's dimension

# Initialize arm position
# for i in range(5):
#     res = vrep.simxSetJointTargetPosition(clientID, h.armJoints[i], startingJoints[i], vrep.simx_opmode_oneshot)
#     vrchk(vrep, res, True)

# x, y will be used to display the area the robot can see, by
# selecting the points within this mesh that are within the visibility range.
x, y = np.meshgrid(np.arange(-dim, dim + resolution, resolution), np.arange(-dim, dim + resolution, resolution))
x, y = x.flatten(), y.flatten()

# these are the axis
xAxis = np.arange(-dim, dim + resolution, resolution)
yAxis = np.arange(-dim, dim + resolution, resolution)

# dimension of the x and y axes
n = len(xAxis)

# Create a list that stores the max high of objects on tables (to determine target)
tablesMaxHigh = np.zeros((1, 3))

# Make sure everything is settled before we start.
# pause(2)

# For infinite loop
p = True
# Flag that determines if the robot holds a object or not
holdObject = False
# Flag that determine if the robot as to rotateAndSlide closer to the table or further (just use during grasping)
slideCloser = True

# Initialiaze the index in "goals" array (here equal to 0 for plot)
i = 0

#
# --- Decide where to start -----------------------------------------------
#
start = 'grasping'

if start == 'navigation':
    navigationFinished = False
    # building up the state map assuming all the states as unknown (= 1)
    statesMap = np.ones((n, n), dtype=int)
    # Initialise the state machine.
    fsm = 'searchAlgo'
    counterSearchAlgo = 0
    print('Switching to state: ', fsm)
    modelTableFlag = True

elif start == 'findtarget':
    navigationFinished = True
    discoverTableCounter = 0
    statesMap = np.loadtxt("saveStatesMap.txt", dtype='i', delimiter=',')
    occupancyGridAstarList = np.loadtxt('saveoccupancyGridAstarList.txt', dtype='i', delimiter=',')
    counterSearchAlgo = 0

    # turn off the hokuyo captor
    res = vrep.simxSetIntegerSignal(clientID, 'handle_xy_sensor', 0, vrep.simx_opmode_oneshot)
    vrchk(vrep, res, True)
    # Initialise the state machine.
    fsm = 'searchTables'
    print('Switching to state: ', fsm)
    modelTableFlag = True

elif start == 'modelTable':
    navigationFinished = True
    discoverTableCounter = 3
    statesMap = np.loadtxt("saveStatesMap.txt", dtype='i', delimiter=',')
    occupancyGridAstarList = np.loadtxt('saveoccupancyGridAstarList.txt', dtype='i', delimiter=',')
    objectsTablesID = np.loadtxt('saveobjectsTablesID.txt', dtype='i', delimiter=',')
    targetID = np.loadtxt("savetargetID.txt", dtype='i', delimiter=',')
    tablesRealCenter = np.loadtxt("savetablesRealCenter.txt", dtype='f', delimiter=',')
    tablesCenters = np.loadtxt("savetablesCenters.txt", dtype='i', delimiter=',')
    table1Neighbours = np.loadtxt("savetable1Neighbours.txt", dtype='i', delimiter=',')
    table2Neighbours = np.loadtxt("savetable2Neighbours.txt", dtype='i', delimiter=',')
    targetNeighbours = np.loadtxt("savetargetNeighbours.txt", dtype='i', delimiter=',')
    ptsTable1 = []
    ptsTable2 = []
    ptsTarget = []
    ptsObjects1 = []
    ptsObjects2 = []
    tabToModel = table1Neighbours
    tabID = objectsTablesID[0]
    neighbour = 0
    counterSearchAlgo = 0
    modelTableFlag = True

    # turn off the hokuyo captor
    res = vrep.simxSetIntegerSignal(clientID, 'handle_xy_sensor', 0, vrep.simx_opmode_oneshot)
    vrchk(vrep, res, True)
    # Initialise the state machine.
    fsm = 'astar'
    print('Switching to state: ', fsm)

elif start == 'imageAnalysis':
    navigationFinished = True
    discoverTableCounter = 3
    statesMap = np.loadtxt("saveStatesMap.txt", dtype='i', delimiter=',')
    occupancyGridAstarList = np.loadtxt('saveoccupancyGridAstarList.txt', dtype='i', delimiter=',')
    objectsTablesID = np.loadtxt('saveobjectsTablesID.txt', dtype='i', delimiter=',')
    targetID = np.loadtxt("savetargetID.txt", dtype='i', delimiter=',')
    tablesRealCenter = np.loadtxt("savetablesRealCenter.txt", dtype='i', delimiter=',')
    tablesCenters = np.loadtxt("savetablesCenters.txt", dtype='i', delimiter=',')
    table1Neighbours = np.loadtxt("savetable1Neighbours.txt", dtype='i', delimiter=',')
    table2Neighbours = np.loadtxt("savetable2Neighbours.txt", dtype='i', delimiter=',')
    targetNeighbours = np.loadtxt("savetargetNeighbours.txt", dtype='i', delimiter=',')
    ptsTable1 = np.loadtxt("saveptsTable1.txt", dtype='f', delimiter=',')
    ptsTable2 = np.loadtxt("saveptsTable2.txt", dtype='f', delimiter=',')
    ptsTarget = np.loadtxt("saveptsTarget.txt", dtype='f', delimiter=',')
    ptsObjects1 = np.loadtxt("saveptsObjects1.txt", dtype='f', delimiter=',')
    ptsObjects2 = np.loadtxt("saveptsObjects2.txt", dtype='f', delimiter=',')
    tabToModel = table1Neighbours
    tabID = objectsTablesID[0]
    neighbour = 4
    counterSearchAlgo = 0
    modelTableFlag = True

    # turn off the hokuyo captor
    res = vrep.simxSetIntegerSignal(clientID, 'handle_xy_sensor', 0, vrep.simx_opmode_oneshot)
    vrchk(vrep, res, True)
    # Initialise the state machine.
    fsm = 'imageAnalysis'
    print('Switching to state: ', fsm)

elif start == 'grasping':
    navigationFinished = True
    statesMap = np.loadtxt("saveStatesMap.txt", dtype='i', delimiter=',')
    occupancyGridAstarList = np.loadtxt('saveoccupancyGridAstarList.txt', dtype='i', delimiter=',')
    centerObject1 = np.loadtxt("savecenterObject1.txt", dtype='f', delimiter=',')
    centerObject2 = np.loadtxt("savecenterObject2.txt", dtype='f', delimiter=',')
    objectsTablesID = np.loadtxt("saveobjectsTablesID.txt", dtype='i', delimiter=',')
    targetID = np.loadtxt("savetargetID.txt", dtype='i', delimiter=',')
    tablesRealCenter = np.loadtxt("savetablesRealCenter.txt", dtype='f', delimiter=',')
    destObjects = np.loadtxt("savedestObjects.txt", dtype='f', delimiter=',')
    centerTarget = np.loadtxt("savecenterTarget.txt", dtype='f', delimiter=',')

    objectID = 0
    discoverTableCounter = 3
    tabID = targetID
    neighbour = 4
    counterSearchAlgo = 0
    modelTableFlag = False

    # turn off the hokuyo captor
    res = vrep.simxSetIntegerSignal(clientID, 'handle_xy_sensor', 0, vrep.simx_opmode_oneshot)
    vrchk(vrep, res, True)
    # Initialise the state machine.
    fsm = 'calculateObjectGoal'
    print('Switching to state: ', fsm)

# obtaining the state map size
sizeMap = statesMap.shape
xLength = sizeMap[0]
yLength = sizeMap[1]

# Get the initial position
res, youbotPos = vrep.simxGetObjectPosition(clientID, h['ref'], -1, vrep.simx_opmode_buffer)
# Set the speed of the wheels to 0.
h = youbot_drive(vrep, h, forwBackVel, rightVel, rotateRightVel)

# Send a Trigger to the simulator: this will run a time step for the physic engine
# because of the synchronous mode. Run several iterations to stabilize the simulation
for i in range(int(1./timestep)):
    vrep.simxSynchronousTrigger(clientID)
    vrep.simxGetPingTime(clientID)

timing = []
counter = 0

# Start the demo.
while p:
    try:
        # Check the connection with the simulator
        start = time.time()
        if vrep.simxGetConnectionId(clientID) == -1:
            sys.exit('Lost connection to remote API.')

        # Get the position and the orientation of the robot.
        res, youbotPos = vrep.simxGetObjectPosition(clientID, h['ref'], -1, vrep.simx_opmode_buffer)
        vrchk(vrep, res, True)  # Check the return value from the previous V-REP call (res) and exit in case of error.
        res, youbotEuler = vrep.simxGetObjectOrientation(clientID, h['ref'], -1, vrep.simx_opmode_buffer)
        vrchk(vrep, res, True)

        # Get the position of the robot in matrix form
        xRobot = round((youbotPos[0] + 7.5)/resolution)
        yRobot = round((youbotPos[1] + 7.5)/resolution)

        if not navigationFinished:

            # --- Drawing the map initialization ---
            #
            # Get data from the hokuyo - return empty if data is not captured
            rotangle = youbotEuler[2] - math.pi/2
            hokuyoPos = np.array([[youbotPos[0]], [youbotPos[1]]]) + np.array([[np.cos(rotangle)], [np.sin(rotangle)]]) * 0.23
            # 0.23 is the distance along Y between youbot_Center and fastHokuyo_ref

            # Determine the position of the Hokuyo with global coordinates (world reference frame).
            trf = trans_rot_matrix(youbotEuler, youbotPos)  # check the file trans_rot_matrix for explanation

            scanned_points, contacts = youbot_hokuyo(vrep, h, vrep.simx_opmode_buffer, trf)
            vrchk(vrep, res)
            # scanned_points is a 6xN matrix, where rows 1 and 4 represents coordinate x,
            # rows 2 and 5 coord y and rows 3 and 6 coord z. So to obtain all the x coords
            # we need to concatenate row 1 and 4. This is done in the following lines of code

            # Free space points
            points = np.vstack((x, y)).T  # x, y defined on line 108

            # reorder scanned_points like x = [x1, x2... xn] and y = [y1, y2 ... yn]
            row1 = scanned_points[0, :]
            row2 = scanned_points[1, :]
            row4 = scanned_points[3, :]
            row5 = scanned_points[4, :]
            arr1 = np.squeeze(np.asarray(row1))  # squeeze change the type from np.matrix to np.array, needed to concatenate
            arr2 = np.squeeze(np.asarray(row2))
            arr4 = np.squeeze(np.asarray(row4))
            arr5 = np.squeeze(np.asarray(row5))
            x_scanned = np.hstack((arr1, arr4))  # concatenate horizontally
            y_scanned = np.hstack((arr2, arr5))
            x_polygon = np.hstack((youbotPos[0], x_scanned))  # concatenate horizontally
            y_polygon = np.hstack((youbotPos[1], y_scanned))
            polygon_vertex = np.vstack((x_polygon, y_polygon)).T

            # Make a polygon with the scanned points (boundary points) and check what points of the statesMap
            # are inside this polygon. The points inside the polygon will be free space
            p = Path(polygon_vertex)  # make a polygon
            grid = p.contains_points(points)  # check what points fall inside (grid represents the index of the points)

            # Get the real coordinates of the points determined by "grid"
            x_free = x[grid]
            y_free = y[grid]

            # get the free point in matrix form
            x_free = (x_free + 7.5)/resolution
            y_free = (y_free + 7.5)/resolution
            x_free = np.round(x_free)
            y_free = np.round(y_free)

            # update reachable states of the statesMap
            for i in range(len(x_free)):
                if statesMap[int(x_free[i]), int(y_free[i])] == 1:
                    statesMap[int(x_free[i]), int(y_free[i])] = 0

            # Obstacle points
            contacts_total = np.hstack((contacts[0, :], contacts[1, :]))
            xObstacle = x_scanned[contacts_total]
            yObstacle = y_scanned[contacts_total]

            xObstacle = (xObstacle + 7.5)/resolution
            yObstacle = (yObstacle + 7.5)/resolution
            xObstacle = np.round(xObstacle)
            yObstacle = np.round(yObstacle)

            # assigning state 2 to the obstacles and state 3 to the points adjacent to the obstacles
            for i in range(len(xObstacle)):
                statesMap[int(xObstacle[i]), int(yObstacle[i])] = 2

                if 0 <= xObstacle[i] + 1 < len(xAxis) \
                        and 0 <= yObstacle[i] + 1 < len(yAxis):
                    if statesMap[int(xObstacle[i] + 1), int(yObstacle[i] + 1)] == 0:
                        statesMap[int(xObstacle[i] + 1), int(yObstacle[i] + 1)] = 3

                if 0 <= xObstacle[i] + 1 < len(xAxis) \
                        and 0 <= yObstacle[i] - 1 < len(yAxis):
                    if statesMap[int(xObstacle[i] + 1), int(yObstacle[i] - 1)] == 0:
                        statesMap[int(xObstacle[i] + 1), int(yObstacle[i] - 1)] = 3

                if 0 <= xObstacle[i] + 1 < len(xAxis) \
                        and 0 <= yObstacle[i] < len(yAxis):
                    if statesMap[int(xObstacle[i] + 1), int(yObstacle[i])] == 0:
                        statesMap[int(xObstacle[i] + 1), int(yObstacle[i])] = 3

                if 0 <= xObstacle[i] - 1 < len(xAxis) \
                        and 0 <= yObstacle[i] + 1 < len(yAxis):
                    if statesMap[int(xObstacle[i] - 1), int(yObstacle[i] + 1)] == 0:
                        statesMap[int(xObstacle[i] - 1), int(yObstacle[i] + 1)] = 3

                if 0 <= xObstacle[i] - 1 < len(xAxis) \
                        and 0 <= yObstacle[i] - 1 < len(yAxis):
                    if statesMap[int(xObstacle[i] - 1), int(yObstacle[i] - 1)] == 0:
                        statesMap[int(xObstacle[i] - 1), int(yObstacle[i] - 1)] = 3

                if 0 <= xObstacle[i] - 1 < len(xAxis) \
                        and 0 <= yObstacle[i] < len(yAxis):
                    if statesMap[int(xObstacle[i] - 1), int(yObstacle[i])] == 0:
                        statesMap[int(xObstacle[i] - 1), int(yObstacle[i])] = 3

                if 0 <= xObstacle[i] < len(xAxis) \
                        and 0 <= yObstacle[i] + 1 < len(yAxis):
                    if statesMap[int(xObstacle[i]), int(yObstacle[i] + 1)] == 0:
                        statesMap[int(xObstacle[i]), int(yObstacle[i] + 1)] = 3

                if 0 <= xObstacle[i] < len(xAxis) \
                        and 0 <= yObstacle[i] - 1 < len(yAxis):
                    if statesMap[int(xObstacle[i]), int(yObstacle[i] - 1)] == 0:
                        statesMap[int(xObstacle[i]), int(yObstacle[i] - 1)] = 3

            # a copy of the statesMap is realized to leave the statesMap neater
            statesMapCopy = statesMap.copy()

            # assigning state 4 to the points adjacent to the state 3 points
            for j in range(len(xAxis)):
                for k in range(len(yAxis)):
                    if statesMapCopy[j, k] == 3:
                        if statesMapCopy[j - 1, k - 1] == 0:
                            statesMapCopy[j - 1, k - 1] = 4

                        if statesMapCopy[j, k - 1] == 0:
                            statesMapCopy[j, k - 1] = 4

                        if statesMapCopy[j + 1, k - 1] == 0:
                            statesMapCopy[j + 1, k - 1] = 4

                        if statesMapCopy[j - 1, k] == 0:
                            statesMapCopy[j - 1, k] = 4

                        if statesMapCopy[j + 1, k] == 0:
                            statesMapCopy[j + 1, k] = 4

                        if statesMapCopy[j - 1, k + 1] == 0:
                            statesMapCopy[j - 1, k + 1] = 4

                        if statesMapCopy[j, k + 1] == 0:
                            statesMapCopy[j, k + 1] = 4

                        if statesMapCopy[j + 1, k + 1] == 0:
                            statesMapCopy[j + 1, k + 1] = 4

            # assigning state 5 to the points adjacent to the state 4 points
            for j in range(len(xAxis)):
                for k in range(len(yAxis)):
                    if statesMapCopy[j, k] == 4:
                        if statesMapCopy[j - 1, k - 1] == 0:
                            statesMapCopy[j - 1, k - 1] = 5

                        if statesMapCopy[j, k - 1] == 0:
                            statesMapCopy[j, k - 1] = 5

                        if statesMapCopy[j + 1, k - 1] == 0:
                            statesMapCopy[j + 1, k - 1] = 5

                        if statesMapCopy[j - 1, k] == 0:
                            statesMapCopy[j - 1, k] = 5

                        if statesMapCopy[j + 1, k] == 0:
                            statesMapCopy[j + 1, k] = 5

                        if statesMapCopy[j - 1, k + 1] == 0:
                            statesMapCopy[j - 1, k + 1] = 5

                        if statesMapCopy[j, k + 1] == 0:
                            statesMapCopy[j, k + 1] = 5

                        if statesMapCopy[j + 1, k + 1] == 0:
                            statesMapCopy[j + 1, k + 1] = 5

        # # Occupancy grid
        #
        # occupancyGrid = np.ones((n, n), dtype=int)
        #
        # for j in range(len(xAxis)):
        #     for k in range(len(yAxis)):
        #         if statesMap[j, k] == 2 or statesMap[j, k] == 3:
        #             occupancyGrid[j, k] = 1
        #         else:
        #             occupancyGrid[j, k] = 0
        #
        # # plt.matshow(occupancyGrid)
        # # plt.colorbar()
        # # plt.show()

        # --- Search algorithm ---
        # search for a goal point to visit

        # Apply the state machine.
        if fsm == 'searchAlgo':

            counterSearchAlgo += 1

            # initialize variables
            distMax = 50
            xTarget = 0
            yTarget = 0
            foundTarget = False

            # first, we search a target near the horizon
            x_scanned = (x_scanned + 7.5)/resolution
            y_scanned = (y_scanned + 7.5)/resolution
            x_scanned = np.round(x_scanned)
            y_scanned = np.round(y_scanned)

            for ii in range(len(x_scanned)):
                if statesMapCopy[int(x_scanned[ii]), int(y_scanned[ii])] == 0:

                    if int(x_scanned[ii]) + 1 <= n and statesMapCopy[int(x_scanned[ii]) + 1, int(y_scanned[ii])] == 1:

                        xUnknown = int(x_scanned[ii]) + 1
                        yUnknown = int(y_scanned[ii])
                        foundTarget = True

                    elif int(x_scanned[ii]) - 1 <= n and statesMapCopy[int(x_scanned[ii]) - 1, int(y_scanned[ii])] == 1:

                        xUnknown = int(x_scanned[ii]) - 1
                        yUnknown = int(y_scanned[ii])
                        foundTarget = True

                    elif int(y_scanned[ii]) + 1 <= n and statesMapCopy[int(x_scanned[ii]), int(y_scanned[ii]) + 1] == 1:

                        xUnknown = int(x_scanned[ii])
                        yUnknown = int(y_scanned[ii]) + 1
                        foundTarget = True

                    elif int(y_scanned[ii]) - 1 <= n and statesMapCopy[int(x_scanned[ii]), int(y_scanned[ii]) - 1] == 1:

                        xUnknown = int(x_scanned[ii])
                        yUnknown = int(y_scanned[ii]) - 1
                        foundTarget = True

                if foundTarget:

                    xT = int(x_scanned[ii])
                    yT = int(y_scanned[ii])

                    dist = math.sqrt((xRobot - xT)**2 + (yRobot - yT)**2)

                    # minimum and maximum threshold for the target
                    if 3 < dist < distMax:
                        xTarget = xT
                        yTarget = yT
                        distMax = dist
                        # print(dist)

                    foundTarget = False

            if xTarget == 0:

                # search for a target point everywhere, finding an unknown element with adjacent free space
                for jj in range(1, xLength - 1):

                    for kk in range(1, yLength - 1):

                        if statesMapCopy[jj, kk] == 1:

                            if statesMapCopy[jj - 1, kk] == 0:
                                xT = jj - 1
                                yT = kk
                                foundTarget = True

                            elif statesMapCopy[jj + 1, kk] == 0:
                                xT = jj + 1
                                yT = kk
                                foundTarget = True

                            elif statesMapCopy[jj - 1, kk + 1] == 0:
                                xT = jj - 1
                                yT = kk + 1
                                foundTarget = True

                            elif statesMapCopy[jj, kk + 1] == 0:
                                xT = jj
                                yT = kk + 1
                                foundTarget = True

                            elif statesMapCopy[jj + 1, kk + 1] == 0:
                                xT = jj + 1
                                yT = kk + 1
                                foundTarget = True

                            elif statesMapCopy[jj - 1, kk - 1] == 0:
                                xT = jj - 1
                                yT = kk - 1
                                foundTarget = True

                            elif statesMapCopy[jj, kk - 1] == 0:
                                xT = jj
                                yT = kk - 1
                                foundTarget = True

                            elif statesMapCopy[jj + 1, kk - 1] == 0:
                                xT = jj + 1
                                yT = kk - 1
                                foundTarget = True

                            if foundTarget:

                                # compute distance and threshold
                                dist = math.sqrt((xRobot - xT)**2 + (yRobot - yT)**2)
                                if 3 < dist < distMax:
                                    xUnknown = jj
                                    yUnknown = kk
                                    xTarget = xT
                                    yTarget = yT
                                    distMax = dist

                                foundTarget = False

            # if the map is fully explored
            if xTarget == 0:
                fsm = 'navigationFinished'
            else:
                fsm = 'astar'

        # --- Manage end of navigation ---
        elif fsm == 'navigationFinished':

            print('No more accessible points to visit...')
            print('Final map created')

            # Every unreachable point is considered as an obstacles
            for j in range(xLength):
                for k in range(yLength):
                    if statesMap[j, k] == 1:
                        statesMap[j, k] = 2

            # # Plot of the total map
            # plt.close()
            # plt.matshow(statesMap)
            # plt.colorbar()
            # plt.show()

            mat = np.matrix(statesMap)
            with open('saveStatesMap.txt', 'wb') as f:
                for line in mat:
                    np.savetxt(f, line, fmt='%.2f', delimiter=',')

            mat2 = np.matrix(occupancyGridAstarList)
            with open('saveoccupancyGridAstarList.txt', 'wb') as f:
                for line in mat2:
                    np.savetxt(f, line, fmt='%.2f', delimiter=',')

            # End the infinite loop
            navigationFinished = True

            # turn off the hokuyo captor
            res = vrep.simxSetIntegerSignal(clientID, 'handle_xy_sensor', 0, vrep.simx_opmode_oneshot)
            vrchk(vrep, res, True)

            fsm = 'searchTables'

            # plt.close()
            # plt.matshow(StatesMap)
            # plt.colorbar()
            # plt.show()

            # Plot the loop time as a histogram

            # print(timing)
            # print(max(timing))
            # print(min(timing))

            # n, bins, patches = plt.hist(x=timing, bins='auto', color='#0504aa', alpha=0.7, rwidth=0.85)
            #
            # plt.grid(axis='y', alpha=0.75)
            # plt.xlabel('Time [s]')
            # plt.ylabel('Number of loops')
            # maxTime = n.max()
            # plt.ylim(ymax=np.ceil(maxTime/10) * 10 if maxTime % 10 else maxTime + 10)
            #
            # plt.show()

            # End the infinite loop
            # p = False

        #
        # --- Astar path planning ---
        #

        elif fsm == 'astar':

            if not navigationFinished:

                occupancyGridAstar = np.ones((n, n), dtype=int)

                # occupancyGridAstar:
                # assign weights to the points of the grid (decreasing weights in relation to
                # the distance from the obstacles)
                for j in range(len(xAxis)):
                    for k in range(len(yAxis)):
                        if statesMapCopy[j, k] == 1 or statesMapCopy[j, k] == 2:
                            occupancyGridAstar[j, k] = 1000

                        elif statesMapCopy[j, k] == 3:
                            occupancyGridAstar[j, k] = 750

                        elif statesMapCopy[j, k] == 4:
                            occupancyGridAstar[j, k] = 500

                        elif statesMapCopy[j, k] == 5:
                            occupancyGridAstar[j, k] = 400

                        else:
                            occupancyGridAstar[j, k] = -1

                occupancyGridAstar = occupancyGridAstar.transpose()
                occupancyGridAstarList = occupancyGridAstar.tolist()

            # Astar algorithm implemetation by Melvin Petrocchi
            from astar_python.astar import Astar

            astar = Astar(occupancyGridAstarList)

            # Choose in which case we are (Goal differs if we are in the
            # navigation phase, grasping, modeling the table,...)
            if not navigationFinished:
                results = astar.run([xRobot, yRobot], [xTarget, yTarget])

            elif discoverTableCounter < 3:
                j = discoverTableCounter
                results = astar.run([xRobot, yRobot], [tablesNeighbours[j, 0], tablesNeighbours[j, 1]])

            elif modelTableFlag and neighbour < 5:
                print('nooo')
                results = astar.run([xRobot, yRobot], [tabToModel[neighbour, 0], tabToModel[neighbour, 1]])

            elif neighbour < 4:
                results = astar.run([xRobot, yRobot], [targetNeighbours[neighbour, 0], targetNeighbours[neighbour, 1]])

            elif objectID < 5:
                if holdObject == False:
                    print('if')
                    results = astar.run([xRobot, yRobot], [posNearObject[0, 0], posNearObject[0, 1]])
                else:
                    if tabID == 0:
                        results = astar.run([xRobot, yRobot], [destObjects[objectID, 0], destObjects[objectID, 1]])
                    else:
                        results = astar.run([xRobot, yRobot], [destObjects[objectID+5, 0], destObjects[objectID+5, 1]])

            # Create a copy of the statesMap to plot the path generated by Astar.
            # In the path, we take only one point every 5: this can be done because of
            # the weights given in occupancyGridAstar (for the points near the obstacles)
            pathMat = statesMap.copy()

            pathLen = int(len(results)/5)

            path = np.zeros((pathLen + 2, 2), dtype=float)

            for i in range(pathLen):
                resultsElem = results[5*i]
                path[i, 0] = xAxis[resultsElem[0]]
                path[i, 1] = yAxis[resultsElem[1]]
                for j in range(xLength):
                    for k in range(yLength):
                        if resultsElem[0] == j and resultsElem[1] == k:
                            pathMat[j, k] = 5

            # here we take the last to points of the original path (not divided by 5)
            resultsElem1 = results[-1]
            resultsElem2 = results[-2]
            path[-1, 0] = xAxis[resultsElem1[0]]
            path[-1, 1] = yAxis[resultsElem1[1]]
            path[-2, 0] = xAxis[resultsElem2[0]]
            path[-2, 1] = yAxis[resultsElem2[1]]

            pathMat[resultsElem1[0], resultsElem1[1]] = 5
            pathMat[resultsElem2[0], resultsElem2[1]] = 5

            # Plot the occupancyGridAstar only 1 time (after 9 iterations of SearchAlgo)
            if counterSearchAlgo == 10:
                plt.close()

            plt.close()
            plt.matshow(pathMat)
            plt.colorbar()
            plt.show(block=False)
            plt.pause(.001)

            if counterSearchAlgo == 9:
                plt.matshow(occupancyGridAstar.transpose())
                plt.colorbar()
                plt.show(block=False)
                plt.pause(.001)

            # Initialize the index for the elements of the path
            iPath = 1
            fsm = 'rotate'
            print('Switching to state: ', fsm)

        #
        # --- Rotation ---
        #

        elif fsm == 'rotate':

            # Rotate until target point is straight ahead (measured with respect to the world's reference frame).
            # Use a proportional controller.
            # youbotEuler[2] is the rotation around the vertical axis.
            if iPath >= len(path)-1 and not navigationFinished:
                fsm = 'searchAlgo'
                print('Switching to state: ', fsm)

            # if iPath >= len(path)-1:
            #     iPath = len(path)-1

            a = path[iPath, 0] - youbotPos[0]
            b = youbotPos[1] - path[iPath, 1]
            rotationAngle = math.atan2(a, b)

            rotateRightVel = angdiff(youbotEuler[2], rotationAngle)

            if rotateRightVel > math.pi:
                rotateRightVel = rotateRightVel - math.pi*2
                # print('primo if')

            if rotateRightVel < - math.pi:
                rotateRightVel = rotateRightVel + math.pi*2
                # print('secondo if')

            # Stop when the robot is at an angle close to rotationAngle
            if abs(angdiff(youbotEuler[2], rotationAngle)) < 0.05:
                rotateRightVel = 0
                fsm = 'forward'
                print('Switching to state: ', fsm)

            # Rotate(vrep, clientID, h, rotateRightVel, path[iPath, 0], path[iPath, 1], youbotPos[0], youbotPos[1])

            # rotateRightVel = 0
            # fsm = 'forward'
            # print('Switching to state: ', fsm)

        #
        # --- Forward ---
        #
        elif fsm == 'forward':

            if not navigationFinished and iPath >= len(path)-1:
                fsm = 'searchAlgo'
                print('Switching to state: ', fsm)
            else:
                # Set a costant velocity
                forwBackVel = - 5

                # Stop when the robot is close to the next element of the path.
                # The tolerance has been determined by experiments
                a = path[iPath, 0] - youbotPos[0]
                b = youbotPos[1] - path[iPath, 1]
                distance = math.sqrt(a**2 + b**2)  # distance between robot and goal
                if not navigationFinished and abs(distance) < .5:
                    forwBackVel = 0  # Stop the robot.
                    iPath = iPath + 1
                    fsm = 'rotate'
                    print('Switching to state: ', fsm)

                # Condition that is verify after the map is fully
                # discovered and when we reached the last goal point
                elif navigationFinished and abs(distance) < .5:
                    forwBackVel = 0  # Stop the robot.
                    iPath = iPath + 1
                    fsm = 'rotate'

                    if len(path)-1 <= iPath:

                        if discoverTableCounter < 3:
                            fsm = 'rotateToCenter'
                            print('Switching to state: ', fsm)

                        elif modelTableFlag and neighbour < 5:
                            modelTableFlag = not modelTableFlag
                            fsm = 'rotateToCenter'
                            print('Switching to state: ', fsm)

                        elif neighbour < 4:
                            fsm = 'rotateToCenter'
                            print('Switching to state: ', fsm)

                        elif objectID < 5 and len(path)-1 < iPath:
                            iPath = iPath - 1
                            fsm = 'preciseForward'
                            print('Switching to state: ', fsm)
                            # iPath = iPath - 1
                            # forwBackVel = - 1
                            # a = path[iPath, 0] - youbotPos[0]
                            # b = youbotPos[1] - path[iPath, 1]
                            # distance = math.sqrt(a**2 + b**2)  # distance between robot and goal
                            # if navigationFinished and abs(distance) < .1:
                            #     forwBackVel = 0  # Stop the robot.
                            #     iPath = iPath + 1
                            #     fsm = 'rotateAndSlide'
                            #     print('Switching to state: ', fsm)

                    # elif iPath == len(path):
                    #     fsm = 'forward'
                    #     iPath = iPath - 1
                    #     forwBackVel = - 1
                    #     a = path[iPath, 0] - youbotPos[0]
                    #     b = youbotPos[1] - path[iPath, 1]
                    #     distance = math.sqrt(a**2 + b**2)  # distance between robot and goal
                    #     if navigationFinished and abs(distance) < .05:
                    #         forwBackVel = 0  # Stop the robot.
                    #         iPath = iPath + 1
                    #         fsm = 'rotateAndSlide'
                    #         print('Switching to state: ', fsm)

                    # elif iPath > len(path):
                    #     if objectID < 5:
                    #         print('if')
                    #         fsm = 'rotateAndSlide'
                    #         print('Switching to state: ', fsm)

                prevPosition = [youbotPos[0], youbotPos[1]]

        elif fsm == 'preciseForward':
                # Set a costant velocity
                forwBackVel = - 0.1

                # Stop when the robot is close to the next element of the path.
                # The tolerance has been determined by experiments
                a = path[iPath, 0] - youbotPos[0]
                b = youbotPos[1] - path[iPath, 1]
                distance = math.sqrt(a**2 + b**2)  # distance between robot and goal
                if abs(distance) < .07:
                    forwBackVel = 0  # Stop the robot.
                    iPath = iPath + 1
                    fsm = 'rotateAndSlide'
                    print('Switching to state: ', fsm)
                    for i in range(20):
                        vrep.simxSynchronousTrigger(clientID)
                        vrep.simxGetPingTime(clientID)

        elif fsm == 'searchTables':

            binaryMap = np.full((n, n), True, dtype=bool)

            for jjj in range(len(xAxis)):
                for kkk in range(len(yAxis)):
                    if statesMap[jjj, kkk] == 2:
                        binaryMap[jjj, kkk] = True
                    else:
                        binaryMap[jjj, kkk] = False

            nn = measure.label(binaryMap, background=None, return_num=False, connectivity=None)
            # plt.close()
            # plt.matshow(statesMap)
            # plt.colorbar()
            # plt.show()
            props = regionprops_table(nn, properties=('centroid',
                                                      'area',
                                                      'perimeter'))

            blobCentroidX = props['centroid-0']
            blobCentroidY = props['centroid-1']
            blobArea = props['area']
            blobPerimeter = props['perimeter']

            blobID = []

            for i in range(len(blobPerimeter)):
                if 13 < blobPerimeter[i] < 14:
                    blobID.append(i)

            centroidX = []
            centroidY = []
            area = []
            perimeter = []

            for j, ID in enumerate(blobID):
                centroidX.append(round(blobCentroidX[ID]))
                centroidY.append(round(blobCentroidY[ID]))
                area.append(blobArea[ID])
                perimeter.append(blobPerimeter[ID])

            centroidX = np.array(centroidX)
            centroidY = np.array(centroidY)

            tablesCenters = np.vstack((centroidX, centroidY))
            tablesCenters = tablesCenters.transpose()

            # Get the coordinates of a free cell near each table in matrix form
            tablesNeighbours = tablesCenters.copy()

            for j in range(3):
                c = tablesCenters[j, 1] + 1
                if statesMap[tablesCenters[j, 0], c] != 0:
                    c = c + 1
                tablesNeighbours[j, 1] = c

            tablesRealCenter = np.zeros((3, 2))
            # Get real (x, y) coordinates for the center points of the tables
            for i in range(3):
                tablesRealCenter[i, 0] = xAxis[tablesCenters[i, 0]]
                tablesRealCenter[i, 1] = yAxis[tablesCenters[i, 1]]
            print('tablesRealCenter', tablesRealCenter)

            # Set a counter for the table to discover
            discoverTableCounter = 0
            fsm = 'astar'
            print('Switching to state: ', fsm)

        # In this section the robot is rotating the camera towards the center of the table
        elif fsm == 'rotateToCenter':
            k = discoverTableCounter

            # get camera position
            [res, rgbdPos] = vrep.simxGetObjectPosition(clientID, h['rgbdCasing'], -1, vrep.simx_opmode_oneshot_wait)
            vrchk(vrep, res, True)

            if k < 3:
                aa = rgbdPos[0] - tablesRealCenter[k, 0]
                bb = tablesRealCenter[k, 1] - rgbdPos[1]
                beta = math.atan2(aa, bb) + math.pi/2
                # if bb > 0:
                #     beta = beta + math.pi
            else:
                aa = rgbdPos[0] - tablesRealCenter[tabID, 0]
                bb = tablesRealCenter[tabID, 1] - rgbdPos[1]
                beta = math.atan2(aa, bb) + math.pi/2
                # if bb > 0:
                #     beta = beta + math.pi

            # Set the desired camera orientation
            vrep.simxSetObjectOrientation(clientID, h['rgbdCasing'], -1, [0, 0, beta], vrep.simx_opmode_oneshot)

            # Checking whether we are in the target finding phase or table modelling one
            if discoverTableCounter < 3:
                fsm = 'findTarget'
                print('Switching to state: ', fsm)
            else:
                fsm = 'modelTable'
                print('Switching to state: ', fsm)

        # In this section a picture of the table is taken and the target is determined
        elif fsm == 'findTarget':

            j = discoverTableCounter

            # Get scan angle of pi/4 for the depth sensor
            res = vrep.simxSetFloatSignal(clientID, 'rgbd_sensor_scan_angle', math.pi / 4, vrep.simx_opmode_oneshot_wait)
            vrchk(vrep, res)

            # Ask the sensor to turn itself on, take A SINGLE POINT CLOUD, and turn itself off again.
            res = vrep.simxSetIntegerSignal(clientID, 'handle_xyz_sensor', 1, vrep.simx_opmode_oneshot_wait)
            vrchk(vrep, res)

            vrep.simxSynchronousTrigger(clientID)
            vrep.simxGetPingTime(clientID)

            # print(h['ref'])

            # Get the point cloud from the depth sensor
            pointCloud = youbot_xyz_sensor(vrep, h, vrep.simx_opmode_oneshot_wait)
            pointCloud = pointCloud.transpose()
            print('pointCloud', pointCloud)

            # print('pointCloud', pointCloud)
            # Take only the points until a distance of 1.2

            # Find highest point for this table
            maxi = - math.inf
            for point in range(len(pointCloud[1, :])):
                if pointCloud[1, point] > maxi:
                    maxi = pointCloud[1, point]

            tablesMaxHigh[0, j] = maxi

            # Next table index
            discoverTableCounter = discoverTableCounter + 1

            # Find the target table and define the neighbourhood of the table where the robot will be sent

            # Vector containing the ID of tables having objects on them
            objectsTablesID = []

            if discoverTableCounter > 2:
                minHigh = min(tablesMaxHigh[0])
                # print('tablesMaxHigh', tablesMaxHigh)
                for j in range(len(tablesMaxHigh[0])):
                    if tablesMaxHigh[0, j] == minHigh:
                        targetID = j
                    else:
                        objectsTablesID.append(j)

                # Define the neighbourhood of the tables with objects
                table1Neighbours = np.zeros((5, 2))
                table2Neighbours = np.zeros((5, 2))

                tab1ID = objectsTablesID[0]
                tab2ID = objectsTablesID[1]

                # Divide 360° in 5 to takes pictures from 5 spots equally spaced
                angles = np.linspace(0, 2*math.pi, num=6)

                # Focus on table 1
                centerTable = tablesRealCenter[tab1ID, :]

                for k in range(5):
                    j = 0.5
                    table1Neighbours[k, :] = centerTable + [math.cos(angles[k]) * j, math.sin(angles[k]) * j]
                    table1Neighbours[k, 0] = round((table1Neighbours[k, 0] + 7.5)/resolution) + 1
                    table1Neighbours[k, 1] = round((table1Neighbours[k, 1] + 7.5)/resolution) + 1

                    # Verify that the cell is free
                    a = int(table1Neighbours[k, 0])
                    b = int(table1Neighbours[k, 1])
                    while statesMap[a, b] != 0:
                        j = j + 0.1
                        table1Neighbours[k, :] = centerTable + [math.cos(angles[k]) * j, math.sin(angles[k]) * j]
                        table1Neighbours[k, 0] = round((table1Neighbours[k, 0] + 7.5)/resolution) + 1
                        table1Neighbours[k, 1] = round((table1Neighbours[k, 1] + 7.5)/resolution) + 1
                        a = int(table1Neighbours[k, 0])
                        b = int(table1Neighbours[k, 1])

                # Focus on table 2
                centerTable = tablesRealCenter[tab2ID, :]

                for k in range(5):
                    j = 0.5
                    table2Neighbours[k, :] = centerTable + [math.cos(angles[k]) * j, math.sin(angles[k]) * j]
                    table2Neighbours[k, 0] = round((table2Neighbours[k, 0] + 7.5)/resolution) + 1
                    table2Neighbours[k, 1] = round((table2Neighbours[k, 1] + 7.5)/resolution) + 1

                    # Verify that the cell is free
                    a = int(table2Neighbours[k, 0])
                    b = int(table2Neighbours[k, 1])
                    while statesMap[a, b] != 0:
                        j = j + 0.1
                        table2Neighbours[k, :] = centerTable + [math.cos(angles[k]) * j, math.sin(angles[k]) * j]
                        table2Neighbours[k, 0] = round((table2Neighbours[k, 0] + 7.5)/resolution) + 1
                        table2Neighbours[k, 1] = round((table2Neighbours[k, 1] + 7.5)/resolution) + 1
                        a = int(table2Neighbours[k, 0])
                        b = int(table2Neighbours[k, 1])

                tabID = targetID

                targetNeighbours = np.zeros((4, 2))

                # Determination of neighbours (N-S-E-W) of the target table
                for j in range(6):
                    a = int(tablesCenters[tabID, 0] + j)
                    b = int(tablesCenters[tabID, 1])
                    if statesMap[a, b] == 0:
                        targetNeighbours[0, :] = [a, b]
                        break

                for j in range(6):
                    a = int(tablesCenters[tabID, 0])
                    b = int(tablesCenters[tabID, 1] - j)
                    if statesMap[a, b] == 0:
                        targetNeighbours[1, :] = [a, b]
                        break

                for j in range(6):
                    a = int(tablesCenters[tabID, 0] - j)
                    b = int(tablesCenters[tabID, 1])
                    if statesMap[a, b] == 0:
                        targetNeighbours[2, :] = [a, b]
                        break

                for j in range(6):
                    a = int(tablesCenters[tabID, 0])
                    b = int(tablesCenters[tabID, 1] + j)
                    if statesMap[a, b] == 0:
                        targetNeighbours[3, :] = [a, b]
                        break

                # Initialize to empty the matrices that will store the
                # points taken with depth camera
                ptsTable1 = []
                ptsTable2 = []
                ptsTarget = []
                ptsObjects1 = []
                ptsObjects2 = []

                # Determine which table the robot has to model first
                tabToModel = table1Neighbours
                tabID = objectsTablesID[0]
                # Determine which neighbour has to be visited
                neighbour = 0

                mat = np.matrix(table1Neighbours)
                with open('savetable1Neighbours.txt', 'wb') as f:
                    for line in mat:
                        np.savetxt(f, line, fmt='%.2f', delimiter=',')
                mat = np.matrix(table2Neighbours)
                with open('savetable2Neighbours.txt', 'wb') as f:
                    for line in mat:
                        np.savetxt(f, line, fmt='%.2f', delimiter=',')
                mat = np.matrix(targetNeighbours)
                with open('savetargetNeighbours.txt', 'wb') as f:
                    for line in mat:
                        np.savetxt(f, line, fmt='%.2f', delimiter=',')
                mat = np.matrix(targetID)
                with open('savetargetID.txt', 'wb') as f:
                    for line in mat:
                        np.savetxt(f, line, fmt='%.2f', delimiter=',')
                mat = np.matrix(tablesRealCenter)
                with open('savetablesRealCenter.txt', 'wb') as f:
                    for line in mat:
                        np.savetxt(f, line, fmt='%.2f', delimiter=',')
                mat = np.matrix(tablesCenters)
                with open('savetablesCenters.txt', 'wb') as f:
                    for line in mat:
                        np.savetxt(f, line, fmt='%.2f', delimiter=',')
                mat = np.matrix(objectsTablesID)
                with open('saveobjectsTablesID.txt', 'wb') as f:
                    for line in mat:
                        np.savetxt(f, line, fmt='%.2f', delimiter=',')
                # save('table1Neighbours.mat','table1Neighbours')
                # save('table2Neighbours.mat','table2Neighbours')
                # save('targetNeighbours.mat', 'targetNeighbours')
                # save('targetID.mat','targetID')
                # save('tablesCentersReal.mat','tablesCentersReal')
                # save('tablesCentersMat.mat','tablesCentersMat')

            fsm = 'astar'
            print('Switching to state: ', fsm)

        # Use the depth camera to take picture of the tables in order to fill in ptsTableX, ptsObjectsX and ptsTarget
        elif fsm == 'modelTable':
            modelTableFlag = True

            # get rgb camera position
            [res, rgbdPos] = vrep.simxGetObjectPosition(clientID, h['rgbdCasing'], -1, vrep.simx_opmode_oneshot_wait)
            vrchk(vrep, res, True)

            # get rgb camera angle
            [res, rgbdEuler] = vrep.simxGetObjectOrientation(clientID, h['rgbdCasing'], -1, vrep.simx_opmode_oneshot_wait)
            vrchk(vrep, res, True)

            # First, use a large angle to have a global view
            res = vrep.simxSetFloatSignal(clientID, 'rgbd_sensor_scan_angle', math.pi/4, vrep.simx_opmode_oneshot_wait)
            vrchk(vrep, res)

            # take a depth picture
            res = vrep.simxSetIntegerSignal(clientID, 'handle_xyz_sensor', 1, vrep.simx_opmode_oneshot_wait)
            vrchk(vrep, res)

            vrep.simxSynchronousTrigger(clientID)
            vrep.simxGetPingTime(clientID)

            # Store this picture in pts
            pts = youbot_xyz_sensor(vrep, h, vrep.simx_opmode_oneshot_wait)
            pts = pts.transpose()
            # print('prim sensors', pts)

            # Only keep points within 1.2 meter, to focus on the table
            pts = pts[0:4, pts[3, :] < 1.2]
            # Only keep points far enough to avoid catch robot points
            pts = pts[0:4, pts[3, :] > 0.25]
            # print('prim prima', pts)

            # Invert 3rd and 1st line to get XYZ in the right order
            pts = np.vstack([pts[2, :], pts[0, :], pts[1, :]])
            # print('prima', pts)
            trf = trans_rot_matrix(rgbdEuler, rgbdPos)  # check the file trans_rot_matrix for explanation
            # Apply the transfer function to get pts in real coordinates
            # pts = homtrans(trf, pts)
            # pts = np.array(pts)

            pts = homtrans(trf, pts)
            # print('dopo', pts)

            # Table height is 185 mm
            # we only keep points with a height between 80 and 150 mm (keep
            # margin) to identify the table by removing parasite points
            ptsTable = pts[0:3, pts[2, :] < 0.15]
            # print('prima', ptsTable)
            ptsTableInter = ptsTable[0:3, ptsTable[2, :] > 0.08]
            # print('primaInter', ptsTableInter)
            ptsTable = np.vstack([ptsTableInter[0, :], ptsTableInter[1, :]])

            print('tabID', tabID)
            # When not dealing with target table, focus also on objects
            if tabID != targetID:
                # To know object points, keep the one with high above 0.187m
                # (small margin with the 0.185m of table high)
                ptsObject = pts[0:3, pts[2, :] > 0.187]

                # Reduce angle view to focus more on objects
                res = vrep.simxSetFloatSignal(clientID, 'rgbd_sensor_scan_angle', math.pi/10, vrep.simx_opmode_oneshot_wait)
                vrchk(vrep, res)

                # take a depth picture
                res = vrep.simxSetIntegerSignal(clientID, 'handle_xyz_sensor', 1, vrep.simx_opmode_oneshot_wait)
                vrchk(vrep, res)
                # Store this picture in pts
                pts = youbot_xyz_sensor(vrep, h, vrep.simx_opmode_oneshot_wait)
                pts = pts.transpose()
                # Only keep points within 1.2 meter, to focus on the table
                pts = pts[0:3, pts[3, :] < 1.2]

                # Invert 3rd and 1st line to get XYZ in the right order
                pts = np.vstack([pts[2, :], pts[0, :], pts[1, :]])
                # Apply the transfer function to get pts in real coordinates
                # pts = homtrans(trf, pts)
                pts = np.array(pts)
                pts = homtrans(trf, pts)

                # To know object points, keep the one with high above 0.187m
                # (small margin with the 0.185m of table high)
                ptsObject_focus = pts[0:3, pts[2, :] > 0.187]

            # Check which table is currently analyzed
            if tabID == objectsTablesID[0]:
                # save points of table 1 in a matrix and remove the multiple points

                if type(ptsTable1) == list:
                    tempMatrix = np.round(250*ptsTable)/250
                    ptsTable1 = np.hstack([ptsTable, tempMatrix])
                    # c = c.tolist()
                else:
                    tempMatrix = np.round(250*ptsTable1)/250
                    ptsTable1 = np.hstack([ptsTable1, tempMatrix])
                    # c = c.tolist()
                # ptsTable1 = np.unique(ptsTable1, return_index=True, return_inverse=True, axis=0)
                # ptsTable1 = ptsTable1[0]

                if type(ptsObjects1) == list:
                    tempMatrix1 = np.round(250*ptsObject)/250
                    tempMatrix2 = np.round(250*ptsObject_focus)/250
                    ptsObjects1 = np.hstack([ptsObject, tempMatrix1, tempMatrix2])
                    # c = c.tolist()
                else:
                    tempMatrix1 = np.round(250*ptsObjects1)/250
                    tempMatrix2 = np.round(250*ptsObject_focus)/250
                    ptsObjects1 = np.hstack([ptsObjects1, tempMatrix1, tempMatrix2])
                    # c = c.tolist()
                # ptsObjects1 = np.unique(ptsObjects1, return_index=True, return_inverse=True, axis=0)
                # ptsObjects1 = ptsObjects1[0]

                # Increment the neighbour counter to send the youbot to next table neighbour
                neighbour = neighbour + 1
                if neighbour > 4:
                    # Visit the second table if finished with first one
                    tabToModel = table2Neighbours
                    tabID = objectsTablesID[1]
                    neighbour = 0
                fsm = 'astar'
                print('Switching to state: ', fsm)

            elif tabID == objectsTablesID[1]:
                # save points of table 2 in a matrix and remove the multiple points

                if type(ptsTable2) == list:
                    tempMatrix = np.round(250*ptsTable)/250
                    ptsTable2 = np.hstack([ptsTable, tempMatrix])
                    # c = c.tolist()
                else:
                    tempMatrix = np.round(250*ptsTable2)/250
                    ptsTable2 = np.hstack([ptsTable2, tempMatrix])
                    # c = c.tolist()
                # ptsTable2 = np.unique(c, return_index=True, return_inverse=True, axis=0)
                # ptsTable2 = ptsTable2[0]

                if type(ptsObjects2) == list:
                    tempMatrix1 = np.round(250*ptsObject)/250
                    tempMatrix2 = np.round(250*ptsObject_focus)/250
                    ptsObjects2 = np.hstack([ptsObject, tempMatrix1, tempMatrix2])
                    # c = c.tolist()
                else:
                    tempMatrix1 = np.round(250*ptsObjects2)/250
                    tempMatrix2 = np.round(250*ptsObject_focus)/250
                    ptsObjects2 = np.hstack([ptsObjects2, tempMatrix1, tempMatrix2])
                #     c = c.tolist()
                # ptsObjects2 = np.unique(c, return_index=True, return_inverse=True, axis=0)
                # ptsObjects2 = ptsObjects2[0]

                # Increment the neighbour counter to send the youbot to next table neighbour
                neighbour = neighbour + 1
                if neighbour > 4:
                    tabID = targetID
                    modelTableFlag = False
                    neighbour = 0

                fsm = 'astar'
                print('Switching to state: ', fsm)
            else:
                # save points of target table in a matrix and remove the multiple points
                if type(ptsTarget) == list:
                    tempMatrix = np.round(250*ptsTable)/250
                    ptsTarget = np.hstack([ptsTable, tempMatrix])
                    # c = c.tolist()
                else:
                    tempMatrix = np.round(250*ptsTarget)/250
                    ptsTarget = np.hstack([ptsTarget, tempMatrix])
                #     c = c.tolist()
                # ptsTarget = np.unique(c, return_index=True, return_inverse=True, axis=0)
                # ptsTarget = ptsTarget[0]

                # Increment the neighbour counter to send bot to next table neighbour
                neighbour = neighbour + 1
                if neighbour > 3:

                    mat = np.matrix(ptsObjects1)
                    with open('saveptsObjects1.txt', 'wb') as f:
                        for line in mat:
                            np.savetxt(f, line, fmt='%.2f', delimiter=',')
                    mat = np.matrix(ptsObjects2)
                    with open('saveptsObjects2.txt', 'wb') as f:
                        for line in mat:
                            np.savetxt(f, line, fmt='%.2f', delimiter=',')
                    mat = np.matrix(ptsTable1)
                    with open('saveptsTable1.txt', 'wb') as f:
                        for line in mat:
                            np.savetxt(f, line, fmt='%.2f', delimiter=',')
                    mat = np.matrix(ptsTable2)
                    with open('saveptsTable2.txt', 'wb') as f:
                        for line in mat:
                            np.savetxt(f, line, fmt='%.2f', delimiter=',')
                    mat = np.matrix(ptsTarget)
                    with open('saveptsTarget.txt', 'wb') as f:
                        for line in mat:
                            np.savetxt(f, line, fmt='%.2f', delimiter=',')

                    fsm = 'imageAnalysis'
                    print('Switching to state: ', fsm)

                else:
                    modelTableFlag = False
                    fsm = 'astar'
                    print('Switching to state: ', fsm)

        # Analyze the points taken through the depth pictures to infer objects positions and table center and differentiate table 1 and 2
        elif fsm == 'imageAnalysis':
            modelTableFlag = False

            tab1ID = objectsTablesID[0]
            # xMean = np.mean(ptsTable1[0, :])
            # yMean = np.mean(ptsTable1[1, :])

            # # Remove parasite points for table (points placed to far from it)
            # a = np.sqrt((ptsTable1[0, :] - xMean)**2 + (ptsTable1[1, :] - yMean)**2)
            # ptsTable1 = ptsTable1[:, a < 0.6]
            #
            # xCenter = (np.max(ptsTable1[0, :]) + np.min(ptsTable1[0, :]))/2
            # yCenter = (np.max(ptsTable1[1, :]) + np.min(ptsTable1[1, :]))/2
            # preciseCenter1 = np.zeros((1, 2))
            # preciseCenter1[0, :] = [xCenter, yCenter]
            # tablesRealCenter[tab1ID, :] = preciseCenter1

            # Remove parasite points for objects (points placed to far from it)
            xCenter = tablesRealCenter[tab1ID, 0]
            yCenter = tablesRealCenter[tab1ID, 1]
            a = np.sqrt((ptsObjects1[0, :] - xCenter)**2 + (ptsObjects1[1, :] - yCenter)**2)
            ptsObjects1 = ptsObjects1[:, a < 0.8]

            # # Considering the second table
            tab2ID = objectsTablesID[1]
            # xMean = np.mean(ptsTable2[0, :])
            # yMean = np.mean(ptsTable2[1, :])
            #
            # # Remove parasite points for table (points placed to far from it)
            # a = np.sqrt((ptsTable2[0, :] - xMean)**2 + (ptsTable2[1, :] - yMean)**2)
            # ptsTable2 = ptsTable2[:, a < 0.6]
            #
            # xCenter = (np.max(ptsTable2[0, :]) + np.min(ptsTable2[0, :]))/2
            # yCenter = (np.max(ptsTable2[1, :]) + np.min(ptsTable2[1, :]))/2
            # preciseCenter2 = np.zeros((1, 2))
            # preciseCenter2[0, :] = [xCenter, yCenter]
            # tablesRealCenter[tab1ID, :] = preciseCenter2

            # Remove parasite points for objects (points placed to far from it)
            xCenter = tablesRealCenter[tab2ID, 0]
            yCenter = tablesRealCenter[tab2ID, 1]
            a = np.sqrt((ptsObjects2[0, :] - xCenter)**2 + (ptsObjects2[1, :] - yCenter)**2)
            ptsObjects2 = ptsObjects2[:, a < 0.8]

            # Through the ptsTarget points, we can have a more accurate idea of the target table points and center
            # point. So update them
            # xMean = np.mean(ptsTarget[0, :])
            # yMean = np.mean(ptsTarget[1, :])

            # Remove parasite points for table (points situated to far from it)
            xCenter = tablesRealCenter[targetID, 0]
            yCenter = tablesRealCenter[targetID, 1]
            a = np.sqrt((ptsTarget[0, :] - xCenter)**2 + (ptsTarget[1, :] - yCenter)**2)
            ptsTarget = ptsTarget[:, a < 0.6]

            # xCenter = (np.max(ptsTarget[0, :]) + np.min(ptsTarget[0, :]))/2
            # yCenter = (np.max(ptsTarget[1, :]) + np.min(ptsTarget[1, :]))/2
            # preciseCenter = np.zeros((1, 2))
            # preciseCenter[0, :] = [xCenter, yCenter]
            # tablesRealCenter[targetID, :] = [xCenter, yCenter]

            # ADD THIS
            from sklearn.cluster import KMeans

            # Regrouping the points to find the centers of the objects on the tables.
            distToClusterCenters1 = math.inf

            kmeans1 = KMeans(init="random", n_clusters=5, n_init=50, max_iter=300, random_state=None)
            kmeans1.fit(ptsObjects1.transpose())
            centerObject1 = kmeans1.cluster_centers_
            idObject1 = kmeans1.labels_

            distToClusterCenters2 = math.inf

            kmeans2 = KMeans(init="random", n_clusters=5, n_init=50, max_iter=300, random_state=None)
            kmeans2.fit(ptsObjects2.transpose())
            centerObject2 = kmeans2.cluster_centers_
            idObject2 = kmeans2.labels_

            # Check the distances between the mean objects center points and the objects center points
            # The table for which the distance is the higher has many space between objects. So it is table 1.
            # So invert ID if needed
            meanCenterObject1 = [np.mean(centerObject1[:, 0]), np.mean(centerObject1[:, 1]), np.mean(centerObject1[:, 2])]
            meanCenterObject2 = [np.mean(centerObject2[:, 0]), np.mean(centerObject2[:, 1]), np.mean(centerObject2[:, 2])]
            distanceToCenters1 = 0
            distanceToCenters2 = 0
            for i in range(5):
                distanceToCenters1 = distanceToCenters1 + sum((meanCenterObject1 - centerObject1[i, :])**2)

            for i in range(5):
                distanceToCenters2 = distanceToCenters2 + sum((meanCenterObject2 - centerObject2[i, :])**2)

            if distanceToCenters1 < distanceToCenters2:
                temp = ptsObjects1
                ptsObjects1 = ptsObjects2
                ptsObjects2 = temp

                temp = ptsTable1
                ptsTable1 = ptsTable2
                ptsTable2 = temp

                temp = idObject1
                idObject1 = idObject2
                idObject2 = temp

                temp = centerObject1
                centerObject1 = centerObject2
                centerObject2 = temp

                temp = objectsTablesID[0]
                objectsTablesID[0] = objectsTablesID[1]
                objectsTablesID[1] = temp


            # Plot table 1
            plt.close()
            ax = plt.axes(projection='3d')
            ax.scatter3D(ptsTable1[0, :], ptsTable1[1, :])
            plt.title('Table 1')
            plt.show()

            # Plot the clustering for table 1
            plt.close()
            ax = plt.axes(projection='3d')
            ax.scatter3D(ptsObjects1[0, :], ptsObjects1[1, :], ptsObjects1[2, :])
            ax.scatter3D(centerObject1[:, 0], centerObject1[:, 1], centerObject1[:, 2])
            plt.title('Objects points and cluster centers - Table 1')
            plt.show()

            # Plot table 2
            plt.close()
            ax = plt.axes(projection='3d')
            ax.scatter3D(ptsTable2[0, :], ptsTable2[1, :])
            plt.title('Table 2')
            plt.show()

            # Plot the clustering for table 2
            plt.close()
            ax = plt.axes(projection='3d')
            ax.scatter3D(ptsObjects2[0, :], ptsObjects2[1, :], ptsObjects2[2, :])
            ax.scatter3D(centerObject2[:, 0], centerObject2[:, 1], centerObject2[:, 2])
            plt.title('Objects points and cluster centers - Table 2')
            plt.show()

            mat = np.matrix(idObject1)
            with open('saveidObject1.txt', 'wb') as f:
                for line in mat:
                    np.savetxt(f, line, fmt='%.2f', delimiter=',')

            mat = np.matrix(idObject2)
            with open('saveidObject2.txt', 'wb') as f:
                for line in mat:
                    np.savetxt(f, line, fmt='%.2f', delimiter=',')

            mat = np.matrix(centerObject1)
            with open('savecenterObject1.txt', 'wb') as f:
                for line in mat:
                    np.savetxt(f, line, fmt='%.2f', delimiter=',')

            mat = np.matrix(centerObject2)
            with open('savecenterObject2.txt', 'wb') as f:
                for line in mat:
                    np.savetxt(f, line, fmt='%.2f', delimiter=',')

            mat = np.matrix(objectsTablesID)
            with open('saveobjectsTablesID.txt', 'wb') as f:
                for line in mat:
                    np.savetxt(f, line, fmt='%.2f', delimiter=',')

            # fprintf('table 1 is (%f,%f)', tablesCentersReal(objectsTablesID(1),1),tablesCentersReal(objectsTablesID(1),2))

            tabID = objectsTablesID[0]
            objectID = 0
            fsm = 'computedestObjects'
            print('Switching to state: ', fsm)

        # Compute the positions where we will send to robot to place the blocks on the target table
        elif fsm == 'computedestObjects':
            # If n objects to place on the target, find target table free neighbour
            # cells with the more space between them

            # For milestone 2a, we have 5 destinations
            destObjects = np.zeros((5, 2))
            # divide 360° in 5 for milestone 2a
            angles = np.linspace(0, 2*math.pi, num=6)
            print('angles', angles)
            centerTarget = tablesRealCenter[targetID, :]

            for k in range(5):
                j = 0.5
                # destObjects[k, :] = centerTarget + [math.cos(angles[k]) * j, math.sin(angles[k]) * j]
                destObjects[k, 0] = centerTarget[0] + (math.cos(angles[k]) * j)
                destObjects[k, 1] = centerTarget[1] + (math.sin(angles[k]) * j)
                destObjects[k, 0] = round((destObjects[k, 0] + 7.5)/resolution) + 1
                destObjects[k, 1] = round((destObjects[k, 1] + 7.5)/resolution) + 1

                # Verify that the cell is a free cell
                while statesMap[int(destObjects[k, 0]), int(destObjects[k, 1])] != 0:
                    j = j + 0.1
                    # destObjects[k, :] = centerTarget + [math.cos(angles[k]) * j, math.sin(angles[k]) * j]
                    destObjects[k, 0] = centerTarget[0] + (math.cos(angles[k]) * j)
                    destObjects[k, 1] = centerTarget[1] + (math.sin(angles[k]) * j)
                    destObjects[k, 0] = round((destObjects[k, 0] + 7.5)/resolution) + 1
                    destObjects[k, 1] = round((destObjects[k, 1] + 7.5)/resolution) + 1

            mat = np.matrix(destObjects)
            with open('savedestObjects.txt', 'wb') as f:
                for line in mat:
                    np.savetxt(f, line, fmt='%.2f', delimiter=',')
            mat = np.matrix(centerTarget)
            with open('savecenterTarget.txt', 'wb') as f:
                for line in mat:
                    np.savetxt(f, line, fmt='%.2f', delimiter=',')

            fsm = 'calculateObjectGoal'
            print('Switching to state: ', fsm)

        # For each object to grasp, find nearest cell to send the robot to
        elif fsm == 'calculateObjectGoal':

            if tabID == targetID:
                centerObject = centerObject1
                tableCenter = [tablesRealCenter[objectsTablesID[0], 0], tablesRealCenter[objectsTablesID[0], 1]]
            else:
                centerObject = centerObject2
                tableCenter = [tablesRealCenter[objectsTablesID[1], 0], tablesRealCenter[objectsTablesID[1], 1]]

            posObject = [centerObject[objectID, 0], centerObject[objectID, 1]]

            print('objectID', objectID)
            a = posObject[1] - tableCenter[1]
            b = posObject[0] - tableCenter[0]
            angle = math.atan2(a, b)

            # if posObject[0] < tableCenter[0]:
            #     angle = angle + math.pi

            j = 0.5
            print('tablecenter', tableCenter)
            print('posObject', posObject)
            posNearObject = np.zeros((1, 2))
            posNearObject[0, 0] = tableCenter[0] + (math.cos(angle) * j)
            posNearObject[0, 1] = tableCenter[1] + (math.sin(angle) * j)
            print('posNearObject', posNearObject)
            posNearObject[0, 0] = round((posNearObject[0, 0] + 7.5)/resolution) + 1
            posNearObject[0, 1] = round((posNearObject[0, 1] + 7.5)/resolution) + 1

            # Verify that the cell is a free cell
            while statesMap[int(posNearObject[0, 0]), int(posNearObject[0, 1])] != 0:
                j = j + 0.1
                posNearObject[0, 0] = tableCenter[0] + (math.cos(angle) * j)
                posNearObject[0, 1] = tableCenter[1] + (math.sin(angle) * j)
                posNearObject[0, 0] = round((posNearObject[0, 0] + 7.5)/resolution) + 1
                posNearObject[0, 1] = round((posNearObject[0, 1] + 7.5)/resolution) + 1

            print('posNearObject', posNearObject)

            fsm = 'astar'
            print('Switching to state: ', fsm)

        # If needed rotate the robot parallel to the table (table on its left) and slide closer or further
        elif fsm == 'rotateAndSlide':

            for i in range(50):
                vrep.simxSynchronousTrigger(clientID)
                vrep.simxGetPingTime(clientID)

            [res, youbotPos] = vrep.simxGetObjectPosition(clientID, h['ref'], -1, vrep.simx_opmode_buffer)
            vrchk(vrep, res, True)
            # Get initial sliding position
            startingPoint = [youbotPos[0], youbotPos[1]]

            # If the robot should slide closer to the table, add the rotation phase before the sliding phase
            if slideCloser:

                # if holding and object and have to slide closer, we know that the table of interest is the target
                if holdObject:
                    centerToReach = centerTarget
                    print('centerTarget', centerTarget)
                    a = - (youbotPos[0] - centerToReach[0])
                    b = - (centerToReach[1] - youbotPos[1])
                    rotationAngle = math.atan2(a, b) - math.pi/2
                    print('a', a)
                    print('b', b)
                    print('math.atan2', math.atan2(a, b))
                else:
                    centerToReach = tableCenter
                    a = - (youbotPos[0] - centerToReach[0])
                    b = - (centerToReach[1] - youbotPos[1])
                    rotationAngle = math.atan2(a, b) - math.pi/2
                    print('youbotPos', youbotPos)
                    print('centerToReach', centerToReach)
                    print('a', a)
                    print('b', b)
                    print('math.atan2', math.atan2(a, b))

                print('rotationAngle', rotationAngle)
                #
                # a = youbotPos[0] - centerToReach[0]
                # b = centerToReach[1] - youbotPos[1]
                # rotationAngle = math.atan2(a, b) + math.pi/2

                # rotateRightVel = angdiff(youbotEuler[2], rotationAngle)
                #
                # if rotateRightVel > math.pi:
                #     rotateRightVel = rotateRightVel - math.pi*2
                #
                # if rotateRightVel < - math.pi:
                #     rotateRightVel = rotateRightVel + math.pi*2
                #
                # if abs(angdiff(youbotEuler[2], rotationAngle)) < 0.05:
                #     rotateRightVel = 0

                # # Find the angle to ensure table on the left
                # if youbotPos[1] >= centerToReach[1]:
                #     angle = - angle - math.pi/2
                # else:
                #     angle = math.pi - angle - math.pi/2

                # Launch the rotation
                # rotateRightVel = 1
                # The orientation of youbot is obtained:
                rotation = True

                while rotation:
                    [res, youbotEuler] = vrep.simxGetObjectOrientation(clientID, h['ref'], -1, vrep.simx_opmode_buffer)
                    vrchk(vrep, res, True)

                    rotateRightVel = angdiff(youbotEuler[2], rotationAngle)

                    if rotateRightVel > math.pi:
                        rotateRightVel = rotateRightVel - math.pi*2

                    if rotateRightVel < - math.pi:
                        rotateRightVel = rotateRightVel + math.pi*2

                    rotateRightVel = angdiff(youbotEuler[2], rotationAngle)

                    if abs(angdiff(youbotEuler[2], rotationAngle)) < 0.05:
                        rotateRightVel = 0
                        rotation = False

                    h = youbot_drive(vrep, h, forwBackVel, rightVel, rotateRightVel)
                    forwBackVel = 0
                    rightVel = 0

                    vrep.simxSynchronousTrigger(clientID)
                    vrep.simxGetPingTime(clientID)

                # Update wheel velocities.
                forwBackVel = 0
                rightVel = 0
                h = youbot_drive(vrep, h, forwBackVel, rightVel, rotateRightVel)
                vrep.simxSynchronousTrigger(clientID)
                vrep.simxGetPingTime(clientID)

                # h = youbot_drive(vrep, h, forwBackVel, rightVel, rotateRightVel)

                # rotateRightVel = 0
                #
                # for i in range(20):
                #     vrep.simxSynchronousTrigger(clientID)
                #     vrep.simxGetPingTime(clientID)

                # Proceed to the computation to know how far we are from
                # the table and which distance has to be travelled
                a = startingPoint[1] - centerToReach[1]
                b = startingPoint[0] - centerToReach[0]
                angle = math.atan2(a, b)

                # if startingPoint[0] < centerToReach[0]:
                #     angle = angle + math.pi

                closestPoint = [(centerToReach[0] + 0.63 * math.cos(angle)), (centerToReach[1] + 0.63 * math.sin(angle))]
                # print('closestPoint', closestPoint)

                totDist = np.sqrt((startingPoint[0] - closestPoint[0])**2 + (startingPoint[1] - closestPoint[1])**2)
                # print('totDist', totDist)

            travelledDist = 0

            # Make the robot slide until it has covered the distance we want
            while travelledDist < totDist - .06:
                vrep.simxSynchronousTrigger(clientID)
                vrep.simxGetPingTime(clientID)
                if slideCloser:
                    h = youbot_drive(vrep, h, 0, 0.1, 0)
                else:
                    h = youbot_drive(vrep, h, 0, -0.2, 0)

                [res, youbotPos] = vrep.simxGetObjectPosition(clientID, h['ref'], -1, vrep.simx_opmode_buffer)
                vrchk(vrep, res, True)
                travelledDist = np.sqrt((startingPoint[0] - youbotPos[0])**2 + (startingPoint[1] - youbotPos[1])**2)
                # print('travelledDist', travelledDist)

            # Stops wheels if position reached
            res = vrep.simxSetJointTargetVelocity(clientID, h['wheelJoints'][0], 0, vrep.simx_opmode_oneshot)
            vrchk(vrep, res)
            # set target velocity of the joint to 0
            res = vrep.simxSetJointTargetVelocity(clientID, h['wheelJoints'][1], 0, vrep.simx_opmode_oneshot)
            vrchk(vrep, res)
            # set target velocity of the joint to 0
            res = vrep.simxSetJointTargetVelocity(clientID, h['wheelJoints'][2], 0, vrep.simx_opmode_oneshot)
            vrchk(vrep, res)
            # set target velocity of the joint to 0
            res = vrep.simxSetJointTargetVelocity(clientID, h['wheelJoints'][3], 0, vrep.simx_opmode_oneshot)
            vrchk(vrep, res)

            for i in range(50):
                vrep.simxSynchronousTrigger(clientID)
                vrep.simxGetPingTime(clientID)

            if not slideCloser:
                if not holdObject:
                    fsm = 'calculateObjectGoal'
                    print('Switching to state: ', fsm)
                else:
                    fsm = 'astar'
                    print('Switching to state: ', fsm)

            else:
                if not holdObject:
                    fsm = 'preciseobjectAnalysis'
                    print('Switching to state: ', fsm)
                else:
                    fsm = 'armMotion'
                    print('Switching to state: ', fsm)

            slideCloser = not slideCloser

        # Take a precise picture of the object we want to grasp to have a quite accurate grasping
        elif fsm == 'preciseobjectAnalysis':
            # get rgb camera position
            [res, rgbdPos] = vrep.simxGetObjectPosition(clientID, h['rgbdCasing'], -1, vrep.simx_opmode_oneshot_wait)
            vrchk(vrep, res, True)

            a = rgbdPos[0] - centerObject[objectID, 0]
            b = centerObject[objectID, 1] - rgbdPos[1]
            angle = math.atan2(a, b) + math.pi/2

            # if centerObject[objectID, 1] - rgbdPos[1] > 0:
            #     angle = angle + math.pi

            # Orientate rgbd camera to the object of interest
            vrep.simxSetObjectOrientation(clientID, h['rgbdCasing'], -1, [0, 0, angle], vrep.simx_opmode_oneshot)

            # get rgb camera angle
            [res, rgbdEuler] = vrep.simxGetObjectOrientation(clientID, h['rgbdCasing'], -1, vrep.simx_opmode_oneshot_wait)
            vrchk(vrep, res, True)

            # Use a very small view to focus on the object
            res = vrep.simxSetFloatSignal(clientID, 'rgbd_sensor_scan_angle', math.pi/12, vrep.simx_opmode_oneshot_wait)
            vrchk(vrep, res)

            # take a depth picture
            res = vrep.simxSetIntegerSignal(clientID, 'handle_xyz_sensor', 1, vrep.simx_opmode_oneshot_wait)
            vrchk(vrep, res)

            vrep.simxSynchronousTrigger(clientID)
            vrep.simxGetPingTime(clientID)

            # Store picture in pts
            pts = youbot_xyz_sensor(vrep, h, vrep.simx_opmode_oneshot_wait)
            pts = pts.transpose()

            # Only keep points within 0.8 meter, to focus on the table
            pts = pts[0:3, pts[3, :] < 0.8]

            # Invert 3rd and 1st line to get XYZ in the right order
            pts = np.vstack([pts[2, :], pts[0, :], pts[1, :]])
            trf = trans_rot_matrix(rgbdEuler, rgbdPos)  # check the file trans_rot_matrix for explanation
            # Apply transfer function to get real coordinates
            # pts = np.array(pts)
            ptsObject = homtrans(trf, pts)

            # Get points high enough (= remove table points)
            ptsObject = ptsObject[:, ptsObject[2, :] > 0.187]
            # print('ptsObj', ptsObject)
            # ptsObject = ptsObjectT.transpose()

            # Focus on points with X and Y distances to the centers smaller than 0.06 to focus on object
            # ptsObject = np.unique([(ptsObject[ptsObject[:, 0]]) > (centerObject[objectID, 0]-0.06), 0: 2], 'rows')
            # a1 = centerObject[objectID, 0]-0.06
            # print('a1', a1)
            ptsObject = ptsObject[:, ptsObject[0, :] > centerObject[objectID, 0]-0.06]
            ptsObject = ptsObject[:, ptsObject[0, :] < centerObject[objectID, 0]+0.06]
            ptsObject = ptsObject[:, ptsObject[1, :] > centerObject[objectID, 1]-0.06]
            ptsObject = ptsObject[:, ptsObject[1, :] < centerObject[objectID, 1]+0.06]
            plt.close()
            ax = plt.axes(projection='3d')
            ax.scatter3D(ptsObject[0, :], ptsObject[1, :], ptsObject[2, :])
            # ax.scatter3D(centerObject1[:, 0], centerObject1[:, 1], centerObject1[:, 2])
            plt.title('Objects points and cluster centers - Table 1')
            # plt.show()


            fsm = 'armMotion'
            print('Switching to state: ', fsm)

        # Manage the arm to take an object or to put an object depending on the context
        elif fsm == 'armMotion':
            # Condition verify if we have to grasp an object
            if not holdObject:
                # get rgb position
                [res, rgbdPos] = vrep.simxGetObjectPosition(clientID, h['rgbdCasing'], -1, vrep.simx_opmode_oneshot_wait)
                vrchk(vrep, res, True)

                # get youbotAngle
                [res, youbotEuler] = vrep.simxGetObjectOrientation(clientID, h['ref'], -1, vrep.simx_opmode_buffer)
                vrchk(vrep, res, True)

                gripPos = np.array([0, 0.166, 0])
                trf = trans_rot_matrix(youbotEuler, youbotPos)  # check the file trans_rot_matrix for explanation
                # Apply transfer function to get real gripper coordinates
                gripPos = homtrans(trf, gripPos.transpose())

                a = np.array(ptsObject[0, :] - gripPos[0])
                # print('a', a)
                b = np.array(ptsObject[1, :] - gripPos[1])
                # print('b', b)
                distToGrip = np.hypot(a, b)
                # print('distTogrip', distToGrip)
                minDist = np.min(np.hypot(a, b))
                # print('mindist', minDist)

                # Among the object points, find the one which is the closer to the gripper
                indexNearestPoint = np.where(distToGrip == minDist)
                print('indexNearestPointbef', indexNearestPoint)
                # print('indexNearestPoint', indexNearestPoint)
                indexNearestPoint = indexNearestPoint[0]
                indexNearestPoint = indexNearestPoint[0]
                print('indexNearestPointaft', indexNearestPoint)

                # Plot the given configuration : object, bot, gripper,
                # camera, object center, nearest object point from bot

                # figure;
                # plot(ptsObject(:,1), ptsObject(:,2), '*',rgbdPos(1), rgbdPos(2), '.', gripPos(1), gripPos(2), '+',...
                #      centerObject(objectID,1),centerObject(objectID,2),'*' , youbotPos(1),  youbotPos(2), '^',...
                #      ptsObject(indexNearestPoint,1), ptsObject(indexNearestPoint,2), '*' );

                # Re-adjust center of the object to get a better manipulation with the gripper
                # kmeans = KMeans(init="random", n_clusters=1, n_init=50, max_iter=300, random_state=None)
                # kmeans.fit(ptsObject.transpose())
                # ptsObject = kmeans.cluster_centers_
                # print(ptsObject)
                # idObject1 = kmeans.labels_

                # centerObject[objectID, 0] = ptsObject[0, 0]
                # centerObject[objectID, 1] = ptsObject[0, 1]
                # centerObject[objectID, 2] = ptsObject[0, 2]

                # ptsObject(indexNearestPoint,1)

                centerObject[objectID, 0] = (centerObject[objectID, 0] + ptsObject[0, indexNearestPoint])/2
                centerObject[objectID, 1] = (centerObject[objectID, 1] + ptsObject[1, indexNearestPoint])/2

                print('centerObject', centerObject)

                # Find the angle of first joint, orientation to the object
                a = - gripPos[0] + centerObject[objectID, 0]
                b = - centerObject[objectID, 1] + gripPos[1]
                angleJ1 = math.atan2(a, b) - youbotEuler[2]
                print('angleJ1', angleJ1)

                # if centerObject[objectID, 1] - gripPos[1] > 0:
                #     angleJ1 = angleJ1 + math.pi

            # Put the gripper in vertical position

            res = vrep.simxSetJointTargetPosition(clientID, h['armJoints'][1], 0, vrep.simx_opmode_oneshot_wait)
            vrchk(vrep, res, True)
            res = vrep.simxSetJointTargetPosition(clientID, h['armJoints'][2], 0, vrep.simx_opmode_oneshot_wait)
            vrchk(vrep, res, True)
            res = vrep.simxSetJointTargetPosition(clientID, h['armJoints'][3], 0, vrep.simx_opmode_oneshot_wait)
            vrchk(vrep, res, True)
            res = vrep.simxSetJointTargetPosition(clientID, h['armJoints'][4], 0, vrep.simx_opmode_oneshot_wait)
            vrchk(vrep, res, True)

            # Stops wheels if position reached
            res = vrep.simxSetJointTargetVelocity(clientID, h['wheelJoints'][0], 0, vrep.simx_opmode_oneshot)
            vrchk(vrep, res)
            # set target velocity of the joint to 0
            res = vrep.simxSetJointTargetVelocity(clientID, h['wheelJoints'][1], 0, vrep.simx_opmode_oneshot)
            vrchk(vrep, res)
            # set target velocity of the joint to 0
            res = vrep.simxSetJointTargetVelocity(clientID, h['wheelJoints'][2], 0, vrep.simx_opmode_oneshot)
            vrchk(vrep, res)
            # set target velocity of the joint to 0
            res = vrep.simxSetJointTargetVelocity(clientID, h['wheelJoints'][3], 0, vrep.simx_opmode_oneshot)
            vrchk(vrep, res)

            # Wait for the arm to get the given angles
            # time.sleep(1)

            # Apply the computated joint 1 angle
            res = vrep.simxSetJointTargetPosition(clientID, h['armJoints'][0], angleJ1, vrep.simx_opmode_oneshot_wait)
            vrchk(vrep, res, True)

            arm = True
            armCounter = 0
            while arm:
                vrep.simxSynchronousTrigger(clientID)
                vrep.simxGetPingTime(clientID)

                armCounter += 1
                if armCounter == 50:
                    arm = False

            # Wait for the arm to get the given angle
            # time.sleep(1)

            # Condition verify if we have to grasp an object
            if not holdObject:
                # Youbot constants
                length1 = 0.147+0.0952  # initial height
                length2 = 0.155         # first arm length
                length3 = 0.135         # second arm length
                length4 = 0.15          # third arm length

                gripPos[2] = length1

                # Find height difference between object and gripper
                heightDiff = centerObject[objectID, 2] - gripPos[2]
                # Find distance between gripper and object
                DistGripObject = np.sqrt((centerObject[objectID, 0] - gripPos[0])**2 + (centerObject[objectID, 1] - gripPos[1])**2) - 0.005

                # System of equations to find angles to take
                # phi2, phi3 = symbols('phi2 phi3')
                # print('DistGripObject', DistGripObject)
                # print('heightDiff', heightDiff)

                def func(phi):
                    return [length2 * math.sin(phi[0]) + length3 * math.sin(phi[0] + phi[1]) + length4 - DistGripObject,
                            length2 * math.cos(phi[0]) + length3 * math.cos(phi[0] + phi[1]) - heightDiff]

                # # Restrict the search interval to get desired angles values
                searchInterval1 = (-1.5707963705063, 1.308996796608)
                searchInterval2 = (0, 1.2863812446594)
                # # #
                x0 = searchInterval1[1]
                y0 = searchInterval2[1]
                xguess = np.array([x0, y0])

                S = root(func, xguess, method='anderson')
                # print('S1', S1)
                interPhi2 = S.x[0]
                interPhi3 = S.x[1]
                print('interPhi2', interPhi2)
                print('interPhi3', interPhi3)

                Phi4 = math.pi/2 - (interPhi2) - (interPhi3)
                print('Phi4', Phi4)

            # Orientate arms with the angles just computed
            vrep.simxSynchronousTrigger(clientID)
            vrep.simxGetPingTime(clientID)
            res = vrep.simxSetJointTargetPosition(clientID, h['armJoints'][2], interPhi3, vrep.simx_opmode_oneshot_wait)
            vrchk(vrep, res, True)
            for i in range(10):
                vrep.simxSynchronousTrigger(clientID)
                vrep.simxGetPingTime(clientID)

            res = vrep.simxSetJointTargetPosition(clientID, h['armJoints'][3], Phi4, vrep.simx_opmode_oneshot_wait)
            vrchk(vrep, res, True)
            # time.sleep(2)
            for i in range(10):
                vrep.simxSynchronousTrigger(clientID)
                vrep.simxGetPingTime(clientID)

            res = vrep.simxSetJointTargetPosition(clientID, h['armJoints'][1], interPhi2, vrep.simx_opmode_oneshot_wait)
            vrchk(vrep, res, True)
            # time.sleep(2)
            for i in range(10):
                vrep.simxSynchronousTrigger(clientID)
                vrep.simxGetPingTime(clientID)

            arm = True
            armCounter = 0
            while arm:
                vrep.simxSynchronousTrigger(clientID)
                vrep.simxGetPingTime(clientID)

                armCounter += 1
                if armCounter == 50:
                    arm = False

            if not holdObject:
                vrep.simxSynchronousTrigger(clientID)
                vrep.simxGetPingTime(clientID)
                # Close the gripper if not object hold
                res = vrep.simxSetIntegerSignal(clientID, 'gripper_open', 0, vrep.simx_opmode_oneshot_wait)
                vrchk(vrep, res)

                # arm = True
                # armCounter = 0
                # while arm:
                #     vrep.simxSynchronousTrigger(clientID)
                #     vrep.simxGetPingTime(clientID)
                #
                # armCounter += 1
                # if armCounter == 1:
                #     arm = False
                for i in range(20):
                    vrep.simxSynchronousTrigger(clientID)
                    vrep.simxGetPingTime(clientID)
                # # Ensure gripper well closed before continue
                # # time.sleep(2)
                # arm = True
                # armCounter = 0
                # while arm:
                #     vrep.simxSynchronousTrigger(clientID)
                #     vrep.simxGetPingTime(clientID)
                #
                #     armCounter += 1
                #     if armCounter == 50:
                #         arm = False

                # We have to know that we are now holding an object
                holdObject = True

            else:
                # Open the gripper if object hold to place it on the table
                res = vrep.simxSetIntegerSignal(clientID, 'gripper_open', 1, vrep.simx_opmode_oneshot_wait)
                vrchk(vrep, res)

                # Put the gripper in vertical position to avoid hitting object
                res = vrep.simxSetJointTargetPosition(clientID, h['armJoints'][1], 0, vrep.simx_opmode_oneshot_wait)
                vrchk(vrep, res, True)
                res = vrep.simxSetJointTargetPosition(clientID, h['armJoints'][2], 0, vrep.simx_opmode_oneshot_wait)
                vrchk(vrep, res, True)
                res = vrep.simxSetJointTargetPosition(clientID, h['armJoints'][3], 0, vrep.simx_opmode_oneshot_wait)
                vrchk(vrep, res, True)
                res = vrep.simxSetJointTargetPosition(clientID, h['armJoints'][4], 0, vrep.simx_opmode_oneshot_wait)
                vrchk(vrep, res, True)
                # time.sleep(1.5)
                for i in range(30):
                    vrep.simxSynchronousTrigger(clientID)
                    vrep.simxGetPingTime(clientID)

                # arm = True
                # armCounter = 0
                # while arm:
                #     vrep.simxSynchronousTrigger(clientID)
                #     vrep.simxGetPingTime(clientID)
                #
                #     armCounter += 1
                #     if armCounter == 50:
                #         arm = False
                # We have to know that we do not hold object anymore
                holdObject = False
                objectID = objectID + 1
                if objectID > 4:
                    objectID = 0
                    tabID = tabID + 1

                # If the robot is supposed to grasp object of table 2
                if tabID == 2:
                    print('Grasping Finished')
                    p = False

            # Go back to rest position.
            #  Set each joint to their original angle, as given by startingJoints.
            for i in range(5):
                res = vrep.simxSetJointTargetPosition(clientID, h['armJoints'][i], startingJoints[i], vrep.simx_opmode_oneshot_wait)
                vrchk(vrep, res, True)
            # time.sleep(2)
            arm = True
            armCounter = 0
            while arm:
                vrep.simxSynchronousTrigger(clientID)
                vrep.simxGetPingTime(clientID)

                armCounter += 1
                if armCounter == 50:
                    arm = False
            fsm = 'rotateAndSlide'

        #
        # --- Finished ---
        #
        elif fsm == 'finished':
            print('Finish')
            # time.sleep(3)
            break
        else:
            sys.exit('Unknown state ' + fsm)
#
        # Update wheel velocities.
        h = youbot_drive(vrep, h, forwBackVel, rightVel, rotateRightVel)

        # Send a Trigger to the simulator: this will run a time step for the physic engine
        # because of the synchronous mode.
        vrep.simxSynchronousTrigger(clientID)
        vrep.simxGetPingTime(clientID)
        if counter > 10:
            end = time.time()
            timing.append(end-start)
        counter += 1

    except KeyboardInterrupt:
        cleanup_vrep(vrep, clientID)
        sys.exit('Stop simulation')


cleanup_vrep(vrep, clientID)
print('Simulation has stopped')

