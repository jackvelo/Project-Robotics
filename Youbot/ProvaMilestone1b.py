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
import time
import math
import numpy as np
import sys
import time

from cleanup_vrep import cleanup_vrep
from vrchk import vrchk
from youbot_init import youbot_init
from youbot_drive import youbot_drive
from youbot_hokuyo_init import youbot_hokuyo_init
from youbot_hokuyo import youbot_hokuyo
from youbot_xyz_sensor import youbot_xyz_sensor
from beacon import beacon_init, youbot_beacon
from utils_sim import angdiff
from Prova_obs import example_filter

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
                                                           vrep.simx_opmode_oneshot_wait)

xC1 = beacons_world_pos[0, 0]
yC1 = beacons_world_pos[0, 1]
xC2 = beacons_world_pos[1, 0]
yC2 = beacons_world_pos[1, 1]
xC3 = beacons_world_pos[2, 0]
yC3 = beacons_world_pos[2, 1]

xC = [xC1, xC2, xC3]
yC = [yC1, yC2, yC3]

R12 = math.sqrt((xC1-xC2)**2+(yC1-yC2)**2)
R23 = math.sqrt((xC2-xC3)**2+(yC2-yC3)**2)
R31 = math.sqrt((xC1-xC3)**2+(yC1-yC3)**2)

R = [R12, R23, R31]

# Parameters for controlling the youBot's wheels: at each iteration,
# those values will be set for the wheels.
# They are adapted at each iteration by the code.
forwBackVel = 0  # Move straight ahead.
rightVel = 0  # Go sideways.
rotateRightVel = 0  # Rotate.


if True:
    # representation of the 2D map explored by the youBot
    resolution = 0.25
    dim = 7.5   # house's dimension
    # x, y will be used to display the area the robot can see, by
    # selecting the points within this mesh that are within the visibility range.
    x, y = np.meshgrid(np.arange(-dim, dim + resolution, resolution), np.arange(-dim, dim + resolution, resolution))
    x, y = x.flatten(), y.flatten()
    # these are the axis
    xAxis = np.arange(-dim, dim + resolution, resolution)
    yAxis = np.arange(-dim, dim + resolution, resolution)

# dimension of the x and y axes
n = len(xAxis)

# building up the state map assuming all the states as unknown (= 1)
statesMap = np.ones((n, n), dtype=int)

# obtaining the state map size
sizeMap = statesMap.shape
xLength = sizeMap[0]
yLength = sizeMap[1]

# np.set_printoptions(threshold=sys.maxsize)

# Get the initial position
res, youbotPos = vrep.simxGetObjectPosition(clientID, h['ref'], -1, vrep.simx_opmode_buffer)
# Set the speed of the wheels to 0.
h = youbot_drive(vrep, h, forwBackVel, rightVel, rotateRightVel)
youbotPosz = youbotPos[2]

# Initialise the state machine.
fsm = 'searchAlgo'
counterSearchAlgo = 0
print('Switching to state: ', fsm)

# Send a Trigger to the simulator: this will run a time step for the physic engine
# because of the synchronous mode. Run several iterations to stabilize the simulation
for i in range(int(1./timestep)):
    vrep.simxSynchronousTrigger(clientID)
    vrep.simxGetPingTime(clientID)

timing = []
counter = 0

# from sympy import symbols, Eq, nsolve
# x, y = symbols('x y')
# eq1 = Eq(x**2 + y**2 + 6*x -6*y +8)
# eq2 = Eq(x**2 - y**2 + 14*x - 2*y)
# # solve((eq1,eq2), (x, y))
# sol_dict = nsolve((eq1,eq2), (x, y), (0, 0))
# print(sol_dict)
# # print(f'x = {sol_dict[0]}')
# # print(f'y = {sol_dict[1]}')

# x1 = -3
# x2 = -7
# y1 = 3
# y2 = 1
# r1 = math.sqrt(10)
# r2 = math.sqrt(50)
# R = math.sqrt(20)
#
# intersecX = 0.5*(x1 + x2) + (r1**2 - r2**2)/(2*R**2)*(x2 -x1) - 0.5*math.sqrt(2*(r1**2 + r2**2)/(R**2) - ((r1**2 - r2**2)**2)/(R**4) - 1)*(y2 - y1)
# intersecY = 0.5*(y1 + y2) + (r1**2 - r2**2)/(2*R**2)*(y2 -y1) + 0.5*math.sqrt(2*(r1**2 + r2**2)/(R**2) - ((r1**2 - r2**2)**2)/(R**4) - 1)*(x2 - x1)
#
# print(intersecX)
# print(intersecY)

p = True
# Start the demo.
while p:
    try:
        # Check the connection with the simulator
        start = time.time()
        if vrep.simxGetConnectionId(clientID) == -1:
            sys.exit('Lost connection to remote API.')

        # Get the position and the orientation of the robot.
        distance_beacon = youbot_beacon(vrep, clientID, beacons_handle, h, flag=False, noise=True)

        r1 = distance_beacon[0]
        r2 = distance_beacon[1]
        r3 = distance_beacon[2]
        r = [r1, r2, r3]

        xGuess = np.zeros((1, 3))
        yGuess = np.zeros((1, 3))

        # intersection point of two circles
        for mm in range(3):
            if mm == 0:
                i = 0
                j = 1
                k = 2
            elif mm == 1:
                i = 1
                j = 2
                k = 0
            elif mm == 2:
                i = 2
                j = 0
                k = 1

            # https://math.stackexchange.com/questions/256100/how-can-i-find-the-points-at-which-two-circles-intersect

            XA = 0.5*(xC[i] + xC[j]) + (r[i]**2 - r[j]**2)/(2*R[i]**2)*(xC[j] -xC[i]) - 0.5*math.sqrt(abs(2*(r[i]**2 + r[j]**2)/(R[i]**2) - ((r[i]**2 - r[j]**2)**2)/(R[i]**4) - 1))*(yC[j] - yC[i])
            YA = 0.5*(yC[i] + yC[j]) + (r[i]**2 - r[j]**2)/(2*R[i]**2)*(yC[j] -yC[i]) + 0.5*math.sqrt(abs(2*(r[i]**2 + r[j]**2)/(R[i]**2) - ((r[i]**2 - r[j]**2)**2)/(R[i]**4) - 1))*(xC[j] - xC[i])
            XB = 0.5*(xC[i] + xC[j]) + (r[i]**2 - r[j]**2)/(2*R[i]**2)*(xC[j] -xC[i]) + 0.5*math.sqrt(abs(2*(r[i]**2 + r[j]**2)/(R[i]**2) - ((r[i]**2 - r[j]**2)**2)/(R[i]**4) - 1))*(yC[j] - yC[i])
            YB = 0.5*(yC[i] + yC[j]) + (r[i]**2 - r[j]**2)/(2*R[i]**2)*(yC[j] -yC[i]) - 0.5*math.sqrt(abs(2*(r[i]**2 + r[j]**2)/(R[i]**2) - ((r[i]**2 - r[j]**2)**2)/(R[i]**4) - 1))*(xC[j] - xC[i])

            distanceA = math.sqrt((XA-xC[k])**2+(YA-yC[k])**2)
            distanceB = math.sqrt((XB-xC[k])**2+(YB-yC[k])**2)
            tol = 0.1

            if r[k] - tol < distanceA < r[k] + tol:
                xGuess[0, mm] = XA
                yGuess[0, mm] = YA
            else:
                xGuess[0, mm] = XB
                yGuess[0, mm] = YB

        youbotPos[0] = (xGuess[0, 0] + xGuess[0, 1] + xGuess[0, 2])/3
        youbotPos[1] = (yGuess[0, 0] + yGuess[0, 1] + yGuess[0, 2])/3

        print('youbotPos without filter', youbotPos)

        # tmp = np.zeros((1, 2))
        # tmp[0][0] = youbotPos[0]
        # tmp[0][1] = youbotPos[1]
        # x_filter = tmp
        # youbotPos = example_filter(x_filter, distance_beacon)
        # youbotPos = np.hstack((youbotPos, youbotPosz))
        # print('youbotPos with filter', youbotPos)

        # Milestone1.A
        res, RealyoubotPos = vrep.simxGetObjectPosition(clientID, h['ref'], -1, vrep.simx_opmode_buffer)
        vrchk(vrep, res, True)  # Check the return value from the previous V-REP call (res) and exit in case of error.
        print('RealyoubotPos', RealyoubotPos)

        res, youbotEuler = vrep.simxGetObjectOrientation(clientID, h['ref'], -1, vrep.simx_opmode_buffer)
        vrchk(vrep, res, True)

        # Get the position of the robot in matrix form
        xRobot = round((youbotPos[0] + 7.5)/resolution)
        yRobot = round((youbotPos[1] + 7.5)/resolution)

        # --- Drawing the map initialization ---
        #
        # Get data from the hokuyo - return empty if data is not captured
        rotangle = youbotEuler[2] - math.pi/2
        hokuyoPos = np.array([[youbotPos[0]], [youbotPos[1]]]) + np.array([[np.cos(rotangle)], [np.sin(rotangle)]]) * 0.23
        # 0.23 is the distance along Y between youbot_Center and fastHokuyo_ref

        # Determine the position of the Hokuyo with global coordinates (world reference frame).
        from trans_rot_matrix import trans_rot_matrix
        trf = trans_rot_matrix(youbotEuler, youbotPos)  # check the file trans_rot_matrix for explanation

        scanned_points, contacts = youbot_hokuyo(vrep, h, vrep.simx_opmode_buffer, trf)
        vrchk(vrep, res)
        # scanned_points is a 6xN matrix, where rows 1 and 4 represents coordinate x,
        # rows 2 and 5 coord y and rows 3 and 6 coord z. So to obtain all the x coords
        # we need to concatenate row 1 and 4. This is done in the following lines of code

        # Free space points
        from matplotlib.path import Path
        import matplotlib.pyplot as plt
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
            if int(xObstacle[i]) >= xLength or int(yObstacle[i]) >= yLength:
                break

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
                if j >= xLength or k >= yLength:
                    break

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
                if j >= xLength or k >= yLength:
                    break

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

            # Plot of the total map
            plt.close()
            plt.matshow(statesMap)
            plt.colorbar()
            plt.show()

            # Plot the loop time as a histogram

            # print(timing)
            # print(max(timing))
            # print(min(timing))

            n, bins, patches = plt.hist(x=timing, bins='auto', color='#0504aa', alpha=0.7, rwidth=0.85)

            plt.grid(axis='y', alpha=0.75)
            plt.xlabel('Time [s]')
            plt.ylabel('Number of loops')
            maxTime = n.max()
            plt.ylim(ymax=np.ceil(maxTime/10) * 10 if maxTime % 10 else maxTime + 10)

            plt.show()

            # End the infinite loop
            p = False

        #
        # --- Astar path planning ---
        #

        elif fsm == 'astar':

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

            results = astar.run([xRobot, yRobot], [xTarget, yTarget])

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
            # youbotEuler(3) is the rotation around the vertical axis.
            if iPath >= len(path)-1:
                fsm = 'searchAlgo'
                print('Switching to state: ', fsm)

            a = path[iPath, 0] - youbotPos[0]
            b = youbotPos[1] - path[iPath, 1]
            rotationAngle = math.atan2(a, b)

            rotateRightVel = angdiff(youbotEuler[2], rotationAngle)

            if rotateRightVel > math.pi:
                rotateRightVel = rotateRightVel - math.pi*2

            if rotateRightVel < - math.pi:
                rotateRightVel = rotateRightVel + math.pi*2

            # Stop when the robot is at an angle close to rotationAngle
            if abs(angdiff(youbotEuler[2], rotationAngle)) < 0.1:
                rotateRightVel = 0
                fsm = 'forward'
                print('Switching to state: ', fsm)

        #
        # --- Forward ---
        #
        elif fsm == 'forward':

            if iPath >= len(path)-1:
                fsm = 'searchAlgo'
                print('Switching to state: ', fsm)
            else:
                # Set a costant velocity
                forwBackVel = -1.0

                tmp = np.zeros((1, 2))
                tmp[0][0] = youbotPos[0]
                tmp[0][1] = youbotPos[1]
                x_filter = tmp
                youbotPos = example_filter(x_filter, distance_beacon, youbotEuler[2])
                youbotPos = np.hstack((youbotPos, youbotPosz))
                print('youbotPos with filter', youbotPos)

                # Stop when the robot is close to the next element of the path.
                # The tolerance has been determined by experiments
                a = path[iPath, 0] - youbotPos[0]
                b = youbotPos[1] - path[iPath, 1]
                distance = math.sqrt(a**2 + b**2)  # distance between robot and goal
                if abs(distance) < .5:
                    forwBackVel = 0  # Stop the robot.
                    iPath = iPath + 1
                    fsm = 'rotate'
                    print('Switching to state: ', fsm)

        #
        # --- Finished ---
        #
        elif fsm == 'finished':
            print('Finish')
            time.sleep(3)
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
