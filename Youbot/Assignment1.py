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

from cleanup_vrep import cleanup_vrep
from vrchk import vrchk
from youbot_init import youbot_init
from youbot_drive import youbot_drive
from youbot_hokuyo_init import youbot_hokuyo_init
from youbot_hokuyo import youbot_hokuyo
from youbot_xyz_sensor import youbot_xyz_sensor
from beacon import beacon_init, youbot_beacon
from utils_sim import angdiff

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
#cleanup_vrep(vrep, id)

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
beacons_handle = beacon_init(vrep, clientID)

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

if True:
    # representation of the 2D map explored by the youBot
    resolution = 0.25
    dim = 7.5   # house's dimension
    # x, y will be used to display the area the robot can see, by
    # selecting the points within this mesh that are within the visibility range.
    x, y = np.meshgrid(np.arange(-dim, dim + resolution, resolution), np.arange(-dim, dim + resolution, resolution))
    x, y = x.flatten(), y.flatten()
    #these are the axis
    xAxis = np.arange(-dim, dim + resolution, resolution)
    yAxis = np.arange(-dim, dim + resolution, resolution)

# dimension of the x and y axes
n = len(xAxis)

# building up the state map assuming all the states as unknown (= 1)
statesMap = np.ones((n,n), dtype=int)

# obtaining the state map size
sizeMap = statesMap.shape
xLength = sizeMap[0]
yLength = sizeMap[1]

# np.set_printoptions(threshold=sys.maxsize)
# print(xLength)

# Get the initial position
res, youbotPos = vrep.simxGetObjectPosition(clientID, h['ref'], -1, vrep.simx_opmode_buffer)
# Set the speed of the wheels to 0.
h = youbot_drive(vrep, h, forwBackVel, rightVel, rotateRightVel)

# Initialise the state machine.
fsm = 'search'
print('Switching to state: ', fsm)

# Send a Trigger to the simulator: this will run a time step for the physic engine
# because of the synchronous mode. Run several iterations to stabilize the simulation
for i in range(int(1./timestep)):
    vrep.simxSynchronousTrigger(clientID)
    vrep.simxGetPingTime(clientID)

##
## PROVE
##
# print(youbotPos[0])
# print(youbotPos[1])
# print(youbotPos[2])
# A = np.array([[youbotPos[0]], [youbotPos[1]]]) * 2
# print(A)
##
##
##

# Start the demo.
while True:
     try:  # used when a code can give error --> "try" and "except" at the end give the error explanation
         # Check the connection with the simulator
         if vrep.simxGetConnectionId(clientID) == -1:
             sys.exit('Lost connection to remote API.')

         # Get the position and the orientation of the robot.
         res, youbotPos = vrep.simxGetObjectPosition(clientID, h['ref'], -1, vrep.simx_opmode_buffer)
         vrchk(vrep, res, True) # Check the return value from the previous V-REP call (res) and exit in case of error.
         res, youbotEuler = vrep.simxGetObjectOrientation(clientID, h['ref'], -1, vrep.simx_opmode_buffer)
         vrchk(vrep, res, True)

         # Get the position of the robot in matrix form
         xRobot = round((youbotPos[0] + 7.5)/resolution)
         yRobot = round((youbotPos[1] + 7.5)/resolution)

         ## Drawing the map initialization ##
         #
         # Get data from the hokuyo - return empty if data is not captured
         rotangle = youbotEuler[2] - math.pi/2
         hokuyoPos = np.array([[youbotPos[0]], [youbotPos[1]]]) + np.array([[np.cos(rotangle)], [np.sin(rotangle)]]) * 0.23
         #0.23 is the distance along Y between youbot_Center and fastHokuyo_ref
         print(hokuyoPos)

         # Determine the position of the Hokuyo with global coordinates (world reference frame).
         from trans_rot_matrix import trans_rot_matrix
         trf = trans_rot_matrix(youbotEuler, youbotPos) # check the file trans_rot_matrix for explanation

         scanned_points, contacts = youbot_hokuyo(vrep, h, vrep.simx_opmode_buffer, trf)
         vrchk(vrep, res)
         # scanned_points is a 6xN matrix, where rows 1 and 4 represents coordinate x,
         # rows 2 and 5 coord y and rows 3 and 6 coord z. So to obtain all the x coords
         # we need to concatenate row 1 and 4. This is done on the following lines of code

         ## Free space points
         from matplotlib.path import Path
         import matplotlib.pyplot as plt
         points = np.vstack((x,y)).T # x, y defined on line 108

         # reorder scanned_points like x = [x1, x2... xn] and y = [y1, y2 ... yn]
         row1 = scanned_points[0,:]
         row2 = scanned_points[1,:]
         row4 = scanned_points[3,:]
         row5 = scanned_points[4,:]
         arr1 = np.squeeze(np.asarray(row1)) # squeeze change the type from np.matrix to np.array, needed to concatenate
         arr2 = np.squeeze(np.asarray(row2))
         arr4 = np.squeeze(np.asarray(row4))
         arr5 = np.squeeze(np.asarray(row5))
         x_scanned = np.hstack((arr1, arr4)) #concatenate horizontally
         y_scanned = np.hstack((arr2, arr5))
         x_polygon = np.hstack((youbotPos[0], x_scanned)) #concatenate horizontally
         y_polygon = np.hstack((youbotPos[1], y_scanned))
         polygon_vertex = np.vstack((x_polygon, y_polygon)).T

         # Make a polygon with the scanned points (boundary points) and check what points of the statesMap
         # are inside this polygon. The points inside the polygon will be free space
         p = Path(polygon_vertex) # make a polygon
         grid = p.contains_points(points) # check what points fall inside (grid represents the index of the points)

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
         contacts_total = np.hstack((contacts[0,:], contacts[1,:]))
         xObstacle = x_scanned[contacts_total]
         yObstacle = y_scanned[contacts_total]

         xObstacle = (xObstacle + 7.5)/resolution
         yObstacle = (yObstacle + 7.5)/resolution
         xObstacle = np.round(xObstacle)
         yObstacle = np.round(yObstacle)


         for i in range(len(xObstacle)):
             statesMap[int(xObstacle[i]), int(yObstacle[i])] = 2

             #
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

             if 0 <= xObstacle[i]< len(xAxis) \
             and 0 <= yObstacle[i] - 1 < len(yAxis):
                 if statesMap[int(xObstacle[i]), int(yObstacle[i] - 1)] == 0:
                     statesMap[int(xObstacle[i]), int(yObstacle[i] - 1)] = 3


         plt.matshow(statesMap)
         plt.colorbar()
         plt.show()

         ## Occupancy grid

         occupancyGrid = np.ones((n,n), dtype=int)

         for j in range(len(xAxis)):
             for k in range(len(yAxis)):
                 if statesMap[j,k] == 2 or statesMap[j,k] == 3:
                     occupancyGrid[j,k] = 1
                 else:
                     occupancyGrid[j,k] = 0

         plt.matshow(occupancyGrid)
         plt.colorbar()
         plt.show()

         ## Search algorithm
         # search for a goal point to visit

         Apply the state machine.
         if fsm == 'searchAlgo':

         # initialize variables
         xTarget = 0
         yTarget = 0
         foundTarget = False

         # first, we search a target near the horizon
         x_scanned = (x_scanned + 7.5)/resolution
         y_scanned = (y_scanned + 7.5)/resolution
         x_scanned = np.round(x_scanned)
         y_scanned = np.round(y_scanned)

         for ii in range(len(x_scanned)):

             if statesMap[x_scanned[ii],y_scanned[ii]] == 0:

                 if x_scanned[ii] + 1 <= n and statesMap[x_scanned[ii] + 1,y_scanned[ii]] == 1:

                     xUnknown = x_scanned[ii] + 1
                     yUnknown = y_scanned[ii]
                     foundTarget = True

                 elif x_scanned[ii] - 1 <= n and statesMap[x_scanned[ii] - 1,y_scanned[ii]] == 1:

                     xUnknown = x_scanned[ii] - 1
                     yUnknown = y_scanned[ii]
                     foundTarget = True

                 elif y_scanned[ii] + 1 <= n and statesMap[x_scanned[ii],y_scanned[ii] + 1] == 1:

                     xUnknown = x_scanned[ii]
                     yUnknown = y_scanned[ii] + 1
                     foundTarget = True

                 elif y_scanned[ii] - 1 <= n and statesMap[x_scanned[ii],y_scanned[ii] - 1] == 1:

                     xUnknown = x_scanned[ii]
                     yUnknown = y_scanned[ii] - 1
                     foundTarget = True


             if foundTarget:

                 xT = x_scanned[ii]
                 yT = y_scanned[ii]



# # First state of state machine
# fsm = 'forward'
# print('Switching to state: ', fsm)
#
# # Get the initial position
# res, youbotPos = vrep.simxGetObjectPosition(clientID, h['ref'], -1, vrep.simx_opmode_buffer)
# # Set the speed of the wheels to 0.
# h = youbot_drive(vrep, h, forwBackVel, rightVel, rotateRightVel)
#
# # Send a Trigger to the simulator: this will run a time step for the physic engine
# # because of the synchronous mode. Run several iterations to stabilize the simulation
# for i in range(int(1./timestep)):
#     vrep.simxSynchronousTrigger(clientID)
#     vrep.simxGetPingTime(clientID)
#
# # Start the demo.
# while True:
#     try:
#         # Check the connection with the simulator
#         if vrep.simxGetConnectionId(clientID) == -1:
#             sys.exit('Lost connection to remote API.')
#
#         # Get the position and the orientation of the robot.
#         res, youbotPos = vrep.simxGetObjectPosition(clientID, h['ref'], -1, vrep.simx_opmode_buffer)
#         vrchk(vrep, res, True) # Check the return value from the previous V-REP call (res) and exit in case of error.
#         res, youbotEuler = vrep.simxGetObjectOrientation(clientID, h['ref'], -1, vrep.simx_opmode_buffer)
#         vrchk(vrep, res, True)
#
         # # Get the distance from the beacons
         # # Change the flag to True to constraint the range of the beacons
         # beacon_dist = youbot_beacon(vrep, clientID, beacons_handle, h, flag=False)

         # # Get data from the hokuyo - return empty if data is not captured
         # scanned_points, contacts = youbot_hokuyo(vrep, h, vrep.simx_opmode_buffer)
         # vrchk(vrep, res)
#
#         # Apply the state machine.
#         if fsm == 'forward':
#
#             # Make the robot drive with a constant speed (very simple controller, likely to overshoot).
#             # The speed is - 1 m/s, the sign indicating the direction to follow. Please note that the robot has
#             # limitations and cannot reach an infinite speed.
#             forwBackVel = -1
#
#             # Stop when the robot is close to y = - 6.5. The tolerance has been determined by experiments: if it is too
#             # small, the condition will never be met (the robot position is updated every 50 ms); if it is too large,
#             # then the robot is not close enough to the position (which may be a problem if it has to pick an object,
#             # for example).
#             if abs(youbotPos[1] + 6.5) < .02:
#                 forwBackVel = 0  # Stop the robot.
#                 fsm = 'backward'
#                 print('Switching to state: ', fsm)
#
#
#         elif fsm == 'backward':
#             # A speed which is a function of the distance to the destination can also be used. This is useful to avoid
#             # overshooting: with this controller, the speed decreases when the robot approaches the goal.
#             # Here, the goal is to reach y = -4.5.
#             forwBackVel = - 2 * (youbotPos[1] + 4.5)
#             # distance to goal influences the maximum speed
#
#             # Stop when the robot is close to y = 4.5.
#             if abs(youbotPos[1] + 4.5) < .01:
#                 forwBackVel = 0  # Stop the robot.
#                 fsm = 'right'
#                 print('Switching to state: ', fsm)
#
#         elif fsm == 'right':
#             # Move sideways, again with a proportional controller (goal: x = - 4.5).
#             rightVel = - 2 * (youbotPos[0] + 4.5)
#
#             # Stop at x = - 4.5
#             if abs(youbotPos[0] + 4.5) < .01:
#                 rightVel = 0  # Stop the robot.
#                 fsm = 'rotateRight'
#                 print('Switching to state: ', fsm)
#
#         elif fsm == 'rotateRight':
#             # Rotate until the robot has an angle of -pi/2 (measured with respect to the world's reference frame).
#             # Again, use a proportional controller. In case of overshoot, the angle difference will change sign,
#             # and the robot will correctly find its way back (e.g.: the angular speed is positive, the robot overshoots,
#             # the anguler speed becomes negative).
#             # youbotEuler(3) is the rotation around the vertical axis.
#             rotateRightVel = angdiff(youbotEuler[2], (-np.pi/2))
#
#             # Stop when the robot is at an angle close to -pi/2.
#             if abs(angdiff(youbotEuler[2], (-np.pi/2))) < .002:
#                 rotateRightVel = 0
#                 fsm = 'rotateLeft'
#                 print('Switching to state: ', fsm)
#
#         elif fsm == 'rotateLeft':
#             # Rotate until the robot has an angle of -pi/2 (measured with respect to the world's reference frame).
#             # Again, use a proportional controller. In case of overshoot, the angle difference will change sign,
#             # and the robot will correctly find its way back (e.g.: the angular speed is positive, the robot overshoots,
#             # the anguler speed becomes negative).
#             # youbotEuler(3) is the rotation around the vertical axis.
#             rotateRightVel = angdiff(youbotEuler[2], (np.pi/2))
#
#             # Stop when the robot is at an angle close to -pi/2.
#             if abs(angdiff(youbotEuler[2], (np.pi/2))) < .002:
#                 rotateRightVel = 0
#                 fsm = 'finished'
#                 print('Switching to state: ', fsm)
#
#
#         elif fsm == 'finished':
#             print('Finish')
#             time.sleep(3)
#             break
#         else:
#             sys.exit('Unknown state ' + fsm)
#
         # Update wheel velocities.
         h = youbot_drive(vrep, h, forwBackVel, rightVel, rotateRightVel)

         # What happens if you do not update the velocities?
         # The simulator always considers the last speed you gave it,
         # until you set a new velocity.

         # Send a Trigger to the simulator: this will run a time step for the physic engine
         # because of the synchronous mode.
         vrep.simxSynchronousTrigger(clientID)
         vrep.simxGetPingTime(clientID)
     except KeyboardInterrupt:
         cleanup_vrep(vrep, clientID)
         sys.exit('Stop simulation')

cleanup_vrep(vrep, clientID)
print('Simulation has stopped')
