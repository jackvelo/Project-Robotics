import sim as vrep

import numpy as np
import math
from vrchk import vrchk
from youbot_drive import youbot_drive
from utils_sim import angdiff

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


def stopWheels(vrep, id, h):
    # set target velocity of the joint to 0
    res = vrep.simxSetJointTargetVelocity(id, h['wheelJoints'][0], 0, vrep.simx_opmode_oneshot)
    vrchk(vrep, res)
    # set target velocity of the joint to 0
    res = vrep.simxSetJointTargetVelocity(id, h['wheelJoints'][1], 0, vrep.simx_opmode_oneshot)
    vrchk(vrep, res)
    # set target velocity of the joint to 0
    res = vrep.simxSetJointTargetVelocity(id, h['wheelJoints'][2], 0, vrep.simx_opmode_oneshot)
    vrchk(vrep, res)
    # set target velocity of the joint to 0
    res = vrep.simxSetJointTargetVelocity(id, h['wheelJoints'][3], 0, vrep.simx_opmode_oneshot)
    vrchk(vrep, res)


def rotate(angleGoal, h, clientID, vrep):

    rotation = True
    rotationAngle = angleGoal

    # The orientation of youbot is obtained:
    [res, youbotEuler] = vrep.simxGetObjectOrientation(clientID, h['ref'], -1, vrep.simx_opmode_buffer)
    vrchk(vrep, res, True)

    rotateRightVel = angdiff(youbotEuler[2], rotationAngle)

    if rotateRightVel > math.pi:
        rotateRightVel = rotateRightVel - math.pi*2

    if rotateRightVel < - math.pi:
        rotateRightVel = rotateRightVel + math.pi*2

    while rotation:
        [res, youbotEuler] = vrep.simxGetObjectOrientation(clientID, h['ref'], -1, vrep.simx_opmode_buffer)
        vrchk(vrep, res, True)

        # Stop when the robot is at an angle close to rotationAngle
        if abs(angdiff(youbotEuler[2], rotationAngle)) < 0.05:
            rotateRightVel = 0
            rotation = False

        # Update wheel velocities.
        forwBackVel = 0
        rightVel = 0
        h = youbot_drive(vrep, h, forwBackVel, rightVel, rotateRightVel)
        vrep.simxSynchronousTrigger(clientID)
        vrep.simxGetPingTime(clientID)

    h = youbot_drive(vrep, h, forwBackVel, rightVel, rotateRightVel)

    for i in range(20):
        vrep.simxSynchronousTrigger(clientID)
        vrep.simxGetPingTime(clientID)


def rotate2(rotateRightVel, angleGoal, h, clientID, vrep):

    # The orientation of youbot is obtained:
    [res, youbotEuler] = vrep.simxGetObjectOrientation(clientID, h['ref'], -1, vrep.simx_opmode_buffer)
    vrchk(vrep, res, True)

    angleBot = youbotEuler[2]

    prevOrientation = 0
    rotation = True
    fsm = 'nothing'

    # It is necessary to consider each quadrant singularly
    # First quadrant

    if math.pi/2 <= angleGoal <= math.pi:

        if 0 < angleBot < angleGoal or angleGoal - math.pi < angleBot < 0:
            # Turn left
            fsm = 'left'

        if math.pi > angleBot > angleGoal or -math.pi < angleBot < angleGoal - math.pi:
            # Turn right
            fsm = 'right'

    # Second quadrant

    if -math.pi <= angleGoal <= -math.pi/2:

        if -math.pi < angleBot < angleGoal or math.pi > angleBot > angleGoal + math.pi:
            # Turn left
            fsm = 'left'

        if 0 > angleBot > angleGoal or 0 < angleBot < angleGoal + math.pi:
            # Turn right
            fsm = 'right'

    # Third quadrant

    if -math.pi/2 <= angleGoal <= 0:

        if -math.pi < angleBot < angleGoal or math.pi > angleBot > angleGoal + math.pi:
            # Turn left
            fsm = 'left'

        if 0 > angleBot > angleGoal or 0 < angleBot < angleGoal + math.pi:
            # Turn right
            fsm = 'right'

    # Fourth quadrant

    if 0 <= angleGoal <= math.pi/2:

        if 0 < angleBot < angleGoal or 0 > angleBot > angleGoal - math.pi:
            # Turn left
            fsm = 'left'

        if math.pi > angleBot > angleGoal or -math.pi < angleBot < angleGoal - math.pi:
            # Turn right
            fsm = 'right'

    # Turn left

    if fsm == 'left':

        while rotation:

            vrep.simxSynchronousTrigger(clientID)
            vrep.simxGetPingTime(clientID)

            # Rotation until reach angleGoal:
            # set target velocity of the joint to 0:
            vrep.simxSetJointTargetVelocity(clientID, h['wheelJoints'][0], rotateRightVel, vrep.simx_opmode_oneshot)
            vrchk(vrep, res)
            # set target velocity of the joint to 0
            vrep.simxSetJointTargetVelocity(clientID, h['wheelJoints'][1], rotateRightVel, vrep.simx_opmode_oneshot)
            vrchk(vrep, res)
            # set target velocity of the joint to 0
            vrep.simxSetJointTargetVelocity(clientID, h['wheelJoints'][2], -rotateRightVel, vrep.simx_opmode_oneshot)
            vrchk(vrep, res)
            # set target velocity of the joint to 0
            vrep.simxSetJointTargetVelocity(clientID, h['wheelJoints'][3], -rotateRightVel, vrep.simx_opmode_oneshot)
            vrchk(vrep, res)

            # When the rotation is done move to the next state
            if abs(angdiff(angleGoal, youbotEuler[2])) < .05 and abs(angdiff(prevOrientation, youbotEuler[2])) < .05:

                # Call the function to stop the rotation
                # set target velocity of the joint to 0
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

                # Stop the loop
                rotation = False

            prevOrientation = youbotEuler[2]
            [res, youbotEuler] = vrep.simxGetObjectOrientation(clientID, h['ref'], -1, vrep.simx_opmode_buffer)
            vrchk(vrep, res, True)

    # Turn right

    if fsm == 'right':

        while rotation:

            vrep.simxSynchronousTrigger(clientID)
            vrep.simxGetPingTime(clientID)

            # Rotation until reach angleGoal
            # set target velocity of the joint to 0
            vrep.simxSetJointTargetVelocity(clientID, h['wheelJoints'][0], -rotateRightVel, vrep.simx_opmode_oneshot)
            vrchk(vrep, res)
            # set target velocity of the joint to 0
            vrep.simxSetJointTargetVelocity(clientID, h['wheelJoints'][1], -rotateRightVel, vrep.simx_opmode_oneshot)
            vrchk(vrep, res)
            # set target velocity of the joint to 0
            vrep.simxSetJointTargetVelocity(clientID, h['wheelJoints'][2], rotateRightVel, vrep.simx_opmode_oneshot)
            vrchk(vrep, res)
            # set target velocity of the joint to 0
            vrep.simxSetJointTargetVelocity(clientID, h['wheelJoints'][3], rotateRightVel, vrep.simx_opmode_oneshot)
            vrchk(vrep, res)

            # When the rotation is done move to the next state.
            if abs(angdiff(angleGoal, youbotEuler[2])) < .05 and abs(angdiff(prevOrientation, youbotEuler[2])) < .05:

                # Call the function to stop the rotation
                # set target velocity of the joint to 0
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

                # Stop the loop
                rotation = False

            prevOrientation = youbotEuler[2]
            [res, youbotEuler] = vrep.simxGetObjectOrientation(clientID, h['ref'], -1, vrep.simx_opmode_buffer)
            vrchk(vrep, res, True)
