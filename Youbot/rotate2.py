import numpy as np
import math
from Functions import Rotate, stopWheels
from vrchk import vrchk


def rotate2(rotateRightVel, angleGoal, h, clientID, vrep):

    # The orientation of youbot is obtained:
    [res, youbotEuler] = vrep.simxGetObjectOrientation(clientID, h['ref'], -1, vrep.simx_opmode_buffer)
    vrchk(vrep, res, true)

    angleBot = youbotEuler[2]

    prevOrientation = 0
    rotation = True
    fsm = 'nothing'

    # It is necessary to consider each quadrant singularly
    # First quadrant

    if math.pi/2 <= angleGoal <= math.pi:

        if 0 < angleBot < angleGoal and angleGoal- math.pi < angleBot < 0:
            # Turn left
            fsm = 'left'

        if  math.pi > angleBot > angleGoal and -math.pi < angleBot < angleGoal - math.pi:
            # Turn left
            fsm = 'right'

    # Second quadrant

    if -math.pi <= angleGoal <= -math.pi/2:

        if -math.pi < angleBot < angleGoal and pi > angleBot > angleGoal + math.pi:
            # Turn left
            fsm = 'left'

        if 0 > angleBot > angleGoal and 0 < angleBot < angleGoal + math.pi:
            # Turn right
            fsm = 'right'

    # Third quadrant

    if -math.pi/2 <= angleGoal <= 0:

        if -math.pi < angleBot < angleGoal and math.pi > angleBot > angleGoal + math.pi:
            # Turn left
            fsm = 'left'

        if 0 > angleBot > angleGoal and 0 < angleBot < angleGoal + math.pi:
            # Turn right
            fsm = 'right'

    # Fourth quadrant

    if 0 <= angleGoal <= math.pi/2:

        if 0 < angleBot < angleGoal and 0 > angleBot > angleGoal - math.pi:
            # Turn left
            fsm = 'left'

        if math.pi > angleBot > angleGoal and -math.pi < angleBot < angleGoal - math.pi:
            # Turn right
            fsm = 'right'

    # Turn left

    if fsm == 'left':

        while rotation:

            # Rotation until reach angleGoal:
            # set target velocity of the joint to 0:
            vrep.simxSetJointTargetVelocity(clientID, h.wheelJoints[0], rotateRightVel, vrep.simx_opmode_oneshot)
            vrchk(vrep, res)
            # set target velocity of the joint to 0
            vrep.simxSetJointTargetVelocity(clientID, h.wheelJoints[1], rotateRightVel, vrep.simx_opmode_oneshot)
            vrchk(vrep, res)
            # set target velocity of the joint to 0
            vrep.simxSetJointTargetVelocity(clientID, h.wheelJoints[2], -rotateRightVel, vrep.simx_opmode_oneshot)
            vrchk(vrep, res)
            # set target velocity of the joint to 0
            vrep.simxSetJointTargetVelocity(clientID, h.wheelJoints[3], -rotateRightVel, vrep.simx_opmode_oneshot)
            vrchk(vrep, res)

            # When the rotation is done move to the next state
            if abs(angdiff(angleGoal, youbotEuler[2])) < .01 and abs(angdiff(prevOrientation, youbotEuler[2])) < .01:

                   # stop the rotation
                  stopWheels(h, clientID, vrep )

                  # Stop the loop
                  rotation = False

            prevOrientation = youbotEuler[2]
            [res, youbotEuler] = vrep.simxGetObjectOrientation(clientID, h.ref, -1, vrep.simx_opmode_buffer)
            vrchk(vrep, res, true)

    # Turn right

    if fsm == 'right':

        while rotation:

            # Rotation until reach angleGoal
            # set target velocity of the joint to 0
            vrep.simxSetJointTargetVelocity(clientID, h.wheelJoints[0], -rotateRightVel, vrep.simx_opmode_oneshot)
            vrchk(vrep, res)
            # set target velocity of the joint to 0
            vrep.simxSetJointTargetVelocity(clientID, h.wheelJoints[1], -rotateRightVel, vrep.simx_opmode_oneshot)
            vrchk(vrep, res)
            # set target velocity of the joint to 0
            vrep.simxSetJointTargetVelocity(clientID, h.wheelJoints[2], rotateRightVel, vrep.simx_opmode_oneshot)
            vrchk(vrep, res)
            # set target velocity of the joint to 0
            vrep.simxSetJointTargetVelocity(clientID, h.wheelJoints[3], rotateRightVel, vrep.simx_opmode_oneshot)
            vrchk(vrep, res)

            # When the rotation is done move to the next state.
            if abs(angdiff(angleGoal, youbotEuler[2])) < .01 and abs(angdiff(prevOrientation, youbotEuler[2])) < .01:

                  # Call the function to stop the rotation
                  stopWheels(h, clientID, vrep)

                  # Stop the loop
                  rotation = False

            prevOrientation = youbotEuler[2]
            [res, youbotEuler] = vrep.simxGetObjectOrientation(clientID, h['ref'], -1, vrep.simx_opmode_buffer)
            vrchk(vrep, res, true)
