import numpy as np
import math
from vrchk import vrchk
from youbot_drive import youbot_drive
from utils_sim import angdiff


def stopWheels(vrep, id, h):
    res = vrep.simxSetJointTargetVelocity(id, h['wheelJoints'][0], 0, vrep.simx_opmode_oneshot)  # set target velocity of the joint to 0
    vrchk(vrep, res)
    res = vrep.simxSetJointTargetVelocity(id, h['wheelJoints'][1], 0, vrep.simx_opmode_oneshot)  # set target velocity of the joint to 0
    vrchk(vrep, res)
    res = vrep.simxSetJointTargetVelocity(id, h['wheelJoints'][2], 0, vrep.simx_opmode_oneshot)  # set target velocity of the joint to 0
    vrchk(vrep, res)
    res = vrep.simxSetJointTargetVelocity(id, h['wheelJoints'][3], 0, vrep.simx_opmode_oneshot)  # set target velocity of the joint to 0
    vrchk(vrep, res)


def Rotate(vrep, id, h, rotateRightVel, xgoal, ygoal, xbot, ybot):
    # opposed side and adjacent side of the triangle
    o = abs(ybot - ygoal)
    a = abs(xbot - xgoal)
    # angle phi in the triangle rectangle
    phi = math.atan(o/a)

    # Special case of the fourth quadrant
    if xbot - xgoal < 0 and ybot - ygoal > 0:
        a = abs(ybot - ygoal)
        o = abs(xbot - xgoal)
        phi = math.atan(o/a)

    # Get the orientation of the youbot
    [res, youbotEuler] = vrep.simxGetObjectOrientation(id, h['ref'], -1, vrep.simx_opmode_buffer)
    vrchk(vrep, res, True)

    anglebot = youbotEuler[2]

    prevOrientation = 0
    rotation = True
    fsm = 'nothing'

    # First quadrant
    if xbot - xgoal <= 0 and ybot - ygoal <= 0:

        # anglegoal is the angle the robot should take in the world reference frame
        anglegoal = math.pi/2 + phi

        if (0 <= anglebot <= anglegoal) or (0 >= anglebot >= anglegoal-math.pi):
            fsm = 'left'
        else:
            fsm = 'right'

    # Second quadrant
    elif xbot - xgoal >= 0 and ybot - ygoal <= 0:

        anglegoal = -(math.pi/2 + phi)

        if (-math.pi <= anglebot <= anglegoal) or (math.pi >= anglebot >= anglegoal+math.pi):
            fsm = 'left'
        else:
            fsm = 'right'

    # Third quadrant
    elif xbot - xgoal >= 0 and ybot - ygoal >= 0:

        anglegoal = -(math.pi/2 - phi)

        if (-math.pi <= anglebot <= anglegoal) or (math.pi >= anglebot >= anglegoal+math.pi):
            fsm = 'left'
        else:
            fsm = 'right'

    # Fourth quadrant
    elif xbot - xgoal <= 0 and ybot - ygoal >= 0:

        anglegoal = phi

        if (0 <= anglebot <= anglegoal) or (0 >= anglebot >= anglegoal-math.pi):
            fsm = 'left'
        else:
            fsm = 'right'

    # Turn left
    if fsm == 'left':

        while rotation:

            # Rotation until reach anglegoal
            vrep.simxSetJointTargetVelocity(id, h['wheelJoints'][0], rotateRightVel, vrep.simx_opmode_oneshot)  # set target velocity of the joint to 0
            vrchk(vrep, res)
            vrep.simxSetJointTargetVelocity(id, h['wheelJoints'][1], rotateRightVel, vrep.simx_opmode_oneshot)  # set target velocity of the joint to 0
            vrchk(vrep, res)
            vrep.simxSetJointTargetVelocity(id, h['wheelJoints'][2], -rotateRightVel, vrep.simx_opmode_oneshot)  # set target velocity of the joint to 0
            vrchk(vrep, res)
            vrep.simxSetJointTargetVelocity(id, h['wheelJoints'][3], -rotateRightVel, vrep.simx_opmode_oneshot)  # set target velocity of the joint to 0
            vrchk(vrep, res)

            # When the rotation is done (with a sufficiently high precision), move on to the next state.
            if (abs(angdiff(anglegoal, youbotEuler[2])) < .01) and (abs(angdiff(prevOrientation, youbotEuler[2])) < .01 ):

                # Call the function to stop the rotation
                stopWheels(h, id, vrep)

                # Stop the loop
                rotation = False

            prevOrientation = youbotEuler[2]
            [res, youbotEuler] = vrep.simxGetObjectOrientation(id, h['ref'], -1, vrep.simx_opmode_buffer)
            vrchk(vrep, res, true)

    # Turn right

    if fsm == 'right':

        while rotation:

            # Rotation until reach anglegoal
            vrep.simxSetJointTargetVelocity(id, h['wheelJoints'][0], -rotateRightVel, vrep.simx_opmode_oneshot)  # set target velocity of the joint to 0
            vrchk(vrep, res)
            vrep.simxSetJointTargetVelocity(id, h['wheelJoints'][1], -rotateRightVel, vrep.simx_opmode_oneshot)  # set target velocity of the joint to 0
            vrchk(vrep, res)
            vrep.simxSetJointTargetVelocity(id, h['wheelJoints'][2], rotateRightVel, vrep.simx_opmode_oneshot)  # set target velocity of the joint to 0
            vrchk(vrep, res)
            vrep.simxSetJointTargetVelocity(id, h['wheelJoints'][3], rotateRightVel, vrep.simx_opmode_oneshot)  # set target velocity of the joint to 0
            vrchk(vrep, res)



            # Update wheel velocities.
            h = youbot_drive(vrep, h, 0, 0, rotateRightVel)

            # Send a Trigger to the simulator: this will run a time step for the physic engine
            # because of the synchronous mode.
            vrep.simxSynchronousTrigger(id)
            vrep.simxGetPingTime(id)


            # When the rotation is done (with a sufficiently high precision), move on to the next state.
            print('primaprima')
            if (abs(angdiff(anglegoal, youbotEuler[2])) < .01) and (abs(angdiff(prevOrientation, youbotEuler[2])) < .01):

                # Call the function to stop the rotation
                print('prima')
                stopWheels(h, id, vrep)
                print('dopo')

                # Stop the loop
                rotation = False
            prevOrientation = youbotEuler[2]
            [res, youbotEuler] = vrep.simxGetObjectOrientation(id, h['ref'], -1, vrep.simx_opmode_buffer)
            vrchk(vrep, res, True)
