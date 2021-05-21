def stopWheels(vrep, h, id):
    res = vrep.simxSetJointTargetVelocity(id, h.wheelJoints(1),0, vrep.simx_opmode_oneshot)  # set target velocity of the joint to 0
    vrchk(vrep, res)
    res = vrep.simxSetJointTargetVelocity(id, h.wheelJoints(2),0, vrep.simx_opmode_oneshot)  # set target velocity of the joint to 0
    vrchk(vrep, res)
    res = vrep.simxSetJointTargetVelocity(id, h.wheelJoints(3),0, vrep.simx_opmode_oneshot)  # set target velocity of the joint to 0
    vrchk(vrep, res)
    res = vrep.simxSetJointTargetVelocity(id, h.wheelJoints(4),0, vrep.simx_opmode_oneshot)  # set target velocity of the joint to 0
    vrchk(vrep, res)

