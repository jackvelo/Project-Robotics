# In this section the robot is rotating the camera towards the center of the table
elif fsm == 'rotateToCenter':
    k = discoverTableCounter

    # get camera position
    [res, rgbdPos] = vrep.simxGetObjectPosition(clientID, h['ref'], -1,vrep.simx_opmode_oneshot_wait)
    vrchk(vrep, res, true)

    aa = rgbdPos[0] - tablesCentersReal[k, 0]
    bb = tablesCentersReal[j, 1] - rgbdPos[1]

    if k < 3:
        aa = rgbdPos[0] - tablesCentersReal[k, 0]
        bb = tablesCentersReal[j, 1] - rgbdPos[1]
        beta = math.atan2(aa,bb) - math.pi/2
        if bb > 0:
            beta = beta + pi
    else:
        aa = rgbdPos[0] - tablesCentersReal[tabID, 0]
        bb = tablesCentersReal[tabID, 1] - rgbdPos[1]
        beta = math.atan2(aa,bb) - math.pi/2
        if bb > 0:
            beta = beta + pi

    # Set the desired camera orientation
    vrep.simxSetObjectOrientation(clientID, h['ref'], -1,[0 0 angle], vrep.simx_opmode_oneshot)

    # Checking whether we are in the target finding phase or table modelling one
    if discoverTableCounter < 3:
        fsm = 'findTarget'
    else
        fsm = 'modelTable'

# In this section a picture of the table is taken and the target is determined
elif fsm == 'findTarget':

    k = discoverTableCounter

    # Get scan angle of pi/4 for the depth sensor
    res = vrep.simxSetFloatSignal(clientID, 'rgbd_sensor_scan_angle', pi / 4, vrep.simx_opmode_oneshot_wait)
    vrchk(vrep, res)

    # Ask the sensor to turn itself on, take A SINGLE POINT CLOUD, and turn itself off again.
    res = vrep.simxSetIntegerSignal(clientID, 'handle_xyz_sensor', 1, vrep.simx_opmode_oneshot_wait)
    vrchk(vrep, res)

    # Get the point cloud from the depth sensor
    pointCloud = youbot_xyz_sensor(vrep,  h['ref'] , vrep.simx_opmode_oneshot_wait)
    # Take only the points until a distance of 1.2

    # Find highest point for this table
    maxi = - math.inf
    for point in range(len(pointCloud)):
        if pointCloud[1, point] > maxi:
            maxi = pointCloud[1 , point]









