        # Use the depth camera to take picture of the tables in order to fill in ptsTableX, ptsObjectsX and ptsTarget
        elif fsm == 'modelTable':
            # get rgb camera position
            [res, rgbdPos] = vrep.simxGetObjectPosition(clientID, h.rgbdCasing, -1,vrep.simx_opmode_oneshot_wait)
            vrchk(vrep, res, True)

            # get rgb camera angle
            [res, rgbdEuler] = vrep.simxGetObjectOrientation(clientID, h.rgbdCasing, -1,vrep.simx_opmode_oneshot_wait)
            vrchk(vrep, res, True)

            # First, use a large angle to have a global view
            res = vrep.simxSetFloatSignal(clientID, 'rgbd_sensor_scan_angle', pi/4, vrep.simx_opmode_oneshot_wait)
            vrchk(vrep, res)

            # take a depth picture
            res = vrep.simxSetIntegerSignal(clientID, 'handle_xyz_sensor', 1, vrep.simx_opmode_oneshot_wait)
            vrchk(vrep, res)

            # Store this picture in pts
            pts = youbot_xyz_sensor(vrep, h, vrep.simx_opmode_oneshot_wait)

            # Only keep points within 1.2 meter, to focus on the table
            pts = pts[0:3, pts[3, :] < 1.2]
            # Only keep points far enough to avoid catch robot points
            pts = pts[0:3, pts[3, :] > 0.25]

            # Invert 3rd and 1st line to get XYZ in the right order
            pts = [pts[2,:],pts[0,:],pts[1,:]]
            trf = trans_rot_matrix(rgbdPos, rgbdEuler)  # check the file trans_rot_matrix for explanation
            # Apply the transfer function to get pts in real coordinates
            ## pts = homtrans(trf, pts)
            pts = np.dot(trf, pts)

            # Table height is 185 mm
            # we only keep points with a height between 80 and 150 mm (keep
            # margin) to identify the table by removing parasite points
            ptsTable = pts[0:2,pts[2,:] < 0.15]
            ptsTableInter = ptsTable[0:2,ptsTable[2,:] > 0.08]
            ptsTable = [ptsTableInter[0,:], ptsTableInter[1,:]]

            # When not dealing with target table, focus also on objects
            if tabID != targetID:
                # To know object points, keep the one with high above 0.187m
                # (small margin with the 0.185m of table high)
                ptsObject = pts[0:2,pts[2,:] > 0.187]

                # Reduce angle view to focus more on objects
                res = vrep.simxSetFloatSignal(id, 'rgbd_sensor_scan_angle', pi/7, vrep.simx_opmode_oneshot_wait)
                vrchk(vrep, res)

                # take a depth picture
                res = vrep.simxSetIntegerSignal(id, 'handle_xyz_sensor', 1, vrep.simx_opmode_oneshot_wait)
                vrchk(vrep, res)
                # Store this picture in pts
                pts = youbot_xyz_sensor(vrep, h, vrep.simx_opmode_oneshot_wait)
                # Only keep points within 1.2 meter, to focus on the table
                pts = pts[0:2, pts[3, :] < 1.2]

                # Invert 3rd and 1st line to get XYZ in the right order
                pts = [pts[2, :], pts[0, :], pts[1, :]]
                # Apply the transfer function to get pts in real coordinates
                ## pts = homtrans(trf, pts)
                pts = np.dot(trf, pts)

                # To know object points, keep the one with high above 0.187m
                # (small margin with the 0.185m of table high)
                ptsObject_focus = pts[0:2,pts[2,:] > 0.187]

            # Check which table is currently analyzed
            if tabID == objectsTablesID[1]:
                # save points of table 1 in a matrix and remove the multiple points
                tempMatrix = np.round(250*ptsTable)/250
                ptsTable1 = np.unique[[ptsTable1, tempMatrix.transpose()], 'rows']
                tempMatrix1 = np.round(250*ptsObject)/250
                tempMatrix2 = np.round(250*ptsObject_focus)/250
                ptsObjects1 = unique[[ptsObjects1, tempMatrix1.transpose(), tempMatrix2.transpose()], 'rows']
                # Increment the neighbour counter to send the youbot to next table neighbour
                neighbour = neighbour + 1
                if neighbour > 5:
                    # Visit the second table if finished with first one
                    tabToModel = table2Neighbours
                    tabID = objectsTablesID(2)
                    neighbour = 1
                fsm = 'astar'

            elif tabID == objectsTablesID(2):
                # save points of table 2 in a matrix and remove the multiple points
                tempMatrix = np.round(250*ptsTable)/250
                ptsTable2 = np.unique[[ptsTable2, tempMatrix.transpose()], 'rows']
                tempMatrix1 = np.round(250*ptsObject)/250
                tempMatrix2 = np.round(250*ptsObject_focus)/250
                ptsObjects2 = unique[[ptsObjects2, tempMatrix1.transpose(), tempMatrix2.transpose()], 'rows']
                # Increment the neighbour counter to send the youbot to next table neighbour
                neighbour = neighbour + 1
                if neighbour > 5:
                    tabID = targetID
                    neighbour = 1

                fsm = 'astar'
            else:
                # save points of target table in a matrix and remove the multiple points
                tempMatrix = np.round(250*ptsTable)/250
                ptsTarget = np.unique[[ptsTarget, tempMatrix.transpose()], 'rows']

                # Increment the neighbour counter to send bot to next table neighbour
                neighbour = neighbour + 1
                if neighbour > 4:
                    fsm = 'imageAnalysis'
                else:
                    fsm = 'astar'

        ## Analyze the points taken through the depth pictures to infer objects positions and table center and differentiate table 1 and 2
        elif fsm == 'imageAnalysis':
