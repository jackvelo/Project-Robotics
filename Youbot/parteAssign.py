
        ## Analyze the points taken through the depth pictures to infer objects positions and table center and differentiate table 1 and 2
        elif fsm == 'imageAnalysis':

            tab1ID = objectsTablesID[0]
            xMean = np.mean(ptsTable1[:, 0])
            yMean = np.mean(ptsTable1[:, 1])

            # Remove parasite points for table (points placed to far from it)
            ptsTable1 = np.transpose(ptsTable1)
            a = np.sqrt((ptsTable1[0, :] - xMean)**2 + (ptsTable1[1, :] - yMean)**2)
            ptsTable1 = ptsTable1[:, a < 0.6]

            xCenter = (np.max(ptsTable1[0, :]) + np.min(ptsTable1[0, :]))/2
            yCenter = (np.max(ptsTable1[1, :]) + np.min(ptsTable1[1, :]))/2
            preciseCenter1 = np.zeros((1, 2))
            preciseCenter1[0, :] = [xCenter, yCenter]
            tablesRealCenter[tab1ID, :] = preciseCenter1

            # Remove parasite points for objects (points placed to far from it)
            ptsObjects1 = np.transpose(ptsObjects1)
            a = np.sqrt((ptsObjects1[0, :] - xCenter)**2 + (ptsObjects1[1, :] - yCenter)**2)
            ptsObjects1 = ptsObjects1[:, a < 0.4]
            ptsObjects1 = np.transpose(ptsObjects1)

            # Considering the second table
            tab2ID = objectsTablesID[1]
            xMean = np.mean(ptsTable2[:, 0])
            yMean = np.mean(ptsTable2[:, 1])

            # Remove parasite points for table (points placed to far from it)
            ptsTable2 = np.transpose(ptsTable2)
            a = np.sqrt((ptsTable2[0, :] - xMean)**2 + (ptsTable2[1, :] - yMean)**2)
            ptsTable2 = ptsTable2[:, a < 0.6]

            xCenter = (np.max(ptsTable2[0, :]) + np.min(ptsTable2[0, :]))/2
            yCenter = (np.max(ptsTable2[1, :]) + np.min(ptsTable2[1, :]))/2
            preciseCenter2 = np.zeros((1, 2))
            preciseCenter2[0, :] = [xCenter, yCenter]
            tablesRealCenter[tab1ID, :] = preciseCenter2

            # Remove parasite points for objects (points placed to far from it)
            ptsObjects2 = np.transpose(ptsObjects2)
            a = np.sqrt((ptsObjects2[0, :] - xCenter)**2 + (ptsObjects2[1, :] - yCenter)**2)
            ptsObjects2 = ptsObjects2[:, a < 0.4]
            ptsObjects2 = np.transpose(ptsObjects2)

            # Through the ptsTarget points, we can have a more accurate idea of the target table points and center
            # point. So update them
            xMean = np.mean(ptsTarget[:, 0])
            yMean = np.mean(ptsTarget[:, 1])

            # Remove parasite points for table (points situated to far from it)
            ptsTarget = np.transpose(ptsTarget)
            a = np.sqrt((ptsTarget[0, :] - xMean)**2 + (ptsTarget[1, :] - yMean)**2)
            ptsTarget = ptsTarget[:, a < 0.6]

            xCenter = (np.max(ptsTarget[0, :]) + np.min(ptsTarget[0, :]))/2
            yCenter = (np.max(ptsTarget[1, :]) + np.min(ptsTarget[1, :]))/2
            preciseCenter = np.zeros((1, 2))
            preciseCenter[0, :] = [xCenter, yCenter]
            tablesRealCenter[targetID, :] = [xCenter, yCenter]

            # plot target table
            plt.close()
            plt.matshow(ptsTarget)
            plt.colorbar()
            plt.show()

            # ADD THIS
            # from sklearn.cluster import KMeans

            # # Regrouping the points to find the centers of the objects on the tables.
            # distToClusterCenters1 = math.inf

            for i in range(30):
                kmeans = KMeans(n_clusters=5, random_state=0).fit(ptsObjects1)
                centerObject1 = kmeans.cluster_centers_

            # distToClusterCenters2 = Inf;

            for i in range(30):
                kmeans = KMeans(n_clusters=5, random_state=0).fit(ptsObjects2)
                centerObject2 = kmeans.cluster_centers_

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

                # temp = idObject1;
                # idObject1 = idObject2;
                # idObject2 = temp;

                temp = centerObject1
                centerObject1 = centerObject2
                centerObject2 = temp

                temp = objectsTablesID[0]
                objectsTablesID[0] = objectsTablesID[1]
                objectsTablesID[1] = temp

            # Plot table 1
            plt.close()
            plt.matshow(ptsTable1[0, :], ptsTable1[1, :])
            plt.colorbar()
            plt.show()

            # Plot the clustering for table 1
            plt.close()
            plt.matshow(ptsObjects1[:, 0], ptsObjects1[:, 1], ptsObjects1[:, 2], centerObject1[:, 0], centerObject1[:, 1], centerObject1[:,2])
            plt.colorbar()
            plt.show()

            # Plot table 2
            plt.close()
            plt.matshow(ptsTable2[0, :], ptsTable2[1, :])
            plt.colorbar()
            plt.show()

            # Plot the clustering for table 2
            plt.close()
            plt.matshow(ptsObjects2[:, 0], ptsObjects2[:, 1], ptsObjects2[:, 2], centerObject2[:, 0], centerObject2[:, 1], centerObject2[:,2])
            plt.colorbar()
            plt.show()

            # save('ptsObjects1.mat','ptsObjects1')
            # save('ptsObjects2.mat','ptsObjects2')
            # save('ptsTable1.mat','ptsTable1')
            # save('ptsTable2.mat','ptsTable2')
            # save('idObject1.mat','idObject1')
            # save('idObject2.mat','idObject2')
            # save('centerObject1.mat','centerObject1')
            # save('centerObject2.mat','centerObject2')
            # save('objectsTablesID.mat','objectsTablesID')
            # save('objectsTablesID.mat','objectsTablesID')
            # save('tablesCentersReal.mat','tablesCentersReal')

            # fprintf('table 1 is (%f,%f)', tablesCentersReal(objectsTablesID(1),1),...
            #     tablesCentersReal(objectsTablesID(1),2))

            tableID = 0
            objectID = 0
            fsm = 'computedestObjects'

        # Compute the positions where we will send to robot to place the blocks on the target table
        elif fsm == 'computedestObjects':
            # If n objects to place on the target, find target table free neighbour
            # cells with the more space between them

            # For milestone 2a, we have 5 destinations
            destObjects = np.zeros[5, 2]
            # divide 360° in 5 for milestone 2a
            angles = np.linspace(0, 2*pi, num=6)
            centerTarget = tablesCentersReal[targetID, :]

            for k in range(5):
                j = 0.5
                destObjects[k, :] = centerTarget + [math.cos(angles[k]) * j, math.sin(angles[k]) * j]
                destObjects[k, 0] = round((destObjects[k, 0] + 7.5)/resolution) + 1
                destObjects[k, 1] = round((destObjects[k, 1] + 7.5)/resolution) + 1

                # Verify that the cell is a free cell
                while statesMap(destObjects(k, 0), destObjects(k, 1)) != 0:
                    j = j + 0.1
                    destObjects[k, :] = centerTarget + [math.cos(angles[k]) * j, math.sin(angles[k]) * j]
                    destObjects[k, 0] = round((destObjects[k, 0] + 7.5)/resolution) + 1
                    destObjects[k, 1] = round((destObjects[k, 1] + 7.5)/resolution) + 1

            mat = np.matrix(destObjects)
            with open('savedestObjects.txt', 'wb') as f:
                for line in mat:
                    np.savetxt(f, line, fmt='%.2f', delimiter=',')
            mat = np.matrix(centerTarget)
            with open('centerTarget.txt', 'wb') as f:
                for line in mat:
                    np.savetxt(f, line, fmt='%.2f', delimiter=',')

            fsm = 'calculateObjectGoal'
            print('Switching to state: ', fsm)

        # For each object to grasp, find nearest cell to send the robot to
        elif fsm == 'calculateObjectGoal':
            if tableID == 1:
                centerObject = centerObject1
                tableCenter = [tablesCentersReal[objectsTablesID[0], 0], tablesCentersReal[objectsTablesID[0], 1]]
            else:
                centerObject = centerObject2
                tableCenter = [tablesCentersReal[objectsTablesID[1], 0], tablesCentersReal[objectsTablesID[1], 1]]

            posObject = [centerObject[objectID, 0], centerObject[objectID, 1]]

            angle = atan((posObject[1] - tableCenter[1]) / (posObject[0] - tableCenter[0]))

            if posObject[0] < tableCenter[0]:
                angle = angle + math.pi

            j = 0.5
            posNearObject = tableCenter + [math.cos(angle) * j, math.sin(angle) * j]
            posNearObject[0] = round((posNearObject[0] + 7.5)/resolution) + 1
            posNearObject[1] = round((posNearObject[1] + 7.5)/resolution) + 1

            # Verify that the cell is a free cell
            while statesMap[posNearObject[0], posNearObject[1]] != 0:
                j = j + 0.1
                posNearObject = tableCenter + [math.cos(angle) * j, math.sin(angle) * j]
                posNearObject[0] = round((posNearObject[0] + 7.5)/resolution) + 1
                posNearObject[1] = round((posNearObject[1] + 7.5)/resolution) + 1

            fsm = 'astar'
            print('Switching to state: ', fsm)

        # If needed rotate the robot parallel to the table (table on its left) and slide closer or further
        elif fsm == 'rotateAndSlide':
            [res, youbotPos] = vrep.simxGetObjectPosition(clientID, h.ref, -1, vrep.simx_opmode_buffer)
            vrchk(vrep, res, True)
            # Get initial sliding position
            startingPoint = [youbotPos[0], youbotPos[1]]

            # If the robot should slide closer to the table, add the rotation phase before the sliding phase
            if slideCloser:

                # if holding and object and have to slide closer, we know that the table of interest is the target
                if holdObject:
                    centerToReach = centerTarget
                else:
                    centerToReach = tableCenter

                angle = math.atan((centerToReach[0] - youbotPos[0]) / (centerToReach[1] - youbotPos[1]))

                # Find the angle to ensure table on the left
                if youbotPos[1] >= centerToReach[1]:
                    angle = - angle - math.pi/2
                else:
                    angle = pi - angle - math.pi/2

                # Launch the rotation
                rotateRightVel = 1.7
                rotate2(rotateRightVel, angle, h, ClientID, vrep)
                rotateRightVel = 0

                # Proceed to the computation to know how far we are from
                # the table and which distance has to be travelled
                angle = math.atan((startingPoint[1] - centerToReach([1]) / (startingPoint[0] - centerToReach[0]))

                if startingPoint[0] < centerToReach[0]:
                    angle = angle + math.pi

                closestPoint = [(centerToReach[0] + 0.63 * math.cos(angle)), (centerToReach[1] + 0.63 * math.sin(angle))]

                totDist = sqrt((startingPoint[0] - closestPoint[0])**2 +(startingPoint[1] -  closestPoint[1])**2)

            travelledDist = 0

            # Make the robot slide until it has covered the distance we want
            while travelledDist < totDist:
                if slideCloser:
                    h = youbot_drive(vrep, h, 0, 0.1, 0)
                else:
                    h = youbot_drive(vrep, h, 0, -0.2, 0)
                [res, youbotPos] = vrep.simxGetObjectPosition(clientID, h.ref, -1, vrep.simx_opmode_buffer)
                vrchk(vrep, res, true)
                travelledDist = sqrt((startingPoint[0] -  youbotPos[0])**2 +(startingPoint[1] -  youbotPos[1])**2)

            # Stops wheels if position reached
            stopWheels(h, clientID, vrep)

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
            [res, rgbdPos] = vrep.simxGetObjectPosition(clientID, h.rgbdCasing, -1,vrep.simx_opmode_oneshot_wait)
            vrchk(vrep, res, True)

            angle = math.atan((rgbdPos[0]- centerObject[objectID, 0])/(centerObject[objectID, 1]-rgbdPos[1])) - math.pi/2
            if (centerObject[objectID, 1] - rgbdPos[1]) > 0:
                angle = angle + math.pi

            # Orientate rgbd camera to the object of interest
            vrep.simxSetObjectOrientation(id, h.rgbdCasing, -1,[0 0 angle], vrep.simx_opmode_oneshot)

            # get rgb camera angle
            [res, rgbdEuler] = vrep.simxGetObjectOrientation(id, h.rgbdCasing, -1,vrep.simx_opmode_oneshot_wait)
            vrchk(vrep, res, True)

            # Use a very small view to focus on the object
            res = vrep.simxSetFloatSignal(id, 'rgbd_sensor_scan_angle', pi/8, vrep.simx_opmode_oneshot_wait)
            vrchk(vrep, res)

            # take a depth picture
            res = vrep.simxSetIntegerSignal(id, 'handle_xyz_sensor', 1, vrep.simx_opmode_oneshot_wait)
            vrchk(vrep, res)

            # Store picture in pts
            pts = youbot_xyz_sensor(vrep, h, vrep.simx_opmode_oneshot_wait)

            # Only keep points within 0.8 meter, to focus on the table
            pts = pts[0:3, pts[3, :] < 0.8]

            # Invert 3rd and 1st line to get XYZ in the right order
            pts = [pts[2, :], pts[0, :], pts[1, :]]
            trf = trans_rot_matrix(rgbdPos, rgbdEuler)  # check the file trans_rot_matrix for explanation
            # Apply transfer function to get real coordinates
            ptsObject = np.dot(trf, pts)

            # Get points high enough (= remove table points)
            ptsObjectT = ptsObject[:, ptsObject[2, :] > 0.187]
            ptsObject = ptsObjectT.transpose()

            # Focus on points with X and Y distances to the centers smaller than 0.06 to focus on object
            ptsObject = np.unique([(ptsObject[ptsObject[:, 0]]) > (centerObject[objectID, 0]-0.06), 0:2], 'rows')
            ptsObject = ptsObject[(ptsObject[:, 0]) < (centerObject[objectID, 0]+0.06), :]
            ptsObject = ptsObject[(ptsObject[:, 1]) > (centerObject[objectID, 1]-0.06), :]
            ptsObject = ptsObject[(ptsObject[:, 1]) < (centerObject[objectID, 1]+0.06), :]

            fsm = 'armMotion'
            print('Switching to state: ', fsm)

        # Manage the arm to take an object or to put an object depending on the context
        elif fsm == 'armMotion':
            # Condition verify if we have to grasp an object
            if not holdObject:
                # get rgb position
                [res, rgbdPos] = vrep.simxGetObjectPosition(clientID, h.rgbdCasing, -1,vrep.simx_opmode_oneshot_wait)
                vrchk(vrep, res, True)

                # get youbotAngle
                [res, youbotEuler] = vrep.simxGetObjectOrientation(id, h.ref, -1, vrep.simx_opmode_buffer)
                vrchk(vrep, res, True)

                gripPos = [0, 0.166, 0]
                trf = trans_rot_matrix(youbotPos, youbotEuler)  # check the file trans_rot_matrix for explanation
                # Apply transfer function to get real gripper coordinates
                gripPos = np.dot(trf, gripPos.transpose())

                distToGrip = bsxfun(@hypot, ptsObject(:,1)-gripPos(1), ptsObject(:,2)-gripPos(2))
                minDist = min(bsxfun(@hypot, ptsObject(:,1)-gripPos(1), ptsObject(:,2)-gripPos(2)))

                # Among the object points, find the one which is the closer to the gripper
                indexNearestPoint = find(distToGrip == minDist,1)

                # Plot the given configuration : object, bot, gripper,
                # camera, object center, nearest object point from bot

                figure;
                plot(ptsObject(:,1), ptsObject(:,2), '*',rgbdPos(1), rgbdPos(2), '.', gripPos(1), gripPos(2), '+',...
                     centerObject(objectID,1),centerObject(objectID,2),'*' , youbotPos(1),  youbotPos(2), '^',...
                     ptsObject(indexNearestPoint,1), ptsObject(indexNearestPoint,2), '*' );

                % Re-adjust center of the object to get a better manipulation with the gripper
                centerObject(objectID,1) = (centerObject(objectID,1) + ptsObject(indexNearestPoint, 1))/2;
                centerObject(objectID,2) = (centerObject(objectID,2) + ptsObject(indexNearestPoint, 2))/2;

                 % Find the angle of first joint --> orientation to the
                 % object
                 angleJ1 = atan((gripPos(1)- centerObject(objectID,1))/...
                     (centerObject(objectID,2)-gripPos(2))) - youbotEuler(3);
                 if (centerObject(objectID,2)-gripPos(2)) > 0
                     angleJ1 = angleJ1 + pi;
                 end
            end

            % Put the gripper in vertical position
            res = vrep.simxSetJointTargetPosition(id, h.armJoints(2), 0, vrep.simx_opmode_oneshot);
            vrchk(vrep, res, true);
            res = vrep.simxSetJointTargetPosition(id, h.armJoints(3), 0, vrep.simx_opmode_oneshot);
            vrchk(vrep, res, true);
            res = vrep.simxSetJointTargetPosition(id, h.armJoints(4), 0, vrep.simx_opmode_oneshot);
            vrchk(vrep, res, true);
            res = vrep.simxSetJointTargetPosition(id, h.armJoints(5), 0, vrep.simx_opmode_oneshot);
            vrchk(vrep, res, true);
            % Wait for the arm to get the given angles
            pause(1);

            % Apply the computated joint 1 angle
            res = vrep.simxSetJointTargetPosition(id, h.armJoints(1), angleJ1, vrep.simx_opmode_oneshot);
            vrchk(vrep, res, true);
            % Wait for the arm to get the given angle
            pause(1)

            % Condition verify if we have to grasp an object
            if not(holdObject)
                % Youbot constants
                length1 = 0.147+0.0952; % initial height
                length2 = 0.155; % first arm length
                length3 = 0.135; % second arm length
                length4 = 0.15; % third arm length

                gripPos(3) = length1;

                % Find height difference between object and gripper
                heightDiff = centerObject(objectID,3) - gripPos(3);
                % Find distance between gripper and object
                DistGripObject = sqrt((centerObject(objectID,1) - gripPos(1))^2 + ...
                    (centerObject(objectID,2) - gripPos(2))^2);

                % System of equations to find angles to take
                syms phi2 phi3
                eq1 = length2*sin(phi2) + length3*sin(phi2 + phi3) + length4 == DistGripObject;
                eq2 = length2*cos(phi2) + length3*cos(phi2 + phi3) == heightDiff;

                % Restrict the search interval to get desired angles values
                searchInterval = [-1.5707963705063, 1.308996796608; 0, 2.2863812446594];
                S = vpasolve([eq1 eq2], [phi2 phi3], searchInterval);
                interPhi2 = S.phi2;
                interPhi3 = S.phi3;

                Phi4  = pi/2 - interPhi2(1) - interPhi3(1);
            end

            % Orientate arms with the angles just computed
            res = vrep.simxSetJointTargetPosition(id, h.armJoints(3), interPhi3(1), vrep.simx_opmode_oneshot);
            vrchk(vrep, res, true);
            res = vrep.simxSetJointTargetPosition(id, h.armJoints(4), Phi4, vrep.simx_opmode_oneshot);
            vrchk(vrep, res, true);
            pause(2);
            res = vrep.simxSetJointTargetPosition(id, h.armJoints(2), interPhi2(1), vrep.simx_opmode_oneshot);
            vrchk(vrep, res, true);
            pause(2);

            if not(holdObject)
                % Close the gripper if not object hold
                res = vrep.simxSetIntegerSignal(id, 'gripper_open', 0, vrep.simx_opmode_oneshot_wait);
                vrchk(vrep, res);
                % Ensure gripper well closed before continue
                pause(2);
                % We have to know that we are now holding an object
                holdObject = true;

            else
                % Open the gripper if object hold to place it on the table
                res = vrep.simxSetIntegerSignal(id, 'gripper_open', 1, vrep.simx_opmode_oneshot_wait);
                vrchk(vrep, res);

                % Put the gripper in vertical position to avoid hitting
                % object
                res = vrep.simxSetJointTargetPosition(id, h.armJoints(2), 0, vrep.simx_opmode_oneshot);
                vrchk(vrep, res, true);
                res = vrep.simxSetJointTargetPosition(id, h.armJoints(3), 0, vrep.simx_opmode_oneshot);
                vrchk(vrep, res, true);
                res = vrep.simxSetJointTargetPosition(id, h.armJoints(4), 0, vrep.simx_opmode_oneshot);
                vrchk(vrep, res, true);
                res = vrep.simxSetJointTargetPosition(id, h.armJoints(5), 0, vrep.simx_opmode_oneshot);
                vrchk(vrep, res, true);
                pause(1.5);
                % We have to know that we do not hold object anymore
                holdObject = false;
                objectID = objectID + 1;
                if objectID > 5
                    objectID = 1;
                    tableID = tableID + 1;
                end
                % If the robot is supposed to grasp object of table 2 now,
                % stop here for milestone 2a
                if tableID == 2
                    disp('Grasping Finished');
                    p = false;
                end
            end

            % Go back to rest position.
            % Set each joint to their original angle, as given by startingJoints.
            for i = 1:5
                res = vrep.simxSetJointTargetPosition(id, h.armJoints(i), startingJoints(i), vrep.simx_opmode_oneshot);
                vrchk(vrep, res, true);
            end
            pause(2)
            fsm = 'rotateAndSlide';
        end
    end
end
<<<<<<< Updated upstream
=======

>>>>>>> Stashed changes
