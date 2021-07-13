        function out = compProjectionPoint(obj, pose, waypoints)
        %computeProjectionPoint Find closest point past the last Projection point
            [obj] = setupObject(pose);
            pose = reshape(pose,1,3);
            searchFlag = false;
            % If Projection point is not initialized, start searching from
            % first waypoint
            if obj.ProjectionLineIndex == 0
                searchFlag = true;
                obj.ProjectionPoint = waypoints(1,1:2);
                obj.ProjectionLineIndex = cast(1, 'like', pose);
            end

            if size(waypoints, 1) == 1
                obj.ProjectionPoint = waypoints(1,1:2);
                return;
            end
            % Start searching from the current projection line segment
            [obj.ProjectionPoint, minDistance] = ...
                closestPointOnLine(obj.ProjectionPoint, ...
                                                     waypoints(obj.ProjectionLineIndex+1,1:2), pose(1:2));

            dist = norm(obj.ProjectionPoint - waypoints(obj.ProjectionLineIndex+1,1:2));
            for i = obj.ProjectionLineIndex+1:size(waypoints,1)-1

                if ~searchFlag && dist > obj.LookaheadDistance
                    break;
                end
                dist = dist + norm(waypoints(i,1:2) - waypoints(i+1,1:2));
                % Check the remaining waypoints
                [tempPoint, tempDistance] = ...
                    closestPointOnLine(waypoints(i,1:2), ...
                                                         waypoints(i+1,1:2), pose(1:2));

                if tempDistance < minDistance
                    minDistance = tempDistance;
                    obj.ProjectionPoint = tempPoint;
                    obj.ProjectionLineIndex = cast(i, 'like', pose);
                end
            end
            out = obj;
        end
