function loadObject(obj, svObj)


        % Find index of each waypoint
        coder.varsize('tempIndex');
        tempIndex = 1;
        for i=1:size(svObj.Waypoints,1)
            distToPt = sum((svObj.Path - svObj.Waypoints(i,:)).^2,2);
            tempIndex = [tempIndex find(distToPt < sqrt(eps)).'];  %#ok<AGROW>
        end
        index = sort(unique(tempIndex));
        higherIdx = index > svObj.ProjPointIdx;
        obj.ProjectionLineIndex = max(find(higherIdx, 1)-1, 1);
        obj.ProjectionPoint = svObj.Path(svObj.ProjPointIdx, :);
        computeProjectionPoint(obj, svObj.LastPose, svObj.Waypoints);
    else
        % Uninitialized 15a-16a objects
        % Only double datatype was supported in 15a-16a
        obj.ProjectionPoint = nan(1,2);
        obj.LookaheadPoint = zeros(1,2);
        obj.LastPose = zeros(1,3);
        obj.ProjectionLineIndex = 0;
    end

    if isempty(svObj.Waypoints)
        % Prevent warning for uninitialized object
        return;
    else
        % Call base class method
        loadObject@matlab.System(obj,svObj,wasLocked);
    end
end