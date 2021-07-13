function lookaheadPoint = getLookaheadPoint(pars,obj, waypoints)
%getLookaheadPoint Find the look ahead point on the path past
%the Projection point.

    if size(waypoints, 1) == 1
        lookaheadPoint = waypoints(1,1:2);
        return;
    end

    % First check the current line segment
    dist = norm(obj.ProjectionPoint-waypoints(obj.ProjectionLineIndex+1,1:2));
    lookaheadStartPt = obj.ProjectionPoint;
    lookaheadEndPt = waypoints(obj.ProjectionLineIndex+1,1:2);
    overshootDist = dist - pars.LookaheadDistance;
    lookaheadIdx = obj.ProjectionLineIndex;

    % If the remaining path on current line segment is not long
    % enough for look ahead, check the waypoints past current line
    % segment.
    while overshootDist < 0 && lookaheadIdx < size(waypoints,1)-1
        lookaheadIdx = lookaheadIdx + 1;

        lookaheadStartPt = waypoints(lookaheadIdx,1:2);
        lookaheadEndPt = waypoints(lookaheadIdx+1,1:2);
        dist = dist + norm(lookaheadStartPt-lookaheadEndPt);

        overshootDist = dist - pars.LookaheadDistance;
    end

    % Find the exact look ahead point by interpolating between the
    % start and end point of the line segment on which the
    % lookahead point lies.
    alpha = overshootDist/norm(lookaheadStartPt - lookaheadEndPt);
    if alpha > 0
        lookaheadPoint = alpha.*lookaheadStartPt + (1-alpha).*lookaheadEndPt;
    else
        lookaheadPoint = lookaheadEndPt;
    end
end