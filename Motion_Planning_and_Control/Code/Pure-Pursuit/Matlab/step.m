function [v, w, lookaheadPoint, targetDir] = step(pars, currentPose)

        wayptsIn = controller.Waypoints;
        waypts = cast(wayptsIn, 'like', currentPose);

        % Handle nan values
        b = ~isnan(waypts);
        nanidx = b(:,1) & b(:,2);
        waypoints = waypts(nanidx, :);

        if isempty(waypoints)
            v = cast(0, 'like', currentPose);
            w = cast(0, 'like', currentPose);
            targetDir = cast(0, 'like', currentPose);
            lookaheadPoint = currentPose(1:2);
            return;
        end
        % Find the nearest point as the projection
        computeProjectionPoint(pars, currentPose, waypoints);
        
        % Compute carrot point based on projection. If near end, then
        % the end point is the carrot point
        pars.LookaheadPoint = pars.getLookaheadPoint(waypoints);

        % Angle between robot heading and the line connecting robot and
        % the carrot point
        slope = atan2((pars.LookaheadPoint(2) - currentPose(2)), ...
                      (pars.LookaheadPoint(1) - currentPose(1)));
        alpha = robotics.internal.angdiff(currentPose(3), slope);
        
        % Using eq. (2) on page 11 of Reference [1].
        w = (2*sin(alpha))/pars.LookaheadDistance;

        % Pick a constant rotation when robot is facing in the opposite
        % direction of the path
        if abs(abs(alpha) - cast(pi, 'like', currentPose)) < sqrt(eps(class(currentPose)))
            w = sign(w)*1;
        end
        
        w = pars.saturate(w);

        v = cast(pars.DesiredLinearVelocity, 'like', currentPose);

        lookaheadPoint = pars.LookaheadPoint;
        %Update the last pose
        pars.LastPose = [currentPose(1) currentPose(2) currentPose(3)];

        targetDir = alpha;
        if isnan(targetDir)
            targetDir = cast(0, 'like', currentPose);
        end

end