function [v, w, lookaheadPoint, targetDir, alpha] = Copy_of_next_step(pars, currentPose)

wayptsIn = pars.Waypoints;
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
obj = compProjectionPoint(pars ,currentPose, waypoints);

% Compute carrot point based on projection. If near end, then
% the end point is the carrot point
pars.LookaheadPoint = getLookaheadPoint(pars,obj,waypoints);

% Angle between robot heading and the line connecting robot and
% the carrot point
slope = atan2((pars.LookaheadPoint(2) - currentPose(2)), ...
              (pars.LookaheadPoint(1) - currentPose(1)));
          
          
alpha = anglediff(currentPose(3),slope);

% from unicycle model 
D = floor(hypot(pars.LookaheadPoint(1)-currentPose(1),pars.LookaheadPoint(2) - currentPose(2)));
L = pars.LookaheadDistance/D;
w = (2*sin(alpha))/L;
% w = (2*sin(alpha))/pars.LookaheadDistance;
fprintf('alpha = %.2f slope= %.2f\n',alpha,slope)
% Pick a constant rotation when robot is facing in the opposite
% direction of the path
if abs(abs(alpha) - pi) < sqrt(eps(class(currentPose)))
    w = sign(w)*1;
end


%saturate the angular velocity
if abs(w) > pars.MaxAngularVelocity
w = sign(w)*pars.MaxAngularVelocity;
end

%calculate desired velocity
v = cast(pars.DesiredLinearVelocity, 'like', currentPose);
%         grad = abs(alpha) - abs(pars.lastAlpha);
%         if abs(grad) > 3
%             %if the corner is aggresive slow down
%             v = cast(pars.MinLinearVelocity, 'like', currentPose);
%         else
%             v = cast(pars.DesiredLinearVelocity, 'like', currentPose);
%         end
pars.lastAlpha = alpha;
lookaheadPoint = pars.LookaheadPoint;
%Update the last pose
pars.LastPose = [currentPose(1) currentPose(2) currentPose(3)];

targetDir = alpha;
if isnan(targetDir)
    targetDir = cast(0, 'like', currentPose);
end

end