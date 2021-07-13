function [w, index] = pure_pursuit_steer_control(state, map, struct)
%pure_pursuit_steer_control Compute Control output for vehical to head toward desired waypoints
%   The pure-pursuit Controller is A geometric Controller that consists of
%   two parts, the first is the trajectory search component
[index, Lf] = Search_target_index(state,struct,map);


tx = map(index, 1);
ty = map(index, 2);



% % Angle between robot heading and the line connecting robot and
% % the carrot point
% slope = atan2((ty - state(2)), ...
%               (tx - state(1)));
%           
%           
% alpha = anglediff(state(3),slope);
% 
% % from unicycle model 
% w = (2*sin(alpha))/Lf;
% % fprintf('alpha = %.2f slope= %.2f\n',alpha,slope)
% % Pick a constant rotation when robot is facing in the opposite
% % direction of the path
% if abs(abs(alpha) - pi) < sqrt(eps(class(state)))
%     w = sign(w)*1;
% end




alpha = atan2(ty - state(2), tx - state(1)) - state(3);

w = atan2(2* struct.Wb*sin(alpha)/Lf, 1);


end



function [index, Lf] = Search_target_index(state,struct,map)

if struct.oldNearestPoint == 0 
    %to speed up only do this first time
    dx = state(1) - map(:,1);
    dy = state(2)- map(:,2);
    distance = hypot(dx, dy); %sqrt(x^2+y^2)
    [~,index] = min(distance); %find the closest point
    struct.old_nearest_point_index = index; %assign new value    
    
else
    index = struct.old_nearest_point_index;
    distance_this_index = calc_distance(state, map.x(index), map.y(index));
    
    while(1)
        distance_next_index = calc_distance(map.x(index + 1), map.y(index +1));
        if distance_this_index < distance_next_index
            break;
        end
        
        if (index +1) < length(map.x)
        index = index + 1;
        else
            distance_this_index = distance_next_index;
        end
        struct.old_nearest_point_index = index;
    end
end

Lf = struct.k*1 + struct.Lfc;  %update look ahead distance  %TODO add velocity to gain for more acurate tuning

%after finding the closest point place it the distance chosen through the
%lookahead distance
while Lf > calc_distance(state, map(index,1), map(index,2))
    if (index +1) >= length(map(:,1))
        break;
    end
    index= index+1;
end

end



function [Distance_this_index] = calc_distance(state, pointX, pointY)
% calculate the distance of the object from desired point
dx = state(1) - pointX;
dy = state(2) - pointY;

Distance_this_index = hypot(dx,dy); %sqrt(x^2+y^2)

end