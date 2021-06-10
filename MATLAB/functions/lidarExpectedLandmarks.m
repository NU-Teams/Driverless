function expected = lidarExpectedLandmarks(s,x)
%% Description:
% Returns landmarks expected to be seen in range bearing format, and a
% marker.
    % 0: Landmark not matched (plot measurement in red)
    % 1: Landmark matched (plot measurement in green)
    % 2: New landmark sighted (plot measurement in blue)
expected = [];
% Number of landmarks
M = length(x(s.nx+1:end))/2;            

for o = 1:M % can I vectorise this
    % Convert landmark mean to range bearing
    dx = x(s.nx+s.ny*o-1) - x(1);
    dy = x(s.nx+s.ny*o) - x(2);
    q = dx^2 + dy^2;

    % Psuedo range/bearing model for landmark
    ztemp = [sqrt(q);
             wrapTo2Pi(atan2(dx,dy) - x(3))];

    % Is landmark expected to be seen?
    if (ztemp(1) <= s.par_rb.maxRange) && (ztemp(2) >= (360-1-s.par_rb.angle/2)*pi/180 || ztemp(2) <= (1+s.par_rb.angle/2)*pi/180)
        % Add a counter
        expected = [expected, o];        
    end
end

%% end of function
end