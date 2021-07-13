function d = anglediff(x, y)
%ANGDIFF Find difference between two angles
%
%   DTHETA = angdiff(THETA1, THETA2) computes difference between angles 
%   THETA1 and THETA2 and returns the difference DTHETA within the interval 
%   [-pi pi]. Positive, odd multiples of pi map to pi and negative, odd 
%   multiples of pi map to -pi.


if nargin == 1
    % Single input means that x should be a numeric vector
    % Syntax: ANGDIFF(X)
    
    % Calculate difference
    d = diff(x);
else
    % There are two inputs
    % Syntax: ANGDIFF(X,Y)    
    
    d = y - x;
end

% Make sure the output is in the [-pi,pi) range
if any(abs(d) > pi, 'all')
    % Only wrap values if one or more needs to be wrapped
    piVal = cast(pi,'like',d);
    
    theta = d + piVal;  
    
    twoPiVal = cast(2*pi,'like',theta);

    % Check if inputs are positive
    pos = (theta > 0);
    % Wrap to 2*pi
    thetaWrap = mod(theta, twoPiVal);

    % Make sure that positive multiples of 2*pi map to 2*pi
    thetaWrap((thetaWrap == 0) & pos) = twoPiVal;
    
    d = thetaWrap - piVal;
end

end
