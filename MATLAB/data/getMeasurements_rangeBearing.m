function [zgt,zn] = getMeasurements_rangeBearing(x, lm, s)
%% Description:
    % Simulates a range and bearing measurement, expected from such
    % sensors such as LiDAR or camera from ground truth perspective.
%% Inputs:
    % x:  Ground-truth: current states 
    % lm: Ground-truth Landmarks
%% Outputs:
    % zgt: Ground truth measurements [range;bearing]
    % zn:  Noise measurements, to simulate a realworld sensor [range;bearing]
%% Notes:
    % - 
%% References:
    % EKF SLAM algorithm with ML corresopences Table 10.2 pg 322,
    % Probabilistic Robotics, 2006, [Thrun et al]

%% get measured

zgt = [];
zn  = [];

if size(lm,1) >= size(lm,2)
    total_landmarks = size(lm,1);
else
    total_landmarks = size(lm,2);
end

% Get Measurments each landmark (for loop this)
for i = 1:total_landmarks
    dx = lm(i,1) - x(1);
    dy = lm(i,2) - x(2);
    
    % Calculate range and bearing
    r = sqrt(dx^2 + dy^2);
    b = wrapTo2Pi(atan2(dx,dy) - x(3));
    
    % Check to see if it is within field of view
    if (r <= s.par_rb.maxRange) && ...
            (b >= (360-s.par_rb.angle/2)*pi/180 || ...
             b <= (s.par_rb.angle/2)*pi/180)
        zgt = [zgt, [r;b]];
        
        rn  = r + s.par_rb.noise_range*randn;
        bn  = b + s.par_rb.noise_angle*randn;
        zn  = [zn, [rn;bn]];
    end
end

    
%% end of function
end