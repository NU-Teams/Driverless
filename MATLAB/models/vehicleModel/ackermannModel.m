function [model] = ackermannModel(x,u)
%% Description:  
    % [model] = bicycleModel(x,u)
    % Bicycle Model of a 5 state vehicle. This model is intended for the NU
    % Racing vehicle model. 
%% Inputs:
    % x: System state:  x(1) = x(t) x [metres] Cartesian Coordinates of car
%           at time t:  x(2) = y(t) y [metres] Cartesian Coordinates of car
%                       x(3) = theta(t) heading angle of car
%                       x(4) = v(t) linear velocity of car
%                       x(5) = delta(t) steering angle of car
%
    % u: System Inputs: u(1) = Force
%                       u(2) = phi?? steering rate
%
%% Outputs
    % model: updated states with input propegation [x,y,theta] wrt u
%
%% Notes:
     % model is used as an example to verify SLAM implementation. 
%% References
    % Robot model sourced from lab 9 of MCHA6100 lab by Adrian Wills

%% Roll out..
% Setup System parameters
H = 0.76; % [m] Encoder to centre
L = 2.83; % [m] Length of car (front to rear axle)
a = 3.78; % [m] Length of LiDAR to rear axle
b = 0.5;  % [m] Length of LiDAR to centre of vehice

% Car body forward velocity from encoder data
vc = u(1)/(1-tan(u(2))*H/L);

% % Model
% model   =      [u(1).*sin(x(3));
%                 u(1).*cos(x(3));
%                (u(1)/lr)*tan(u(2))];
% Model
model   =      [vc.*sin(x(3)) - (vc/L)*tan(u(2))*(a*cos(x(3)) - b*sin(x(3)));
                vc.*cos(x(3)) - (vc/L)*tan(u(2))*(-a*sin(x(3)) + b*cos(x(3)));
               -(vc/L)*tan(u(2))];


%% end of function 
end