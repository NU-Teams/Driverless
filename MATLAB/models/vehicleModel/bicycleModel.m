function [model] = bicycleModel(x,u)
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
% m  = 250;    % [kg]   - Car mass
% lr  = 0.5;   % [m]    - distance: CoG to rear wheel axis
% lf  = 0.5;   % [m]    - distance: CoG to front wheel axis

% Params provided by Aidan
m  = 25;    % [kg]   - Car mass
lr  = 1;   % [m]    - distance: CoG to rear wheel axis
lf  = 1;   % [m]    - distance: CoG to front wheel axis

K = lr/(lr+lf);
Beta = atan(K*tan(x(5)));

% Model
model   =      [x(4).*sin(x(3)+ Beta);
               (x(4).*cos(x(3)+ Beta));
               (x(4)/lr)*sin(Beta);
                u(1)/m;
                u(2)];


%% end of function 
end