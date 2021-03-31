function [Jx, Ju] = robotModel2Wheels_Jacobian(s,x,u)
%% Description:
    % Returns Jacobian of desired model from a look up table (pre calculated).
%% Inputs:
    % s: Structure containing [list important structure features]
%% Outputs:
    % s: outputs updated filtered/predicted state mean/covariances w/ 
    %    added landmarks to state mean/covariances
%% Notes:
    % - TODO: Auto generate Jacobian using syms, save jacobian.
    %         Is it necessary? I don't know.
%% References:
    %

% Setup System parameters
J  = 15;      % [kg.m^2]    - Robot Moment of Inertia (Initial Guess)
m  = 20;      % [kg]        - Robot mass
l  = 0.5;     % [m]         - distance: CoG to wheel axis (axle)
a  = 0.5;     % [m]         - distance: wheels to centre axis
bu = 10;      % [N.s/m]     - damping coefficient for linear motor
br = 10;      % [N.m.s/rad] - damping coefficient for rotational motion

    
% State Jacobian Matrix
Jx = s.Delta.*[0, 0, x(4).*cos(x(3)), sin(x(3)), 0;
               0, 0, -x(4).*sin(x(3)), cos(x(3)), 0;
               0, 0, 0, 0, 1;
               0, 0, 0, bu/m, -2*l*x(5);
               0, 0, 0, m*l*x(5)/(J + m*l^2), (m*l*x(4) - br)/(J + m*l^2)];

% Input Jacobian Matrix (used for process noise covariance)
Ju = s.Delta.*[0, 0;
               0, 0;
               0, 0;
               1/m, 1/m;
               a/(J+m*l^2), -a/(J+m*l^2)];