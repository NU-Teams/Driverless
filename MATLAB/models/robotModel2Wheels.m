function [model] = robotModel2Wheels(x,u)
%% [stm, G, V] = dynamicModel_Example(x,u,par)
% Description:  
    % Outputs vehicle dynamics, and Jacobians of wheeled robot system
% Inputs:
    % x: System state:  x(1) = N(t) World fixed north position in metres
%           at time t:  x(2) = E(t) World fixed East Position in metres
%                       x(3) = phi(t) Yaw angle (rad/s) about the down axis
%                       x(4) = u(t) body fixed forward velocity
%                       x(5) = r(t) body fixed yaw rate
%
    % u: System Inputs: u(1) = Force on the robot from the right wheel
%                       u(2) = Force on the robot from the left wheel
%
%   % par: parameters to describe robot.
%
% Outputs:
        % stm: State transition model, predicts model params at next time
%              step
        % G: Jacobian of the system wrt states
        % V: Jacobian of the system wrt inputs
% NOTE: Robot model sourced from lab 9 of MCHA6100 lab by Adrian Wills, 
      % model is used as an example to verify SLAM implementation. 


% Pull params for models
% Setup System parameters
J  = 15;      % [kg.m^2]    - Robot Moment of Inertia (Initial Guess)
m  = 20;      % [kg]        - Robot mass
l  = 0.5;     % [m]         - distance: CoG to wheel axis (axle)
a  = 0.5;     % [m]         - distance: wheels to centre axis
bu = 10;      % [N.s/m]     - damping coefficient for linear motor
br = 10;      % [N.m.s/rad] - damping coefficient for rotational motion



% Model
model   =      [(x(4,:).*sin(x(3,:)));
                (x(4,:).*cos(x(3,:)));
                (x(5,:));
                ((u(1,:) + u(2,:) - bu*x(4,:) - m*l*x(5,:).^2))/m;
                (((m*l*x(4,:) - br).*x(5,:) - a*u(2,:) + a*u(1,:)))/(J+m*l^2)];

%% end of function 
end