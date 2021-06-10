function [model] = motion_model_velocity(x, u)
%% Description:
    % Velocity Motion model from Probabilistic Robotics
    % Model is purely for SLAM dev purposes, a simple model to assist
    % troubleshooting and expected values.
%% Inputs:
    % x: states of the model, robot pose
        % x = [  x  ]  => robot x coordinate
        %     [  y  ]  => robot y coordinate
        %     [theta]  => robot heading (wrt global reference frame)
    %
    % u: Inputs to propegate the system model.
        % u = [  v  ]  => Forward Velocity
        %     [  w  ]  => Angular Velocity
    %
%% Outputs:
    % model: updated states with input propegation [x,y,theta] wrt u
%% Notes:
    % - inputs cannot be zer0 and should be be constant
    % - ADD TO SOFTWARE DOC
%% References:
    % Probabilistic Robotics, pg 148 of pdf, pg 127 of the book

model = [ -(u(1)/u(2))*sin(-x(3));
          (u(1)/u(2))*cos(-x(3));
           u(2)];

%% end of function
end