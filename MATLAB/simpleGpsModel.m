function [zh,e,J] = simpleGpsModel(z, x)
%% Description:  
    % Outputs measurement estimation for simple range bearing model
%% Inputs:
    % x: System state:  x(1) = N(t) World fixed north position in metres
%           at time t:  x(2) = E(t) World fixed East Position in metres
%                       x(3) = phi(t) Yaw angle (rad/s) about the down axis
%                       x(4) = u(t) body fixed forward velocity
%                       x(5) = r(t) body fixed yaw rate
%
    % L:
%
%% Outputs:
    % zh: 
%% NOTE: 
    %
%% References:
    %

%% Lets see what I reckon

% Predicted measurement
zh = [x(1), x(2)];
e = z'-zh';

% J: Jacobian of measurement model wrt states (3 state model)
J = [1, 0, 0;
     0, 1, 0];
J = [J zeros(size(z,2),size(x,1)-3)];


%% end of function
end
