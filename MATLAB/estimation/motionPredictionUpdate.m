function [xb,Pb] = motionPredictionUpdate(s, x, P, u)
%% Description:
    % Computes the motion prediciton update of the vehicle
%% Inputs:
    % s: Structure containing [list important structure features]
%% Outputs:
    % s: outputs updated filtered/predicted state mean/covariances w/ 
    %    added landmarks to state mean/covariances
%% Notes:
    % - 
%% References:
    % EKF SLAM algorithm with ML corresopences Table 10.2 pg 322,
    % Probabilistic Robotics, 2006, [Thrun et al]
    %
s.nLandmarks = length(x((s.nx+1):end))/2;
alpha = 1e-2.*[3 2 3 4]; % robot-dependent motion noise parameters
% Fx: Projection Matrix for State Update
Fx = [eye(s.nx),zeros(s.nx,s.ny*s.nLandmarks)];

% G: Jacobian of process model wrt states, V: Jacobian of process model wrt inputs
[G, V] = Jacobian(s,x,u,Fx); 

% Define R & M?? KRAMER
M = [(alpha(1)*abs(u(1)) + alpha(2)*abs(u(2)))^2, 0;
      0, (alpha(3)*abs(u(1)) + alpha(4)*abs(u(2)))^2];
R = V*M*V';
R = [0.05^2,0,0;0,0.05^2,0;0,0,(0.5*pi/180)^2];

% Mean & Covariance Prediction
xb = STM(s,x,u,Fx);
Pb = G*P*G' + Fx'*R*Fx;
    
%% end of function
end