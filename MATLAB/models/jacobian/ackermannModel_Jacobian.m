function [Jx, Ju] = ackermannModel_Jacobian(s,x,u,TrM)
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
H = 0.76; % [m] Encoder to centre
L = 2.83; % [m] Length of car (front to rear axle)
a = 3.78; % [m] Length of LiDAR to rear axle
b = 0.5;  % [m] Length of LiDAR to centre of vehice

% Car body forward velocity from encoder data
vc = u(1)/(1-tan(u(2))*H/L);
% Model
jx = s.Delta.*[0,0, vc.*cos(x(3)) + (vc/L)*tan(u(2))*(a*sin(x(3)) + b*cos(x(3)));
               0,0,-vc.*sin(x(3)) + (vc/L)*tan(u(2))*(a*cos(x(3)) - b*sin(x(3)));
               0,0,0];
% Jacobian of the STM
Jx = eye(s.ny*s.nLandmarks+s.nx) + TrM'*jx*TrM;

% Input Jacobian Matrix (used for process noise covariance)
Ju = s.Delta.*[cos(x(3))/(1-tan(u(2))*H/L) - tan(u(2))*(a*sin(x(3)) + b*cos(x(3)))*L/(1-tan(u(2))*H/L), ...
               cos(x(3))*u(1)*sec(u(2))^2/(1-tan(u(2))*H/L) + L\u(1)*sec(u(2))^2/((1-tan(u(2))^2))*(a*sin(x(3)) + b*cos(x(3)));...
               sin(x(3))/(1-tan(u(2))*H/L) - tan(u(2))*(a*cos(x(3)) + b*sin(x(3)))*L/(1-tan(u(2))*H/L), ...
               sin(x(3))*u(1)*sec(u(2))^2/(1-tan(u(2))*H/L) + L\u(1)*sec(u(2))^2/((1-tan(u(2))^2))*(a*cos(x(3)) + b*sin(x(3)));...
               tan(u(2))*L/(1-tan(u(2))*H/L),...
               L\u(1)*sec(u(2))^2/((1-tan(u(2))^2))];
            
 %% end of function       
end