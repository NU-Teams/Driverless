function [Jx, Ju] = bicycleModel_Jacobian(s,x,u,TrM)
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
m  = 250;    % [kg]   - Car mass
lr  = 0.5;   % [m]    - distance: CoG to rear wheel axis
lf  = 0.5;   % [m]    - distance: CoG to front wheel axis

K = lr/(lr+lf);
Beta = atan(K*tan(x(5)));

jx = s.Delta.*[0, 0, x(4)*cos(x(3) + Beta),               sin(x(3) + Beta),                                                (K*x(4)*cos(x(3) + Beta)*(tan(x(5))^2 + 1))/(K^2*tan(x(5))^2 + 1);
               0, 0, -x(4)*sin(x(3) + Beta),               cos(x(3) + Beta),                                               -(K*x(4)*sin(x(3) + Beta)*(tan(x(5))^2 + 1))/(K^2*tan(x(5))^2 + 1);
               0, 0,                             0, (2*K*tan(x(5)))/(K^2*tan(x(5))^2 + 1)^(1/2), (2*K*x(4)*(tan(x(5))^2 + 1))/(K^2*tan(x(5))^2 + 1)^(1/2) - (2*K^3*x(4)*tan(x(5))^2*(tan(x(5))^2 + 1))/(K^2*tan(x(5))^2 + 1)^(3/2);
               0, 0,                             0,                                       0,                                                                                                                   0;
               0, 0,                             0,                                       0,                                                                                                                   0];
           
% Jacobian of the STM
Jx = eye(s.ny*s.nLandmarks+s.nx) + TrM'*jx*TrM;

% Input Jacobian Matrix (used for process noise covariance)
Ju = s.Delta.*[ 0,   0;
                0,   0;
                0,   0;
                1/m, 0;
                0,   1];
            
 %% end of function       
end