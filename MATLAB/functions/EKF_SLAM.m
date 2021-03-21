function [s] = EKF_SLAM(s)
%% Description:
    % EKF SLAM Algorithm, predicts and filters state means & covariances.
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

%% Algorithm
