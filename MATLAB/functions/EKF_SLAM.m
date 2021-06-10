function [mu, Sigma] = EKF_SLAM(s, mu, Sigma, u, z)
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

%% SLAM: "Come on and SLAM, if you want to jam" : Space Jame Theme Song

%% Prediction Update:
if strcmp(s.event,'dr')
    [mu,Sigma] = motionPredictionUpdate(s, mu, Sigma, u);
end
    
%% Measurement Update:
if strcmp(s.event,'laser')
    [mu, Sigma] = LiDAR_update(s, mu, Sigma, z);
end
if strcmp(s.event,'gps')
    [mu, Sigma] = gpsUpdate(s, mu, Sigma, z);
end

%% end of function
end