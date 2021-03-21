close all
clear all
clc

addpath('data','functions','models')

disp('Running EKF SLAM Example: SLAM with unknown correspondence & limited field of view')
%% OPTIONS:
 % This is an ONLINE SLAM solution, but data may be saved for debugging
 % purposes.
SLAM.fullSLAM = true;

% Include noise to measurements
SLAM.addNoise = false;

% Animation options
SLAM.animate = true;
SLAM.saveVideo = false; % animate must also be true
SLAM.videoFileName = '../videos/EKFSLAM_limitedFOV_euclideanDistance_Noise_DeletingLandmarks1.avi';

%% Load landmarks (simulation) & initial pose
SLAM.gt.L = load('landmarks_loop_med_LM.mat');
SLAM.gt.initPose = [20; 30; 0]; % x, y, psi

%% Declare Models
% Vehicle Model. 
    % Examples: [robotModel2Wheels, simpleBicycleModel]
SLAM.vehicleModel = 'robotModel2Wheels';

SLAM = setup(SLAM);

for t = 1:N
    %% Generate Data
    % Generate Ground Truth & added noise
        % Include Limited FOV
    
    % vvvvvvvvvv SLAM Function vvvvvvvvvvvvvvvvvvvv
   SLAM = EKF_SLAM(SLAM);
   
    %% Measurement update
    
    
    %% Prediction update
    
    
    % ^^^^^^^^^^^ SLAM Function ^^^^^^^^^^^^^^^^^^
    %% Store, animate etc
    
    
end


%% Overview
% "Come on and SLAM, if you want to jam" : Space Jame Theme Song
%
% Starting with an EKF SLAM utilising landmark mapping.
% 

%% Resources
% 

%% The Plan
% 1: Localisation from simulated data. (Simples. From LiDAR measurement)
    % Create a MAP (Is this just a point or do I declare a landmark??)
    % Create simulated LiDAR data, input data from scanNE (basically adds noise)
    % Input a trajectory through a map, collect simulated data.
    % Localise myself
    
% 2: Add noise to the above

% 3: Add landmarks to the states

% 4: Start Mapping w/ landmarks

% 5: ?

% 6: Profit
