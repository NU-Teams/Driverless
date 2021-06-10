close all
clear all
clc
try
    cd('../../MATLAB') % To Root Folder
catch
    cd(fileparts(which('simulation_SLAM.m'))); % reset to current folder
    cd('../../MATLAB')
end
addpath(genpath('../MATLAB')); % Add ALL sub folders

disp('Running EKF SLAM Example: SLAM with unknown correspondence & limited field of view');

% rng(1) % Set seed for development & debugging
%% SIMULATION OPTIONS:
SLAM.isSim = false; % True:  Generates Landmarks & measurements from Track & input data
                    % False: Uses selected dataset.

if SLAM.isSim
    % Track Option (track xy coordinates) & Data associated w/ track
%     SLAM.track = 'data_motion_model_velocity.mat'; 
%     SLAM.controllerInput = 'data_motion_model_velocity.mat';
%     SLAM.track = 'data_robotModel2Wheels_loop1.mat';
%     SLAM.controllerInput = 'data_robotModel2Wheels_loop1.mat'; 
    SLAM.track = 'data_handlingTrack.mat';
    SLAM.controllerInput = 'data_handlingTrack.mat'; 
    SLAM.dataSet = 'none';
else
    % Currently the Victoria Park Dataset. This will Process the data for
    % SLAM
    SLAM.dataSet = 'victoriaPark';
end

 % Save data for analysis / debugging (written not yet tested)
SLAM.saveData = false;

% Include noise to measurements
if SLAM.isSim
    SLAM.addNoise = true;
end

% Animation options
SLAM.visualise = false;
SLAM.animate = true;
SLAM.saveVideo = true; % animate must also be true
SLAM.videoFileName = 'videos/EKFSLAM_VP_LIDAR_GPS_Odo.avi';

% Setup Simulation
SLAM = setup(SLAM);

% Initialise States
xp = SLAM.xp;
Pp = 10.*eye(SLAM.nx+SLAM.ny*SLAM.nLandmarks);
xf = 0; Pf = 0;
p = [];
e = 1;
SLAM.traj = NaN(2,length(SLAM.events));
SLAM.LidPlot = [0;1];
SLAM.LidState = [0;0;0];
SLAM.uPlot = [0;0];
SLAM.time = 0;
frametime = 1;
% Iniitialise previous odomoetry t (for first iterative)
u = 0;
z = 0;
prev_drt = -1; 
% tic
while SLAM.active
%     if t == 213
%         disp('STOP')
%     end
    %% Generate/Get Data
    if SLAM.isSim
        % Generate Ground Truth States
        if e ~= 1        
            [x] = STM(SLAM,x,SLAM.gt.u(:,e));
        else
            x = xp;
            if prev_drt < 0 && strcmp(SLAM.events{e,1}, 'dr')
                prev_drt = SLAM.events{e,2}(1);
            end
        end
        % Generate Measurements, ground truth & simulated data
        [zgt, zn] = getMeasurements_rangeBearing(x,SLAM.gt.landmarks,SLAM);
        if SLAM.addNoise
            % Add noise to the input signal
            u = SLAM.gt.u(:,e) + 0.01*randn(SLAM.nu,1);
        else
            % Use Ground truth values
            zn = zgt;    
            u = SLAM.gt.u(:,e);
        end
        z.lid = zn;
        
    else % Use real data
        % Check Event
        if strcmp(SLAM.events{e,1}, 'laser')
            z = getLandmarks(SLAM.events{e,2}(2:end));
            SLAM.event = 'laser';
            try
                if max(z(1,:)) > 75
                    disp(max(z(1,:)))
                end
                SLAM.par_rb.maxRange = 1.05*max(z(1,:));
            catch
                SLAM.event = 'none';
            end
        elseif strcmp(SLAM.events{e,1}, 'gps')
            z = SLAM.events{e,2}(2:end);
            SLAM.event = 'gps';
        elseif strcmp(SLAM.events{e,1}, 'dr')
            u = SLAM.events{e,2}(2:end);
            SLAM.event = 'dr';
            SLAM.uPlot = u;
            SLAM.time = SLAM.time + 1;
            if prev_drt < 0
                prev_drt = SLAM.events{e,2}(1);
            end
            SLAM.Delta = (SLAM.events{e,2}(1) - prev_drt)/1000;
            if SLAM.Delta > 0.025
                print(SLAM.Delta)
            end
            prev_drt = SLAM.events{e,2}(1);
        end
       
    end
    
    
    %% SLAM
    if e == 1
        xf = xp;
        Pf = Pp;
        SLAM.traj(:,e) = xf(1:2);
    else
        tic
            [xf,Pf] = EKF_SLAM(SLAM, xf, Pf, u, z);
        toc
        SLAM.traj(:,e) = xf(1:2);
        if strcmp(SLAM.events{e,1}, 'laser')
            SLAM.LidPlot = z;
            SLAM.LidState = xf(1:3);
        end
        
    end
    
    if SLAM.animate %&& (e == 2000 || e ==1 || e == length(SLAM.events) || e ==10000 || e ==30000)
        [p, SLAM] = plotMap(xf, Pf, z, e, p, SLAM);        
        if SLAM.saveVideo
            if strcmp(SLAM.events{e,1}, 'laser')
                F(frametime) = getframe(gcf);
                frametime = frametime + 1;
            end
        end

    end 
    
    %% Store, animate etc
    if SLAM.saveData
        %% Create vectors
        if e == 1
           % States Inputs & Measurements (static / no change in size)
           SLAM.data.x = [];
           SLAM.data.u = [];
           SLAM.data.z = []; % I lied this is dynamic
           
           % Mean & Covariance Matrices (dynamic / changing in size)
           SLAM.data.mu = [];
           SLAM.data.P  = [];
        end
        %% Resize Matrices
            % Resize Z ?
            if size(SLAM.obs.z,1) > size(SLAM.data.z,1)
                size_difference = size(SLAM.obs.z,1) - size(SLAM.data.z,1);
                SLAM.data.z = [SLAM.data.z; NaN(size_difference,size(SLAM.data.z,2))];
            end
            % Resize mu ?
            if size(xf,1) > size(SLAM.data.mu,1)
                size_difference = size(xf,1) - size(SLAM.data.mu,1);
                SLAM.data.mu = [SLAM.data.mu; NaN(size_difference,size(SLAM.data.mu,2))];
            end

            % Resize P ?
            if size(Pf,1) > size(SLAM.data.P,1)
                size_difference = size(Pf,1) - size(SLAM.data.P,1);
                SLAM.data.P = [SLAM.data.P, ...
                               NaN(size(SLAM.data.P,1),size_difference,size(SLAM.data.P,3));...
                               NaN(size_difference,size(SLAM.data.P,2),size(SLAM.data.P,3)), ...
                               NaN(size_difference,size_difference,size(SLAM.data.P,3))];
            end
            
        %% Add data to exisiting fields (ew, inefficient, I know)
        SLAM.data.x = [SLAM.data.x, SLAM.obs.x];
        SLAM.data.u = [SLAM.data.u, SLAM.obs.u];
        SLAM.data.z = [SLAM.data.z, SLAM.obs.z];
        SLAM.data.mu = [SLAM.data.mu, xf];
        
        % FIXME: I dunno if this is will work vvv Its a 3D vector
        SLAM.data.P  = [SLAM.data.P, Pf];
        
        % Store 
    end
 
    % increase timestep for next iteration
    e = e + 1;
    if SLAM.isSim
        if e > size(SLAM.gt.u,2)
            SLAM.active = false;
        end
    else
        if e > length(SLAM.events)
            SLAM.active = false;
        end
    end
end
% toc
if SLAM.saveVideo
    writerObj = VideoWriter(SLAM.videoFileName);
    writerObj.FrameRate = 20; % sets the fps

    % open the video writer
    open(writerObj);

    % write the frames to the video
    for f = 1:length(F)
        % convert the image to a frame
        frame = F(f);
        writeVideo(writerObj, frame)
    end
    close(writerObj);
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
