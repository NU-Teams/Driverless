function [opt] = setup(opt)
%% Description:
    % Sets up matrix and variables for SLAM solution
%% Inputs:
    % opt: SLAM options for models, methods etc.
%% Outputs:
    % opt: SLAMP options with initialised varibles and parameters
%% Notes:
    %
%% References:
    % 
% format longeng
addpath('data/datasets')
%% OPTIONS:
opt.active = true; % Must be true for SLAM solution to run. will go false when simulation has ended.

% Vehicle Model. 
    % Examples: [robotModel2Wheels, simpleBicycleModel, motion_model_velocity, ackermanModel]
opt.vehicleModel = 'ackermanModel';
% Appoximation method []
opt.STM = 'forwardEulerMethod';


% Use Mahalanobis [true] or Euclidean [false]
opt.mahalanobis = true;
if ~opt.mahalanobis
    % Euclidean Distance Params
    opt.min_radius = 5; % radius [m]
end

% Load landmarks (simulation) & initial pose
if opt.isSim
    opt.gt.initPose = [0; 0; 0]; % x, y, psi %% move to setup
end
% Delete options
opt.delete = true;
opt.fileName = 'data/lmmv.bin';
try
    file = fopen(opt.fileName, 'w');
    lmmv = fwrite(file,[],'int8');
    fclose(file);
catch
    disp('no binary file found');
end

%% Find Params
% Get states, input & measurement sizes
opt.nx = size(vehicleModel(opt),1); % number of states
for u = 1:10
    try
        vehicleModel(opt,ones(opt.nx,1),ones(u,1));
        opt.nu = u; % number of inputs
        break;
    catch
        % try again
    end
end

opt.ny = 2;   % number or Landmark states (MAKE DYNAMIC??)
opt.Delta = 0.1;

% Initialse Variables
opt.nLandmarks = 0;

% setup track, cones for sim
if opt.isSim
    opt = generateLandmarks(opt);
    length(opt.gt.landmarks)
    c = load(opt.controllerInput);
    if size(c.u,1) > size(c.u,2)
        c.u = c.u';
    end
    x = zeros(5,length(c.u));
    x(:,1) = [0;0;0;0;0]; % Initial conditions
    figure(2)
    p1 = plot(x(1,1),x(2,1), 'ro');
    hold on
    p2 = plot(sin(x(3,1))/10 + x(1,1), cos(x(3,1))/10+ x(2,1), 'k.');
    p3 = plot(x(1,1),x(2,1), 'b-');
    hold off
    axis equal
    drawnow
    
    for ii = 2:length(c.u)
        x(:,ii) = STM(opt,x(:,ii-1),c.u(:,ii-1));
        set(p1,'xdata',x(1,ii),'ydata',x(2,ii));        
        set(p2,'xdata',sin(x(3,ii))/10 + x(1,ii),'ydata',cos(x(3,ii))/10+ x(2,ii));
        set(p3,'xdata',x(1,1:ii),'ydata',x(2,1:ii));  
        drawnow
        
    end
    
    % Setup Ground truth: Measurements, inputs and states
    opt.gt.z = []; % measured landmarks is a 2x? vector
    opt.gt.u = c.u;
    opt.i.isLidar = true;
    opt.i.isGps = false;
else
    opt.events = getSavedDataset(opt.dataSet);
end


opt.gt.x = zeros(opt.nx+opt.ny*opt.nLandmarks,1);

% Setup observed matrices: as percieved from sensors
opt.obs.z = [];
opt.obs.u = zeros(opt.nu,1);
opt.obs.x = zeros(opt.nx+opt.ny*opt.nLandmarks,1);

% Setup state mean & covariance matrices
opt.xf = zeros(opt.nx + opt.ny*opt.nLandmarks,1);
opt.Pf = zeros(opt.nx + opt.ny*opt.nLandmarks);

% Initialise predicted state mean & covariance matrices
opt.xp = zeros(opt.nx + opt.ny*opt.nLandmarks,1);
if opt.isSim
    try
        for i = 1: length(opt.gt.initPose)
            opt.xp(i) = opt.gt.initPose(i); % assume x, y, psi as states 1:3
            opt.gt.x(i) = opt.gt.initPose(i);
        end
    catch
        opt.xp = [0;0;0]; % assume x, y, psi as states 1:3
        opt.gt.x = [0;0;0];
    end
else % From data
    try % get GPS data to initialise
        c = load('vp_gps.mat');
        opt.xp(1:size(c.gps,2)) = c.gps(1,:);
        opt.xp(3) =   52*pi/180;
    catch % set it to default
        opt.xp = [0;0;0]; % assume x, y, psi as states 1:3
    end
    % For animations
    opt.z.gps = NaN(length(opt.events),2);
    for i = 1:length(opt.events)
        %     opt.z.lid
        if strcmp(opt.events{i,1}, 'gps')
            opt.z.gps(i,:) = opt.events{i,2}(2:end);
        else
            opt.z.gps(i,:) = [0,0];
        end
    end
end
opt.Pp = 10.*eye(opt.nx + opt.ny*opt.nLandmarks);

% % parameters for range/bearing model
% opt.par_rb.maxRange = 20;
% opt.par_rb.angle = 270;

% parameters for range/bearing model (for VP)
opt.par_rb.maxRange = 75;
opt.par_rb.angle = 180;

% Noise amplitude
opt.par_rb.noise_range = 0.1;
opt.par_rb.noise_angle = 0.01;

% Plotting size options
opt.plot.xmin = -5;
opt.plot.xmax = 5;
opt.plot.ymin = -5;
opt.plot.ymax = 5;

% Shortcut to delete
% opt.xp = [-1;-0.3757;0;1.5708;0]; 
% opt.gt.x = [-1;-0.3757;0;1.5708;0];
%% end of function
end