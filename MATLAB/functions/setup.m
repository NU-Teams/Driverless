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

addpath('data/datasets')
%% run code
opt.active = true; % Must be true for SLAM solution to run. will go false when simulation has ended.

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

opt.ny = 2;   % number or Landmark states (NEED TO MAKE DYNAMIC)
opt.Delta = 0.1;

% Initialse Variables
opt.nLandmarks = 0;

% setup track, cones for sim
opt = generateLandmarks(opt);
c = load(opt.controllerInput);
if size(c.u,1) > size(c.u,2)
    c.u = c.u';
end

% Setup Ground truth: Measurements, inputs and states
opt.gt.z = []; % measured landmarks is a 2x? vector
opt.gt.u = c.u;
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
for i = 1: length(opt.gt.initPose)
    opt.xp(i) = opt.gt.initPose(i); % assume x, y, psi as states 1:3
    opt.gt.x(i) = opt.gt.initPose(i);
end
opt.Pp = 0.1.*eye(opt.nx + opt.ny*opt.nLandmarks);

% parameters for range/bearing model
opt.par_rb.maxRange = 40;
opt.par_rb.angle = 270;

% Noise amplitude
opt.par_rb.noise_range = 0.1;
opt.par_rb.noise_angle = 0.001;

%% end of function
end