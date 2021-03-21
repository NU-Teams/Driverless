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

%% run code
% Get states, input & measurement sizes
opt.nx = size(vehicleModel(opt),1); % number of states
for u = 1:10
    try
        vehicleModel(opt,ones(opt.nx,1),ones(u,1));
        opt.nu = u; % number of inputs
        break     
    catch
        %
    end
end

opt.ny = 2;   % number or Landmark states

% Initialse Variables
opt.nLandmarks = 0;

% Setup Ground truth: Measurements, inputs and states
opt.gt.z = []; % measured landmarks is a 2x? vector
opt.gt.u = zeros(opt.nu,1);
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
end
opt.Pp = 100.*eye(opt.nx + opt.ny*opt.nLandmarks);


%% end of function
end