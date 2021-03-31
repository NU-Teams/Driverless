function [fEM] = forwardEulerMethod(opt,x,u)
%% Description:
    % State Transition Model, using specified options (method/model),
    % STM returns estimated states at [t+1]
%% Inputs:
    % opt: structure containing vehicleModel as string
    %      example: opt.vehicleModel = 'robotModel2Wheels'
    % x:   States of vehicle model (optional)
    % u:   Inputs of vehicle model (optional)
%% Outputs:
    % model: returns selected Model
%% Notes:
    % - Switch Case argurment, add to argument when adding new vehicle
    %   modesls
    %   Example:
    %   case 'modelNameHere'
    %       model = modelNameHere()
    % IMPORTANT: Maybe I will return opt with a symbollic model added to
    % the structure? I dunno. Figure out which is faster
%% References:
    % fix up ^^

model = vehicleModel(opt,x,u);
fEM = x + opt.Delta.*model;