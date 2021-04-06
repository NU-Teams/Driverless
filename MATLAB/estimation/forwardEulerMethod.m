function [fEM] = forwardEulerMethod(opt,x,u,TrM)
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

if nargin <= 3
    TrM = eye(size(x,1));
end

model = vehicleModel(opt,x,u);
fEM = x + TrM'*(opt.Delta.*model);

%% end of funciton
end