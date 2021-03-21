function [model] = vehicleModel(opt,x,u)
%% Description:
    % Returns selected Process Model
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
    % 

%% Check input arguments
if nargin <= 2
    x = ones(40,1);
    u = ones(10,1);
end
    %% Select Model
    switch opt.vehicleModel
        case 'robotModel2Wheels'
            model = robotModel2Wheels(x,u);
        otherwise
            % insert fail safe or error code?
    end
   
%% end of function
end