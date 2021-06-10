function data = getSavedDataset(ds)
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

%%
%% Select Model
    switch ds
        case 'victoriaPark'
            data = collectData();
        otherwise
            % insert fail safe or error code?
            % https://youtu.be/unv3GQidxEs
    end