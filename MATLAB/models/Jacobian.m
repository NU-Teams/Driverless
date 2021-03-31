function [J, V] = Jacobian(opt,x,u)
%% Description:
    % Returns Jacobian of desired model from a look up table (pre calculated).
%% Inputs:
    % s: Structure containing [list important structure features]
%% Outputs:
    % s: outputs updated filtered/predicted state mean/covariances w/ 
    %    added landmarks to state mean/covariances
%% Notes:
    % - TODO: Auto generate Jacobian using syms, save jacobian.
    %         Is it necessary? I don't know.
    % Can this just be included in the measurement model?
%% References:
    %

%% Select Model
switch opt.vehicleModel
    case 'robotModel2Wheels'
        [J, V] = robotModel2Wheels_Jacobian(opt,x,u);
    otherwise
        % insert fail safe or error code?
        % https://youtu.be/unv3GQidxEs
end

%% end of function
end