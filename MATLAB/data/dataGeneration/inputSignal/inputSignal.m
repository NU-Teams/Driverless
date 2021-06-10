function [input] = inputSignal(t,u,opt,res)
%% Description:
    % A filter for selecting desired input signal to propegate vehicle
    % model fo simulation. This can be generated data or collected data.
    % Returns input signal at timestep t
%% Inputs:
    % t:
    % u:
    % 
%% Outputs:
    % s: outputs updated filtered/predicted state mean/covariances w/ 
    %    added landmarks to state mean/covariances
%% Notes:
    % - TODO: Auto generate Jacobian using syms, save jacobian.
    %         Is it necessary? I don't know.
%% References:
    %
    
%% Select Model
    switch opt.inputSignal
        case 'input_motion_model_velocity_const'
            input = input_motion_model_velocity_const(t,u);
        case 'input_robotModel2Wheels_loop1'
            input = input_robotModel2Wheels_loop1(t,u,res);
        otherwise
            % insert fail safe or error code?
            % https://youtu.be/unv3GQidxEs
    end
    end