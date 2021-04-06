function [input] = inputSignal(t,u,opt,res)

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