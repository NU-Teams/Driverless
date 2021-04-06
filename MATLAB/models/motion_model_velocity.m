function [model] = motion_model_velocity(x, u)
% The model is from Probabilistic Robotics, pg 148 of pdf, pg 127 of the
% book
% u = [v(t); w(t)]
% x = [x; y; theta]
% Note, inputs cannot be zer0 and should be be constant

model = [-(u(1)/u(2))*sin(-x(3));
          (u(1)/u(2))*cos(-x(3));
           u(2)];

%% end of function
end