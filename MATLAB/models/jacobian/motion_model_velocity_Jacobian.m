function [Jx, Ju] = motion_model_velocity_Jacobian(s, x, u, TrM)
% The model is from Probabilistic Robotics, pg 148 of pdf, pg 127 of the
% book
% u = [v(t); w(t)]
% x = [x; y; theta]
% Note, inputs cannot be zerp and should be be constant

% State Jacobian Matrix
jx = s.Delta.*[0, 0, -(u(1)/u(2))*cos(-x(3));
               0, 0, -(u(1)/u(2))*sin(-x(3));
               0, 0, 0];
% Jacobian of the STM
Jx = eye(s.ny*s.nLandmarks+s.nx) + TrM'*jx*TrM;

% Input Jacobian Matrix (used for process noise covariance)
Ju = s.Delta.*[-(1/u(2))*sin(x(3)), -(u(1)/u(2)^2)*sin(-x(3));
               (1/u(2))*cos(x(3)),  (u(1)/u(2)^2)*cos(-x(3));
                0,1];

%% end of function
end

