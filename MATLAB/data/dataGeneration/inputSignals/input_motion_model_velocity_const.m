function u = input_motion_model_velocity_const(t,u)
u = zeros(size(u));
u(1) = 5;
u(2) = 30*pi/180;
end