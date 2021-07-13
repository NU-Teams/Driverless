function [par] = parameter()
%Create parameter variable to pass through to functions
%constants for model
m               = 250;             %mass of robot
Ts              = 0.1;
l_r = 1; % distance rear wheels to center of gravity of the car
l_f = 1; % distance front wheels to center of gravity of the car
m   = 250.0;   % mass of the car
%Load data
u               = load('ass2SysidData');
y               = [u.N; u.E; u.psi];
z               = [u.F1; u.F2];
par.y           = y;
par.z           = z;
par.Ts          = Ts;
par.M           = 2000;        %Number of particles in BPF
par.Ms          = 100;         %Number of samples from backward simulator (smoother)
par.theta       = 7;           %Initial parameter guess
par.maxit       = 700;         %Maximum number of EM iterations

% Q and a R parameters
par.Rest           = 1e-3*diag([1 1.5 1.5]);
par.Qest        = 1e-3*diag([1 1 1 1 1]);
par.Q2          = diag([1 1 1 1 1]);

%process model
par.A           = @(x,z,JJ) [x(1,:) + Ts*sin(x(4,:) + x(6,:);
                             x(2,:) + Ts*(x(4,:).*sin(x(3,:)));
                             x(3,:) + Ts*x(5,:);
                             x(4,:) + Ts*((z(1) + z(2) - b_u*x(4,:) - m*l*x(5,:).^2) / m);
                             x(5,:) + Ts*(((m*l*x(4,:) - b_r).*x(5,:) - a*z(1) + a*z(2)) / (JJ + m*l^2));
                             x(6,:) = atan(l_r/(l_f + l_r) * tan(x(5,:)))];

 %measurment model
 par.C          = [1,0,0,0,0,0;...
                   0,1,0,0,0,0;
                   0,0,1,0,0,0
                   0,0,0,0,0,0];
end

