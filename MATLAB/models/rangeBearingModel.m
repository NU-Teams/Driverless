function [zh,e,J] = rangeBearingModel(z, x, L)
%% Description:  
    % Outputs measurement estimation for simple range bearing model
%% Inputs:
    % x: System state:  x(1) = N(t) World fixed north position in metres
%           at time t:  x(2) = E(t) World fixed East Position in metres
%                       x(3) = phi(t) Yaw angle (rad/s) about the down axis
%                       x(4) = u(t) body fixed forward velocity
%                       x(5) = r(t) body fixed yaw rate
%
    % L:
%
%% Outputs:
    % zh: 
%% NOTE: 
    %
%% References:
    %

%% Lets see what I reckon
tol = 45;

dx = L(1) - x(1);
dy = L(2) - x(2);
q = dx^2 + dy^2;

% Predicted measurement to landmark
zh = [sqrt(q);
      wrapTo2Pi(atan2(dx,dy) - x(3))];
e = z-zh;

% Error fixing logic (catch errors around limits [equivelent points])
if (z(2) > (360-tol)*pi/180 && zh(2) < (tol)*pi/180) 
    e(2) = e(2)-2*pi;
elseif (zh(2) > (360-tol)*pi/180 && z(2) < (tol)*pi/180)
    e(2) = e(2)+2*pi;
end

% J: Jacobian of measurement model wrt states ()
J = (1/q).*[-zh(1)*dx, -zh(1)*dy, 0;
                   dy,       -dx, -q];

J = [J zeros(size(z,1),size(x,1)-3)];
% J (updated): Jacobian of measurement model wrt landmarks
J = [J,-J(1:2,1:2)]; 

%% end of function
end
