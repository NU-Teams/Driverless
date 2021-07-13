function [theta,Qest,Rest] = mstep(xs,par,theta)

% -Q(theta) = -\int log p(X,Y | theta)  p(X | Y, \theta_k) dX
%
% log p(X, Y | theta) = \sum_{t=1}^{N-1} log p(x_{t+1} | x_t) + \sum_{t=1}^{N} log p(y_t | x_t)

y           = states;        %data
model           = @(z) ForwardEuler( z(3:7), z(1:2), @continuousDynamics,... 
                integrator_stepsize);        %Process Model
 %measurment model
C          = [1,0,0,0,0,0;...
                   0,1,0,0,0,0;
                   0,0,1,0,0,0
                   0,0,0,0,0,0];
z           = u;        %inputs
Ms          = 700;       %Number of samples from backward simulator (smoother)
N           = length(y);
%Theta is coefeficents of Q matrix

options     = optimoptions('fminunc','display','none');

theta       = fminunc(@(x) -Qhat(x), theta, options);


function qh = Qhat(th)
    %Use Monte-Carlo integration to form  -Qhat            
    qh = 0;
    rh = 0;
    for i=1:Ms
        for t=1:N-1
            %State component
            xx = xs(:,i,t);
            ex = xs(:,i,t+1) - model;
            qh = qh + ex*ex';
            
            ey = y(:,t) - C*xx;
            rh = rh + ey*ey';

        end
    end
    
    Qest = ((1/(N-1))*(1/Ms)*qh);
    Rest = ((1/(N-1))*(1/Ms)*rh);
    qh   = -0.5*log(det((Qest)));


end
end

function [xDot] = continuousDynamics(x,u)
% state x = [xPos,yPos,v,theta,delta], input u = [F, phi]
    
    % set physical constants
    l_r = 0.5; % distance rear wheels to center of gravity of the car
    l_f = 0.5; % distance front wheels to center of gravity of the car
    m   = 250.0;   % mass of the car
    
    % set parameters
    beta = atan(l_r/(l_f + l_r) * tan(x(5)));
    
    % calculate dx/dt
    xDot = [x(3) * sin(x(4) + beta);   % dxPos/dt = v*cos(theta+beta)
            x(3) * cos(x(4) + beta);   % dyPos/dt = v*cos(theta+beta)
            u(1)/m;                    % dv/dt = F/m
            x(3)/l_r * sin(beta);      % dtheta/dt = v/l_r*sin(beta)
            u(2)];                     % ddelta/dt = phi

end
