clear; clc; close all;

%% Problem dimensions
    model.N = 30;            % horizon length
    model.nvar = 7;          % number of variables
    model.neq  = 5;          % number of equality constraints
    model.nh = 1;            % number of inequality constraint functions
    model.npar = 4;          % number of runtime parameters
    
%     model.objective = @LSobj;
    model.LSobjective = @LSobj;
%     model.LSobjectiveN = @LSobjN; % increased costs for the last stage

%     model.objective = @(z) 100*abs(z(3)-0) + 100*abs(z(4)-4) + ...
%     0.1*z(1)^2 + 0.01*z(2)^2;
    
    integrator_stepsize = 0.1;
    model.eq = @(z) RK4( z(3:7), z(1:2), @continuousDynamics,...
        integrator_stepsize);
    
    % Indices on LHS of dynamical constraint - for efficiency reasons, make
    % sure the matrix E has structure [0 I] where I is the identity matrix.
    model.E = [zeros(5,2), eye(5)];
%% constraints
    % upper/lower variable bounds lb <= z <= ub
    %            inputs             |             states
    %             F      phi           xPos  yPos     v    theta    delta
    model.lb = [ -250.,  -deg2rad(90),   -inf,  -inf,   0.,  -inf,   -deg2rad(50)]; % -0.48*pi
    model.ub = [ +250.,  deg2rad(90),     inf,   inf,   30,  +inf,    deg2rad(50)];
%         %             F    phi              xPos  yPos   v    theta    delta   
%     model.lb = [ -250.,  -deg2rad(90),   -inf,  -inf,   0.,  -inf,   -deg2rad(50)]; % -0.48*pi
%     model.ub = [ +250.,  deg2rad(90),     inf,   inf,   20,  +inf,    deg2rad(50)];

    % General (differentiable) nonlinear inequalities hl <= h(z,p) <= hu
    model.ineq = @(z,p)  [(z(3)+p(1))^2 + (z(4)+p(2))^2];

    % Upper/lower bounds for inequalities
    model.hu = [inf];
    model.hl = [0];
    %% Initial conditions
    % Initial condition on vehicle states
%     model.xinit = [-7, 0.5, 0., deg2rad(90), 0.]';   
    model.xinit = [60, 146, 0., deg2rad(180), 0.]';  % amz track 
    % v=0 (standstill), heading angle=90, steering angle=0
    model.xinitidx = 3:7; % use this to specify on which variables initial 
    % conditions are imposed
    %% Define solver options
    codeoptions = getOptions('FORCESNLPsolver');
    codeoptions.maxit = 10000;    % Maximum number of iterations was origionally 400
    codeoptions.printlevel = 2; % Use printlevel = 2 to print progress (but 
    % not for timings)
    codeoptions.optlevel = 0;   % 0: no optimization, 1: optimize for size,
    % 2: optimize for speed, 3: optimize for size & speed
    codeoptions.cleanup = false;
    codeoptions.timing = 1;
    options.printlevel = 0;
    options.nlp.hessian_approximation = 'bfgs';
    options.solvemethod = 'SQP_NLP'; % choose the solver method Sequential
    options.maxqps = 5;  % maximum number of quadratic problems to be 
    % solved during one solver call
    options.sqp_nlp.reg_hessian = 0.1;  % increase this parameter if exitflag=-8

    % change this to your server or leave uncommented for using the 
    % standard embotech server at https://www.embotech.com/codegen
    % codeoptions.server = 'https://yourforcesserver.com:1234';
    codeoptions.nlp.ad_tool = 'casadi-351';
    %% Generate FORCESPRO solver
    FORCES_NLP(model, codeoptions);
%% Simulate and call solver
simLength = 1000; % simulate 80 = 8sec

% Variables for storing simulation data
x = zeros(5,simLength+1); % states
x(:,1) = model.xinit;
u = zeros(2,simLength);   % inputs

% Set initial guess to start solver from
x0i = zeros(model.nvar,1);
problem.x0 = repmat(x0i,model.N,1);
problem.reinitialize = 1; % initialize first simulation step of solver 
% always with problem.x0 

% Create 2D points on ellipse which the car is supposed to follow


% numPoints = 80;
% pathPoints = calcPointsOnEllipse(numPoints);
scale = 0.01;

% data1 = load('xpos.mat');
% data2 = load('ypos.mat');
% 
% pathPoints = [data1.xpos;data2.ypos];
% 
% pathPoints = scale*pathPoints(:,2:3:end);

% pp = spline(data1.xpos,data2.ypos);
% pathPoints = ppval(pp,[0:1:50;[0:1:50]]);

% pp = spline([0 1],[0,1]);
% pathPoints = ppval(pp,[linspace(0,100,10*model.N);linspace(0,100,10*model.N)]);

load track2.mat
pathPoints = 100*track2.center;

states = [];
% Create empty plot
startPred = reshape(problem.x0,7,model.N); % first prediction corresponds 
% to initial guess

% Create plot with initial values
createPlot(x,u,startPred,simLength,model,pathPoints,scale);

for k = 1:simLength
    
    % Set initial condition
    problem.xinit = x(:,k);
    
    % Set runtime parameters (here, the next N points on the path)
    % Set runtime parameters
%     params = [param1; param2];
%     problem.all_parameters = repmat(params,model.N,1);
    % Definition of extractNextPathPoints further below in this file
    nextPathPoints = extractNextPathPoints(pathPoints, x(1:2,k), model.N); 
    problem.all_parameters = reshape([nextPathPoints;nextPathPoints],4*model.N,1);
        
    % Solve optimization problem
    [output,exitflag,info] = FORCESNLPsolver(problem);
    
    % Make sure the solver has exited properly
    if( exitflag == 1 )
        fprintf('\nFORCES took %d iterations and ',info.it); 
        fprintf('%f seconds to solve the problem.\n',info.solvetime);
    else
        error('Some problem in solver');
    end
    
    % Apply optimized input u to system and save simulation data
    u(:,k) = output.x01(1:2);
    x(:,k+1) = model.eq( [u(:,k);x(:,k)] )';
    
    % Extract output for prediction plots
    predictedZ = zeros(7,model.N);
    for i = 1:model.N  
        if i < 10 && model.N >= 10
            predictedZ(:,i) = output.(append('x0',int2str(i)));
        else 
            predictedZ(:,i) = output.(append('x',int2str(i)));
        end
    end
    predictedU = predictedZ(1:2,:);
    predictedX = predictedZ(3:7,:);
    
    if k == 1
        % from now on, the solver should be initialized with the solution
        % of its last call
        problem.reinitialize = 0;
    end
    
    % plot current progress
    updatePlot(x,u,predictedX,predictedU,model,k, pathPoints, nextPathPoints)
    F(k) = getframe(gcf); %Frame for video  
    
end

gcf();
subplot(5,2,[1,3,5,7,9])
oldPosPred = findobj(gca,'Color','green');
delete(oldPosPred);
legend('desired trajectory','initial position','car trajectory',...
        'Location','southeast');
    
function [xDot] = continuousDynamics(x,u)
% state x = [xPos,yPos,v,theta,delta], input u = [F, phi]
    
    % set physical constants
    l_r = 0.5; % distance rear wheels to center of gravity of the car
    l_f = 0.5; % distance front wheels to center of gravity of the car
    m = 250.0;   % mass of the car
    
    % set parameters
    beta = atan(l_r/(l_f + l_r) * tan(x(5)));
    
    % calculate dx/dt
    xDot = [x(3) * cos(x(4) + beta); % dxPos/dt = v*cos(theta+beta)
             x(3) * sin(x(4) + beta); % dyPos/dt = v*cos(theta+beta)
             u(1)/m;                  % dv/dt = F/m
             x(3)/l_r * sin(beta);    % dtheta/dt = v/l_r*sin(beta)
             u(2)];                   % ddelta/dt = phi
end

    
    function [r] = LSobj(z,currentTarget)
% least square costs on deviating from the path and on the inputs F and phi
% z = [F,phi,xPos,yPos,v,theta,delta]
% currentTarget = point on path that is to be headed for

    r = [sqrt(2000.0)*(z(3)-currentTarget(1)); % costs for deviating from 
        % the path in x-direction
        sqrt(2000.0)*(z(4)-currentTarget(2));  % costs for deviating from 
        % the path in y-direction
     
        sqrt(0.2)*z(1);                        % penalty on input F
        sqrt(2)*z(2)];                       % penalty on input phi
    end
function [r] = LSobjN(z,currentTarget)
% Increased least square costs for last stage on deviating from the path 
% and on the inputs F and phi
% z = [F,phi,xPos,yPos,v,theta,delta]
% currentTarget = point on path that is to be headed for

    r = [sqrt(0.0001)*(z(3)-currentTarget(1)); % costs for deviating from 
        % the path in x-direction
        sqrt(0.0001)*(z(4)-currentTarget(2));  % costs for deviating from 
        % the path in y-direction
        sqrt(0.01)*z(1);                        % penalty on input F
        sqrt(1000)*z(2)];                       % penalty on input phi
end

    function [idx] = findClosestPoint(points, refPoint)
% Find the index of closest point in points from the current car position
% points = array of points on path
% refPoint = current car position

     squaredDiff = (points-repmat(refPoint, 1, size(points,2))).^2;
     squaredDist = squaredDiff(1,:) + squaredDiff(2,:);
     [~,idx] = min(squaredDist);
end

function [nextPathPoints,idx] = extractNextPathPoints(pathPoints, pos, N)
% Extract the next N points on the path for the next N stages starting from
% the current car position pos

    idx = findClosestPoint(pathPoints, pos);
    numPoints = size(pathPoints,2);
    numEllipses = ceil((idx+N)/numPoints);
    pathPoints = repmat(pathPoints,1,numEllipses);
    nextPathPoints = pathPoints(:,idx:idx+N-1);   
end

function createPlot(x,u,startPred,simLength,model,pathPoints,scale)
% Creates a plot and adds the initial data provided by the arguments 

    figure('units','normalized','outerposition',[0 0 1 1]); 
    clf;
    
    % Plot position
    subplot(5,2,[1,3,5,7,9])
    plot(pathPoints(1,:), pathPoints(2,:),'.','Color', [0.5, 0.5, 0.5],'MarkerSize', 100); hold on;
    plot(pathPoints(1,:), pathPoints(2,:),'--w');
    plot(model.xinit(1),model.xinit(2),'bx','LineWidth',3); 
%     legend('desired trajectory','initial position','Location','southeast')
    plot(x(1,1),x(2,1),'b-', 'DisplayName','car trajectory'); 
    plot(startPred(3,:),startPred(4,:),'-g', 'DisplayName',...
        'desired car traj.');
    title('position'); 
    xlabel('x-coordinate'); ylabel('y-coordinate');
    xlim([min(pathPoints(1,:))-scale/1.5 max(pathPoints(1,:))+scale/1.5])
    ylim([min(pathPoints(2,:))-scale/1.5 max(pathPoints(2,:))+scale/1.5])
%     xlim([-3.,3.]); ylim([-3,3]);
 
    grid on
    % axis equal;
    
    % Plot velocity
    subplot(5,2,2); 
    title('Velocity'); grid on; hold on;
    plot(0.0,x(3,1));
    plot(1:model.N, startPred(5,:),'g-');
    plot([1 simLength], [model.ub(5) model.ub(5)]', 'r:'); 
    plot([1 simLength], [model.lb(5) model.lb(5)]', 'r:');
    xlim([1,simLength]);
    
    % Plot heading angle
    subplot(5,2,4); 
    title('Heading angle'); grid on; hold on; 
    plot(0.0,rad2deg(x(4,1)));
    plot(1:model.N, rad2deg(startPred(6,:)),'g-');
    plot([1 simLength], rad2deg([model.ub(6) model.ub(6)])', 'r:');
    plot([1 simLength], rad2deg([model.lb(6) model.lb(6)])', 'r:');
    xlim([1,simLength]); ylim([-200,200]);
    
    % Plot steering angle
    subplot(5,2,6); 
    title('Steering angle'); grid on; hold on; 
    plot(0.0,rad2deg(x(5,1)));
    plot(1:model.N, rad2deg(startPred(7,:)),'g-');
    plot([1 simLength], rad2deg([model.ub(7) model.ub(7)])', 'r:');
    plot([1 simLength], rad2deg([model.lb(7) model.lb(7)])', 'r:');
    xlim([1,simLength]);
    
    % Plot acceleration force
    subplot(5,2,8);
    title('Acceleration force'); grid on; hold on;
    stairs(0.0,u(1,1));
    stairs(1:model.N, startPred(1,:),'g-');
    plot([1 simLength], [model.ub(1) model.ub(1)]', 'r:');
    plot([1 simLength], [model.lb(1) model.lb(1)]', 'r:');
    xlim([1,simLength]);
    % Plot steering rate
    subplot(5,2,10); 
    stairs(0.0,u(2,1));
    stairs(1:model.N, startPred(2,:),'g-');
    title('Steering rate'); grid on; hold on; 
    plot([1 simLength], rad2deg([model.ub(2) model.ub(2)])', 'r:');
    plot([1 simLength], rad2deg([model.lb(2) model.lb(2)])', 'r:');
    xlim([1,simLength]);
end

function updatePlot(x,u,predictedX,predictedU,model,k, pathPoints, nextPathPoints)
% Deletes old data sets in the current plot and adds the new data sets 
% given by the arguments x, u and predicted_z to the plot.
% x: matrix consisting of a set of state column vectors
% u: matrix consisting of a set of input column vectors
% predictedX: predictions for the next N state vectors
% predictedU: predictions for the next N input vectors
% model: model struct required for the code generation of FORCES Pro
% k: simulation step

    gcf();

    % plot position
    subplot(5,2,[1,3,5,7,9])
    oldPos = findobj(gca,'Color','blue');
    delete(oldPos);
    oldPosPred = findobj(gca,'Color','green');
    delete(oldPosPred);
    oldRef = findobj(gca,'Color','red');
    delete(oldRef);
    plot(model.xinit(1),model.xinit(2),'bx','LineWidth',3); 
    plot(x(1,1:k),x(2,1:k),'b-','DisplayName','car trajectory'); 
    %plot refrence Points
    plot(nextPathPoints(1,:), nextPathPoints(2,:),'.r', 'DisplayName', ...
        'Car Refrence.');
    plot(predictedX(1,:),predictedX(2,:),'*g', 'DisplayName', ...
        'predicted car traj.');
    

    
    % plot velocity
    subplot(5,2,2); 
    oldVel = findobj(gca,'Color','blue');
    delete(oldVel);
    oldVelPred = findobj(gca,'Color','green');
    delete(oldVelPred);
    plot(x(3,1:k),'b-');
    plot(k:k+model.N-1,predictedX(3,:),'g-');
    
    % plot heading angle
    subplot(5,2,4); 
    oldTheta = findobj(gca,'Color','blue');
    delete(oldTheta);
    oldThetaPred = findobj(gca,'Color','green');
    delete(oldThetaPred);
    plot(rad2deg(x(4,1:k)),'b-');
    plot(k:k+model.N-1,rad2deg(predictedX(4,:)),'g-');
    
    % plot steering angle
    subplot(5,2,6); 
    oldDelta = findobj(gca,'Color','blue');
    delete(oldDelta);
    oldDeltaPred = findobj(gca,'Color','green');
    delete(oldDeltaPred);
    plot(rad2deg(x(5,1:k)),'b-');
    plot(k:k+model.N-1,rad2deg(predictedX(5,:)),'g-');
    
    % plot acceleration force
    subplot(5,2,8);
    oldAcc = findobj(gca,'Color','blue');
    delete(oldAcc);
    oldAccPred = findobj(gca,'Color','green');
    delete(oldAccPred);
    stairs(u(1,1:k),'b-');
    stairs(k:k+model.N-1,predictedU(1,:),'g-');
    
    %plot steering rate
    subplot(5,2,10); 
    oldPhi = findobj(gca,'Color','blue');
    delete(oldPhi);
    oldPhiPred = findobj(gca,'Color','green');
    delete(oldPhiPred);
    stairs(rad2deg(u(2,1:k)),'b-');
    stairs(k:k+model.N-1,rad2deg(predictedU(2,:)),'g-');
     
    pause(0.1); % wait 0.1sec to make current progress visible
end