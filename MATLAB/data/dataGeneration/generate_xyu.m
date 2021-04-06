close all;
clear all;
clc;
% Change Directory: 
cd("../../data");
addpath("../models","../functions","../data/dataGeneration/inputSignals", "../estimation");

sampleSize = 1000;
res = 1; % Increase data resolution size

dataName = 'datasets/data_motion_model_velocity_loop1.mat';

%% Description:
% Script will generate simulated data input(u) and coordinate(xy) data.
% Coordinate data is for generating an unknown track and landmarks/
% input data is assumed data from the systems controller. 

%% Toggle options (true/false)
% Animate measurements and robot trajectory
animate = false;
skip = 1; % Skip to this frame (animate)

% Plot Tracjectory
visualise = false;
% Save data
saveData = true;

%% Parameters
% Samples
N = sampleSize*res;

% Models
% Examples: [robotModel2Wheels, simpleBicycleModel, motion_model_velocity]
par.vehicleModel = 'robotModel2Wheels';
% Examples: ['forwardEulerMethod', 'rk4']
par.STM = 'forwardEulerMethod';
% Examples: ['input_motion_model_velocity_const', 'input_robotModel2Wheels_loop1']
par.inputSignal = 'input_robotModel2Wheels_loop1';
% Store values in params
par.nx = size(vehicleModel(par),1); % number of states
for u = 1:10
    try
        vehicleModel(par,ones(par.nx,1),ones(u,1));
        par.nu = u; % number of inputs
        break;
    catch
        % try again
    end
end
par.N =  N;
par.Delta = 0.1/res;
% Initial Conditions
x0 = [0;0;0;0;0];
x = zeros(par.nx,N+1);
x(:,1) = x0;

u = zeros(par.nu, N);

xy = zeros(2,N);

%% SIM-U-LATE (in my head it's a Dalek saying it..)
startplot = 800
for t = 1:N
    u(:,t) = inputSignal(t,u(:,t),par, res);
    x(:,t+1) = STM(par,x(:,t),u(:,t));
    xy(:,t) = x(1:2,t);
    if animate
        figure(1)
        if t == 1
            tr = plot(xy(1,1:t),xy(2,1:t),'b-')
            hold on
            bot = scatter(xy(1,t),xy(2,t),'r')
            hold off
        else
            if t > startplot
                t
                set(tr,'xdata',xy(1,1:t),'ydata',xy(2,1:t))
                hold on
                set(bot,'xdata',xy(1,t),'ydata',xy(2,t))
                hold off
            end
        end
    title(sprintf('Observed Data:\t [k = %i t = %.2f s]  Sampling: [%0.1f Hz]\n [N: %.2f, E: %.2f, \\psi %.2f, u: %.2f, r: %.2f]',...
    t, t*par.Delta,1/par.Delta,...
    x(2,t),x(1,t),x(3,t)*180/pi,x(4,t),x(5,t)))
    axis([-10 50 -50 50])
    end
end
if visualise
    figure(2)
    plot(xy(1,:),xy(2,:))
end
dt = par.Delta;
save(dataName,'xy','u','dt')