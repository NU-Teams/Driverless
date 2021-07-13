clc
close all
%% Load Track
% Dummy test data
% t = 1:7;
% x = rand(size(t));
% y = rand(size(t));
%Aidan Mc Track
% data1 = load('xpos.mat');
% data2 = load('ypos.mat');
% x = data1.xpos;
% y = data2.ypos;

%Handling track
data2 = load('Path.mat');
Array = data2.p;
% double the data in X column
x = interp(Array(:,1),2)';
% dobule the data in Y column
y = interp(Array(:,2),2)';


t = 1:length(x);

%% interpolate path
%using a bezier curve instead of matlabs spline function to interpolate,
%this function can be wrriten in c
path = [x;y]';
fig = 'o';
[bezcurve, intcurveyy] = bezier_(path, 500, [], fig);

% using matlab spline tool
fx = spline(t,x);
fy = spline(t,y);
d1fx = differentiate(fx);
d1fy = differentiate(fy);
d2fx = differentiate(d1fx);
d2fy = differentiate(d1fy);
ti = linspace(min(t),max(t),length(x));

%evaluate splines using ppval
xi = ppval(fx,ti);
yi = ppval(fy,ti);
d1xi = ppval(d1fx,ti);
d1yi = ppval(d1fy,ti);
d2xi = ppval(d2fx,ti);
d2yi = ppval(d2fy,ti);

%% calculate curvature
% Kappa = abs(d1xi.*d2yi - d2xi.*d2yi) ./ sqrt(d1xi.^2+d1yi.^2).^3;
% L = Cumulative arc length , R = Radius of curvature, k = Curvature vector
[L,R,K] = curvaturecal([xi;yi]');


K(isnan(K))=0;

%% plot curvature heatmap
figure(2)
plot(x,y,'ow');
hold on
% FEX https://fr.mathworks.com/matlabcentral/fileexchange/60402-plotcolor

%plot curvature with respect to y as K(:,2)
plot_color(xi,yi,K(:,2),jet,[],'Linewidth',4); 
% plot_color(x,y);
set(gca,'Color','k');
xlim([-100,10]);
ylim([-80,130]);
% axis equal



function ppdf = differentiate(ppf)
% Spline Derivative
ppdf = ppf;
ppdf.order=ppdf.order-1;
ppdf.coefs=ppdf.coefs(:,1:end-1).*(ppdf.order:-1:1);
end