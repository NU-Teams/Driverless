% Dummy test data
t = 1:7;
x = rand(size(t));
y = rand(size(t));
%Aidan Mc Track
% data1 = load('xpos.mat');
% data2 = load('ypos.mat');
% x = data1.xpos;
% y = data2.ypos;
% t = 1:length(x);

fx = spline(t,x);
fy = spline(t,y);
d1fx = differentiate(fx);
d1fy = differentiate(fy);
d2fx = differentiate(d1fx);
d2fy = differentiate(d1fy);
ti = linspace(min(t),max(t),length(x));
xi = ppval(fx,ti);
yi = ppval(fy,ti);

path = [x;y]';


d1xi = ppval(d1fx,ti);
d1yi = ppval(d1fy,ti);
d2xi = ppval(d2fx,ti);
d2yi = ppval(d2fy,ti);
K = abs(d1xi.*d2yi - d2xi.*d2yi) ./ sqrt(d1xi.^2+d1yi.^2).^3;
velocity = log(1./K);        %Velocity Reference
save('velocity','velocity')

fig = figure(1);
[bezcurve, intcurveyy] = bezier_(path, 500, [], fig);

figure(2)
clf
plot(x,y,'ow');
hold on
% FEX https://fr.mathworks.com/matlabcentral/fileexchange/60402-plotcolor
plot_color(xi,yi,log(K),jet,[],'Linewidth',2);
set(gca,'Color','k');
axis equal
function ppdf = differentiate(ppf)
% Spline Derivative
ppdf = ppf;
ppdf.order=ppdf.order-1;
ppdf.coefs=ppdf.coefs(:,1:end-1).*(ppdf.order:-1:1);
end