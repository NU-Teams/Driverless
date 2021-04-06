function h=drawRobot(pose,h)
%% h = drawRobot(pose)
% Description:
    % Draws robot with forward & right axis given robot pose

% Heading in radians
psi = pose.psi;
if isempty(h)
    % Draw robot as a circle with forward and right axis
%     h1 = scatter(pose.east,pose.north,'r');
    h2 = line([pose.east pose.east+5*sin(psi)], [pose.north pose.north+5*cos(psi)]);
    h3 = line([pose.east pose.east+5*sin(psi+pi/2)], [pose.north pose.north+5*cos(psi+pi/2)]);

%     hasbehavior(h1, 'legend', false);
    hasbehavior(h2, 'legend', false);
    hasbehavior(h3, 'legend', false);

%     set(h1,'linewidth',1);
    set(h2,'linewidth',2,'color','g');
    set(h3,'linewidth',2,'color','g');
%     h.h1 = h1;
    h.h2 = h2;
    h.h3 = h3;
else
    
%     set(h.h1,'xdata',pose.east,'ydata',pose.north);
    set(h.h2,'xdata',[pose.east pose.east+5*sin(psi)],'ydata', [pose.north pose.north+5*cos(psi)]);
    set(h.h3,'xdata',[pose.east pose.east+5*sin(psi+pi/2)],'ydata', [pose.north pose.north+5*cos(psi+pi/2)]);
end

end