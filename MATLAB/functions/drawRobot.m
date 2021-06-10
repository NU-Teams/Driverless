function h=drawRobot(pose,h)
%% h = drawRobot(pose)
% Description:
    % Draws robot with forward & right axis given robot pose

% Heading in radians
psi = pose.psi;
if isempty(h)
    % Draw robot as a circle with forward and right axis
    h3 = line([pose.east-4*sin(psi) pose.east], [pose.north-4*cos(psi) pose.north],'DisplayName','Vehicle');
    s1 = line([pose.east+2.8*sin(psi)+0.5*sin(psi+pi/2), pose.east+2.8*sin(psi)+0.5*sin(psi+pi/2)+sin(psi+pose.steer)],[pose.north+2.8*cos(psi)+0.5*cos(psi+pi/2), pose.north+2.8*cos(psi)+0.5*cos(psi+pi/2)+cos(psi+pose.steer)]);
    s2 = line([pose.east+2.8*sin(psi)+0.5*sin(psi-pi/2), pose.east+2.8*sin(psi)+0.5*sin(psi-pi/2)+sin(psi+pose.steer)],[pose.north+2.8*cos(psi)+0.5*cos(psi-pi/2), pose.north+2.8*cos(psi)+0.5*cos(psi-pi/2)+cos(psi+pose.steer)]);

%     h1 = line([pose.east+0.5*sin(psi+pi/2) pose.east+4*sin(psi)], [pose.north+0.5*cos(psi+pi/2) pose.north+4*cos(psi)]);
%     h2 = line([pose.east+0.5*sin(psi-pi/2) pose.east+4*sin(psi)], [pose.north+0.5*cos(psi-pi/2) pose.north+4*cos(psi)]);
%     h3 = line([pose.east pose.east+0.5*sin(psi+pi/2)], [pose.north pose.north+0.5*cos(psi+pi/2)],'DisplayName','Vehicle');
%     h4 = line([pose.east pose.east+0.5*sin(psi-pi/2)], [pose.north pose.north+0.5*cos(psi-pi/2)]);
%     s1 = line([pose.east+2.8*sin(psi)+0.5*sin(psi+pi/2), pose.east+2.8*sin(psi)+0.5*sin(psi+pi/2)+sin(psi+pose.steer)],[pose.north+2.8*cos(psi)+0.5*cos(psi+pi/2), pose.north+2.8*cos(psi)+0.5*cos(psi+pi/2)+cos(psi+pose.steer)]);
%     s2 = line([pose.east+2.8*sin(psi)+0.5*sin(psi-pi/2), pose.east+2.8*sin(psi)+0.5*sin(psi-pi/2)+sin(psi+pose.steer)],[pose.north+2.8*cos(psi)+0.5*cos(psi-pi/2), pose.north+2.8*cos(psi)+0.5*cos(psi-pi/2)+cos(psi+pose.steer)]);

%     hasbehavior(h1, 'legend', false);
%     hasbehavior(h2, 'legend', false);
    hasbehavior(h3, 'legend', true);
%     hasbehavior(h4, 'legend', false);
    hasbehavior(s1, 'legend', false);
    hasbehavior(s2, 'legend', false);

%     set(h1,'linewidth',1,'color','k');
%     set(h2,'linewidth',1,'color','k');
    set(h3,'linewidth',5,'color','b');
%     set(h4,'linewidth',3,'color','k');
    set(s1,'linewidth',2,'color',[1,0,0.9,0.8]);
    set(s2,'linewidth',2,'color',[1,0,0.9,0.8]);
%     h.h1 = h1;
%     h.h2 = h2;
    h.h3 = h3;
%     h.h4 = h4;
    h.s1 = s1;
    h.s2 = s2;
else
%     set(h.h1,'xdata',[pose.east+0.5*sin(psi+pi/2) pose.east+4*sin(psi)],'ydata',[pose.north+0.5*cos(psi+pi/2) pose.north+4*cos(psi)]);
%     set(h.h2,'xdata',[pose.east+0.5*sin(psi-pi/2) pose.east+4*sin(psi)],'ydata', [pose.north+0.5*cos(psi-pi/2) pose.north+4*cos(psi)]);
%     set(h.h3,'xdata',[pose.east pose.east+0.5*sin(psi+pi/2)],'ydata', [pose.north pose.north+0.5*cos(psi+pi/2)]);
%     set(h.h4,'xdata',[pose.east pose.east+0.5*sin(psi-pi/2)],'ydata', [pose.north pose.north+0.5*cos(psi-pi/2)]);
%     set(h.s1,'xdata',[pose.east+2.8*sin(psi)+0.45*sin(-psi+pi/2), pose.east+2.8*sin(psi)+0.45*sin(psi+pi/2)+1.5*sin(psi-pose.steer)],'ydata', [pose.north+2.8*cos(psi)+0.45*cos(psi+pi/2), pose.north+2.8*cos(psi)+0.45*cos(psi+pi/2)+1.5*cos(psi-pose.steer)]);
%     set(h.s2,'xdata',[pose.east+2.8*sin(psi)+0.45*sin(-psi-pi/2), pose.east+2.8*sin(psi)+0.45*sin(psi-pi/2)+1.5*sin(psi-pose.steer)],'ydata', [pose.north+2.8*cos(psi)+0.45*cos(psi-pi/2), pose.north+2.8*cos(psi)+0.45*cos(psi-pi/2)+1.5*cos(psi-pose.steer)]);  

    set(h.h3,'xdata',[pose.east-2*sin(psi) pose.east+2*sin(psi)],'ydata', [pose.north-2*cos(psi) pose.north+2*cos(psi)]);
    set(h.s1,'xdata',[pose.east+2.8*sin(psi)+0.45*sin(-psi+pi/2)-2*sin(psi), pose.east+2.8*sin(psi)+0.45*sin(psi+pi/2)+1.5*sin(psi-pose.steer)-2*sin(psi)],'ydata', [pose.north+2.8*cos(psi)+0.45*cos(psi+pi/2)-2*cos(psi), pose.north+2.8*cos(psi)+0.45*cos(psi+pi/2)+1.5*cos(psi-pose.steer)-2*cos(psi)]);
    set(h.s2,'xdata',[pose.east+2.8*sin(psi)+0.45*sin(-psi-pi/2)-2*sin(psi), pose.east+2.8*sin(psi)+0.45*sin(psi-pi/2)+1.5*sin(psi-pose.steer)-2*sin(psi)],'ydata', [pose.north+2.8*cos(psi)+0.45*cos(psi-pi/2)-2*cos(psi), pose.north+2.8*cos(psi)+0.45*cos(psi-pi/2)+1.5*cos(psi-pose.steer)-2*cos(psi)]);

end

end