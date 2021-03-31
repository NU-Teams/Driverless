function h = updateScanPlot(h,laser,pose,r,colorSelect)

n   = laser.numScans;
idx = 0:n-1;

x0  = pose.east  - laser.right0;
y0  = pose.north - laser.forward0;
h0  = pose.psi   - laser.angle_down0;

xe  = x0 + r.*sin((h0 + laser.startDeg + laser.resDeg*idx(:))*pi/180);
ye  = y0 + r.*cos((h0 + laser.startDeg + laser.resDeg*idx(:))*pi/180);
xp  = x0 + zeros(2*length(xe),1);
yp  = y0 + zeros(2*length(ye),1);
xp(2:2:end) = xe;
yp(2:2:end) = ye;

if isempty(h)
    h   = plot(xp,yp,'-*','MarkerSize',2,'color',colorSelect);
else
    set(h,'xdata',xp,'ydata',yp);
end