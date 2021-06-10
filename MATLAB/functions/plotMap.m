function [p, s] = plotMap(muf,Pf,z,k,p,s)
colourSelect = [0 0 1 0.1];
nLandmarks = length(muf((s.nx+1):end))/2;
try
    gtLandmarks = size(s.gt.landmarks,1);
catch
end

if s.dataSet == 'victoriaPark'
    L = zeros(s.ny,nLandmarks);
    if k == 1
        % Setup animation
        figure(2)
        set(gcf, 'Position',  [10, 200, 600, 600],'Color',[1,1,1])
        hold on
        % Plot GPS
        p.gps = plot(s.z.gps(1:k,1),s.z.gps(1:k,2),'.','color', [0.9,0.9,0,0.4],'DisplayName','GPS','MarkerSize',18);
        hasbehavior(p.gps, 'legend', true);
        % Plot animated trajectory
        p.tr = plot(s.traj(1,1:k),s.traj(2,1:k), '-','color', [0,0,0,0.4],'linewidth',2, 'DisplayName','Trajectory');
        hasbehavior(p.tr, 'legend', true);
        
        % Plot animated measurements
        try
            xe = s.LidState(1) + s.LidPlot(1,:).*sin(s.LidState(3)+s.LidPlot(2,:));
            ye = s.LidState(2) + s.LidPlot(1,:).*cos(s.LidState(3)+s.LidPlot(2,:));

            % Plot Measurements of Landmarks
            xp  = muf(1) + zeros(2*length(xe),1);
            yp  = muf(2) + zeros(2*length(ye),1);
            xp(2:2:end) = xe;
            yp(2:2:end) = ye;
            p.h1 = plot(xp,yp,'-*','MarkerSize',2,'color',colourSelect, 'DisplayName','LiDAR');
            hasbehavior(p.h1, 'legend', true);
        catch 
        end
        
        % Plot Mean & covariance of Landmarks
        p.xm = [];
        p.Pc = [];
        for j = 1:nLandmarks
            L(:,j) = [muf((s.ny*j+s.nx-1));muf((s.ny*j+s.nx))];
            if ~isnan(muf((s.ny*j+s.nx-1)))
            % Create a temp variable for landmark
            try
                p.xm(j,1) = plot(muf((s.ny*j+s.nx-1)),muf((s.ny*j+s.nx)), 'm+', 'linewidth', 1, 'markersize',8,'DisplayName','Object Mean');
                [xx,yy] = cov2elli(muf((s.ny*j+s.nx-1):(s.ny*j+s.nx)),Pf((s.ny*j+s.nx-1):(s.ny*j+s.nx),(s.ny*j+s.nx-1):(s.ny*j+s.nx)));
                p.Pc(j,1) = line(xx,yy,'linestyle','-','color','r', 'DisplayName','Object Covariance');
                hasbehavior(p.xm(j), 'legend', true);
                hasbehavior(p.Pc(j), 'legend', true);
            catch
            end
            else
                p.xm(j,1) = plot(muf((s.ny*j+s.nx-1)),muf((s.ny*j+s.nx)), 'm+', 'linewidth', 1, 'markersize',8,'DisplayName','Object Mean');
                [xx,yy] = cov2elli(muf((s.ny*j+s.nx-1):(s.ny*j+s.nx)),Pf((s.ny*j+s.nx-1):(s.ny*j+s.nx),(s.ny*j+s.nx-1):(s.ny*j+s.nx)));
                p.Pc(j,1) = line(xx,yy,'linestyle','-','color','r', 'DisplayName','Object Covariance');
                hasbehavior(p.xm(j), 'legend', false);
                hasbehavior(p.Pc(j), 'legend', false);
            end
            
        end
        % Draw robot pose & certainty
        pose.east = muf(1);
        pose.north = muf(2);
        pose.psi = muf(3);
        pose.steer = s.uPlot(2);
        p.rh = [];
        p.rh = drawRobot(pose,p.rh); % Plot robot forward right
%         [xx,yy] = cov2elli(muf(1:2),Pf(1:2,1:2));
%         p.rhc = line(xx,yy,'linestyle','-','color','r');
%         hasbehavior(p.rhc, 'legend', false);
    else
        hold on
        
        % Plot animated trajectory
        set(p.tr,'xdata',s.traj(1,1:k),'ydata',s.traj(2,1:k));
        
        % Plot LiDAR measurements
        try
            xe = s.LidState(1) + s.LidPlot(1,:).*sin(s.LidState(3)+s.LidPlot(2,:));
            ye = s.LidState(2) + s.LidPlot(1,:).*cos(s.LidState(3)+s.LidPlot(2,:));

            xp  = muf(1) + zeros(2*length(xe),1);
            yp  = muf(2) + zeros(2*length(ye),1);

            xp(2:2:end) = xe;
            yp(2:2:end) = ye;
            set(p.h1,'xdata',xp,'ydata',yp);
        catch
        end
        % Plot GPS
        set(p.gps,'xdata',s.z.gps(1:k,1),'ydata',s.z.gps(1:k,2));
        
        % Plot mean and covariance of each landmark
        for j = 1:nLandmarks
            L(:,j) = [muf((s.ny*j+s.nx-1));muf((s.ny*j+s.nx))];
            if j <= size(p.xm,1)
                try
                set(p.xm(j,1),'xdata',muf((s.ny*j+s.nx-1)),'ydata',muf((s.ny*j+s.nx)));
                [xx,yy] = cov2elli(muf((s.ny*j+s.nx-1):(s.ny*j+s.nx)),Pf((s.ny*j+s.nx-1):(s.ny*j+s.nx),(s.ny*j+s.nx-1):(s.ny*j+s.nx)));
                set(p.Pc(j,1),'xdata',xx,'ydata',yy);
                catch
                end
            else
                if j == 1
                    p.xm(j,1) = plot(muf((s.ny*j+s.nx-1)),muf((s.ny*j+s.nx)), 'm+', 'linewidth', 1, 'markersize',8,'DisplayName','Object Mean');
                    [xx,yy] = cov2elli(muf((s.ny*j+s.nx-1):(s.ny*j+s.nx)),Pf((s.ny*j+s.nx-1):(s.ny*j+s.nx),(s.ny*j+s.nx-1):(s.ny*j+s.nx)));
                    p.Pc(j,1) = line(xx,yy,'linestyle','-','color','r', 'DisplayName','Object Covariance');
                    hasbehavior(p.xm(j), 'legend', true);
                    hasbehavior(p.Pc(j), 'legend', true);
                else
                    p.xm(j,1) = plot(muf((s.ny*j+s.nx-1)),muf((s.ny*j+s.nx)), 'm+', 'linewidth', 1, 'markersize',8);
                    [xx,yy] = cov2elli(muf((s.ny*j+s.nx-1):(s.ny*j+s.nx)),Pf((s.ny*j+s.nx-1):(s.ny*j+s.nx),(s.ny*j+s.nx-1):(s.ny*j+s.nx)));
                    p.Pc(j,1) = line(xx,yy,'linestyle','-','color','r');
                    hasbehavior(p.xm(j), 'legend', false);
                    hasbehavior(p.Pc(j), 'legend', false);
                end
            end
        end
        
        if j < size(p.xm,1)
            % Remove data from plot
            for jj = 1:(size(p.xm,1)-j)
                set(p.xm(j+jj,1),'xdata',NaN,'ydata',NaN);
                set(p.Pc(j+jj,1),'xdata',muf(1).*ones(1,17),'ydata',muf(2).*ones(1,17));
            end
            % resize the vector
            p.xm = p.xm(1:j,1);
            p.Pc = p.Pc(1:j,1);
        end
        % Plot Robot
        pose.east = muf(1);
        pose.north = muf(2);
        pose.psi = muf(3);        
        pose.steer = s.uPlot(2);

        drawRobot(pose,p.rh); % Plot robot forward right
%         [xx,yy] = cov2elli(muf(1:2),Pf(1:2,1:2));
%         set(p.rhc,'xdata',xx,'ydata',yy);
        
    end


    hold off
    legend('Location','northeast')
    xlabel('East [m]')
    ylabel('North [m]')
    try
        title(sprintf('Event:\t [k = %i t = %.2f s]  Sampling: [%0.1f Hz]\n State: [N: %.2f, E: %.2f, \\psi %.2f] [lm: %i]',...
        k, s.time*s.Delta,1/s.Delta,...
        muf(2),muf(1),muf(3)*180/pi,nLandmarks))
    catch
    end
    % Axis
    xmin = min([L(1,:),muf(1)]);
    xmax = max([L(1,:),muf(1)]);
    ymin = min([L(2,:),muf(2)]);
    ymax = max([L(2,:),muf(2)]);
    xext = sqrt(abs(xmax-xmin)); xmin = xmin-xext; xmax = xmax+xext;
    yext = sqrt(abs(ymax-ymin)); ymin = ymin-yext; ymax = ymax+yext;
    if xmin < s.plot.xmin
        s.plot.xmin = xmin;
    end
    if xmax > s.plot.xmax
        s.plot.xmax = xmax;
    end
    if ymin < s.plot.ymin
        s.plot.ymin = ymin;
    end
    if ymax > s.plot.ymax
        s.plot.ymax = ymax;
    end
%     val = 50;
%     s.plot.xmin = muf(1) - val;
%     s.plot.xmax = muf(1) + val;
%     s.plot.ymin = muf(2) - val;
%     s.plot.ymax = muf(2) + val;
    axis([s.plot.xmin,s.plot.xmax,s.plot.ymin,s.plot.ymax])
    axis equal
    grid on
    drawnow
   
else
L = zeros(s.ny,nLandmarks);
    if k == 1

        % Setup animation
        figure(2)
        set(gcf, 'Position',  [10, 200, 600, 600])
        hold on
        % Plot animated trajectory
%         p.tr = plot(x(1,1:k),x(2,1:k), '-','color', [0,0,0,0.2],'linewidth',2);
%         p.trh = plot(muf(1,1:k),muf(2,1:k), 'c-','linewidth',1);

        try
            p.lm = plot(s.gt.landmarks(:,1),s.gt.landmarks(:,2),'o','MarkerSize',4,'color',[0 0 0 0.2]);
            hasbehavior(p.lm, 'legend', true);
        catch
        end
        
        % Plot animated measurements
        try
            xe = muf(1) + z(1,:).*sin(muf(3)+z(2,:));
            ye = muf(2) + z(1,:).*cos(muf(3)+z(2,:));

            % Plot Measurements of Landmarks
            xp  = muf(1) + zeros(2*length(xe),1);
            yp  = muf(2) + zeros(2*length(ye),1);
            xp(2:2:end) = xe;
            yp(2:2:end) = ye;
            p.h1 = plot(xp,yp,'-*','MarkerSize',2,'color',colourSelect);
            hasbehavior(p.h1, 'legend', true);
        catch 
        end
        % Plot Robot
        pose.east = muf(1);
        pose.north = muf(2);
        pose.psi = muf(3);

        p.xm = [];
        p.Pc = [];
        % Plot Mean & covariance of L1
        for j = 1:nLandmarks
            L(:,j) = [muf((s.ny*j+s.nx-1));muf((s.ny*j+s.nx))];
            if ~isnan(muf((s.ny*j+s.nx-1)))
            % Create a temp variable for landmark
            try
                p.xm(j,1) = plot(muf((s.ny*j+s.nx-1)),muf((s.ny*j+s.nx)), 'm+', 'linewidth', 1, 'markersize',8);
                [xx,yy] = cov2elli(muf((s.ny*j+s.nx-1):(s.ny*j+s.nx)),Pf((s.ny*j+s.nx-1):(s.ny*j+s.nx),(s.ny*j+s.nx-1):(s.ny*j+s.nx)));

                p.Pc(j,1) = line(xx,yy,'linestyle','-','color','r');
            catch
            end
            else
                p.xm(j,1) = plot(muf(1),muf(2), 'm+', 'linewidth', 1, 'markersize',8);
                p.Pc(j,1) = line(0,0,'linestyle','-','color','r');
            end
            if j ~= 1
                hasbehavior(p.xm(j), 'legend', false);
                hasbehavior(p.Pc(j), 'legend', false);
            end
        end
        p.rh = [];
        p.rh = drawRobot(pose,p.rh); % Plot robot forward right
%         [xx,yy] = cov2elli(muf(1:2),Pf(1:2,1:2));
%         p.rhc = line(xx,yy,'linestyle','-','color','r');
%         hasbehavior(p.rhc, 'legend', false);
    else
        hold on

        % Plot trajectory
%         set(p.tr,'xdata',x(1,1:k),'ydata',x(2,1:k));
%         set(p.trh,'xdata',muf(1,1:k),'ydata',muf(2,1:k));

        % Plot measurements of L1
        try
        xe = muf(1) + z(1,:).*sin(muf(3)+z(2,:));
        ye = muf(2) + z(1,:).*cos(muf(3)+z(2,:));

        xp  = muf(1) + zeros(2*length(xe),1);
        yp  = muf(2) + zeros(2*length(ye),1);
        
        xp(2:2:end) = xe;
        yp(2:2:end) = ye;
        set(p.h1,'xdata',xp,'ydata',yp);
        catch
        end

        % Plot Robot
        pose.east = muf(1);
        pose.north = muf(2);
        pose.psi = muf(3);

        drawRobot(pose,p.rh); % Plot robot forward right
        [xx,yy] = cov2elli(muf(1:2),Pf(1:2,1:2));
        set(p.rhc,'xdata',xx,'ydata',yy);
        
        % Plot mean and covariance of each landmark
        for j = 1:nLandmarks
            L(:,j) = [muf((s.ny*j+s.nx-1));muf((s.ny*j+s.nx))];
            if j <= size(p.xm,1)
                try
                set(p.xm(j,1),'xdata',muf((s.ny*j+s.nx-1)),'ydata',muf((s.ny*j+s.nx)));
                [xx,yy] = cov2elli(muf((s.ny*j+s.nx-1):(s.ny*j+s.nx)),Pf((s.ny*j+s.nx-1):(s.ny*j+s.nx),(s.ny*j+s.nx-1):(s.ny*j+s.nx)));
                set(p.Pc(j,1),'xdata',xx,'ydata',yy);
                catch
                end
            else
                p.xm(j,1) = plot(muf((s.ny*j+s.nx-1)),muf((s.ny*j+s.nx)), 'm+', 'linewidth', 1, 'markersize',8);
                [xx,yy] = cov2elli(muf((s.ny*j+s.nx-1):(s.ny*j+s.nx)),Pf((s.ny*j+s.nx-1):(s.ny*j+s.nx),(s.ny*j+s.nx-1):(s.ny*j+s.nx)));
                p.Pc(j,1) = line(xx,yy,'linestyle','-','color','r');
            end

        end
        if j < size(p.xm,1)
            % Remove data from plot
            for jj = 1:(size(p.xm,1)-j)
                set(p.xm(j+jj,1),'xdata',NaN,'ydata',NaN);
                set(p.Pc(j+jj,1),'xdata',muf(1).*ones(1,17),'ydata',muf(2).*ones(1,17));
            end
            % resize the vector
            p.xm = p.xm(1:j,1);
            p.Pc = p.Pc(1:j,1);
        end

    end


    hold off
%     legend('True Landmark', 'Filtered Trajectory', 'Measurments','Mean Landmark','Covariance','Location','northeast')
    legend('True Landmark', 'Measurments','Mean Landmark','Covariance','Location','southeast')
    xlabel('East [m]')
    ylabel('North [m]')
    try
        title(sprintf('Observed Data:\t [k = %i t = %.2f s]  Sampling: [%0.1f Hz]\n [N: %.2f, E: %.2f, \\psi %.2f, u: %.2f, r: %.2f] [lm: %i/%i]',...
        k, k*s.Delta,1/s.Delta,...
        muf(2),muf(1),muf(3)*180/pi,muf(4),muf(5),nLandmarks, gtLandmarks))
    catch
    end
    % Axis
    xmin = min([L(1,:),muf(1)]);
    xmax = max([L(1,:),muf(1)]);
    ymin = min([L(2,:),muf(2)]);
    ymax = max([L(2,:),muf(2)]);
    xext = sqrt(abs(xmax-xmin)); xmin = xmin-xext; xmax = xmax+xext;
    yext = sqrt(abs(ymax-ymin)); ymin = ymin-yext; ymax = ymax+yext;
    if xmin < s.plot.xmin
        s.plot.xmin = xmin;
    end
    if xmax > s.plot.xmax
        s.plot.xmax = xmax;
    end
    if ymin < s.plot.ymin
        s.plot.ymin = ymin;
    end
    if ymax > s.plot.ymax
        s.plot.ymax = ymax;
    end
    axis([s.plot.xmin,s.plot.xmax,s.plot.ymin,s.plot.ymax])
    axis equal
    grid on
    drawnow
   
end
end