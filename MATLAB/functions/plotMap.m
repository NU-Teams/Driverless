function p = plotMap(muf,Pf,z,k,p,s)
colourSelect = [1 0 0 0.05];
hold on

    if k == 1
        % Plot animated trajectory
%         p.tr = plot(x(1,1:k),x(2,1:k), '-','color', [0,0,0,0.2],'linewidth',2);
%         p.trh = plot(muf(1,1:k),muf(2,1:k), 'c-','linewidth',1);


        % Plot animated measurements
        xe = muf(1) + z(1,:).*sin(muf(3)+z(2,:));
        ye = muf(2) + z(1,:).*cos(muf(3)+z(2,:));

        % Plot Measurements of Landmarks
        xp  = muf(1) + zeros(2*length(xe),1);
        yp  = muf(2) + zeros(2*length(ye),1);
        xp(2:2:end) = xe;
        yp(2:2:end) = ye;
        p.h1 = plot(xp,yp,'-*','MarkerSize',2,'color',colourSelect);

        % Plot Robot
        pose.east = muf(1);
        pose.north = muf(2);
        pose.psi = muf(3);

        p.rh = [];
        p.rh = drawRobot(pose,p.rh); % Plot robot forward right
        [xx,yy] = cov2elli(muf(1:2),Pf(1:2,1:2));
        p.rhc = line(xx,yy,'linestyle','-','color','r');
        hasbehavior(p.rhc, 'legend', false);

        p.xm = [];
        p.Pc = [];
        % Plot Mean & covariance of L1
        for j = 1:s.nLandmarks
            if ~isnan(muf((s.ny*j+s.nx-1)))
            % Create a temp variable for landmark
                p.xm(j,1) = plot(muf((s.ny*j+s.nx-1)),muf((s.ny*j+s.nx)), 'm+', 'linewidth', 1, 'markersize',8);
                [xx,yy] = cov2elli(muf((s.ny*j+s.nx-1):(s.ny*j+s.nx)),Pf((s.ny*j+s.nx-1):(s.ny*j+s.nx),(s.ny*j+s.nx-1):(s.ny*j+s.nx)));

                p.Pc(j,1) = line(xx,yy,'linestyle','-','color','r');
            else
                p.xm(j,1) = plot(muf(1),muf(2), 'm+', 'linewidth', 1, 'markersize',8);
                p.Pc(j,1) = line(0,0,'linestyle','-','color','r');
            end
            if j ~= 1
                hasbehavior(p.xm(j), 'legend', false);
                hasbehavior(p.Pc(j), 'legend', false);
            end
        end
    else

        % Plot trajectory
%         set(p.tr,'xdata',x(1,1:k),'ydata',x(2,1:k));
%         set(p.trh,'xdata',muf(1,1:k),'ydata',muf(2,1:k));

        % Plot measurements of L1
        xe = muf(1) + z(1,:).*sin(muf(3)+z(2,:));
        ye = muf(2) + z(1,:).*cos(muf(3)+z(2,:));

        xp  = muf(1) + zeros(2*length(xe),1);
        yp  = muf(2) + zeros(2*length(ye),1);
        
        xp(2:2:end) = xe;
        yp(2:2:end) = ye;
        set(p.h1,'xdata',xp,'ydata',yp);

        % Plot Robot
        pose.east = muf(1);
        pose.north = muf(2);
        pose.psi = muf(3);

        drawRobot(pose,p.rh); % Plot robot forward right
        [xx,yy] = cov2elli(muf(1:2),Pf(1:2,1:2));
        set(p.rhc,'xdata',xx,'ydata',yy);
        
        % Plot mean and covariance of each landmark
        for j = 1:s.nLandmarks
            if j <= size(p.xm,1)
                set(p.xm(j,1),'xdata',muf((s.ny*j+s.nx-1)),'ydata',muf((s.ny*j+s.nx)));
                [xx,yy] = cov2elli(muf((s.ny*j+s.nx-1):(s.ny*j+s.nx)),Pf((s.ny*j+s.nx-1):(s.ny*j+s.nx),(s.ny*j+s.nx-1):(s.ny*j+s.nx)));
                set(p.Pc(j,1),'xdata',xx,'ydata',yy);
            else
                p.xm(j,1) = plot(muf((s.ny*j+s.nx-1)),muf((s.ny*j+s.nx)), 'm+', 'linewidth', 1, 'markersize',8);
                [xx,yy] = cov2elli(muf((s.ny*j+s.nx-1):(s.ny*j+s.nx)),Pf((s.ny*j+s.nx-1):(s.ny*j+s.nx),(s.ny*j+s.nx-1):(s.ny*j+s.nx)));
                p.Pc(j,1) = line(xx,yy,'linestyle','-','color','r');
            end

        end

    end


    hold off
    legend('True Landmark','True Trajectory', 'Filtered Trajectory', 'Measurments','Mean Landmark','Covariance','Location','northeast')
    axis([0,120,0,120])
    xlabel('East [m]')
    ylabel('North [m]')

    title(sprintf('Observed Data:\t [k = %i t = %.2f s]  Sampling: [%0.1f Hz]\n [N: %.2f, E: %.2f, \\psi %.2f, u: %.2f, r: %.2f]',...
    k, k*s.Delta,1/s.Delta,...
    muf(2),muf(1),muf(3)*180/pi,muf(4),muf(5)))

    drawnow
   
end