clc
clear

%% import track
track=readtable('track19.csv');

%% parse table
element=size(track);

class=string(table2array(track(1:element(1),1)));
x=table2array(track(1:element(1),2));
y=table2array(track(1:element(1),3));
direction=table2array(track(1:element(1),4));
xcov=table2array(track(1:element(1),5));
ycov=table2array(track(1:element(1),6));
xycov=table2array(track(1:element(1),7));


%% seprarte cone classes
i2=0;
i3=0;
i4=0;
for i=1:element(1)
    switch class(i)
        case 'car_start'
            Start.x = x(i);
            Start.y = y(i);
        case 'yellow'
            i2=i2+1;
            yellowCone.x(i2) = x(i);
            yellowCone.y(i2) = y(i);
        case 'blue'
            i3=i3+1;
            blueCone.x(i3) = x(i);
            blueCone.y(i3) = y(i);
        case 'big_orange'
            i4=i4+1;
            orangeCone.x(i4) = x(i);
            orangeCone.y(i4)= y(i);
        case 'inactive_noise'
            %ignore this
    end
end

% approximate middle point between cones
midPoint = ApproxMidpoint(yellowCone,blueCone);

%% plot track
figure
set(gcf,'position',[200,100,1000,800])
plot(yellowCone.x,yellowCone.y,'.','Color',[0.8,0.6,0])
hold on
plot(blueCone.x,blueCone.y,'.','Color',[0,0,1])
plot(orangeCone.x,orangeCone.y,'.','MarkerSize',10,'Color',[0.9,0.4,0])
plot(Start.x,Start.y,'x','MarkerSize',10,'Color',[0.2,1,0.2])
plot(midPoint.x,midPoint.y,'-','MarkerSize',10,'Color',[1,0,0])
title('track')
xlabel('x [m]')
ylabel('y [m]')
legend('yellow cones','blue cones','big orange cones','starting posiiton','midpoint approximation')