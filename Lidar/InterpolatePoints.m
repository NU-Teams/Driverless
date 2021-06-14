clc
clear

% load point cloud
addpath test
pcd=pcread('cloud000001.pcd');
[pcd,~] = removeInvalidPoints(pcd)
x=double(pcd.Location(:,1));
y=double(pcd.Location(:,2));
v=double(pcd.Location(:,3));

% create interpolate object
F = scatteredInterpolant(x,y,v)
[xq,yq] = meshgrid(-25:0.05:25);
F.Method = 'natural';

% interpolate using the set grid
vq1 = F(xq,yq);

figure
plot3(x,y,v,'mo')
hold on
mesh(xq,yq,vq1)
axis([-25 25 -25 25 -25 25])
title('Mesh interpolation from point cloud')
legend('Points','Interpolated Surface')