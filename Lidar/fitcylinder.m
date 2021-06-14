addpath test
data=pcread('cloud000001.pcd')

groundPtsIdx = segmentGroundFromLidarData(data);
data = select(data,~groundPtsIdx,'OutputSize','full');
figure
pcshow(data)


maxDistance = 100;
referenceVector = [0,0,1];
maxAngularDistance=10;
[model,inlierIndices] = pcfitcylinder(data,maxDistance,referenceVector,maxAngularDistance,'Confidence',1);
pc = select(data,inlierIndices);

figure
pcshow(pc)
title('Cylinder Point Cloud')
xlabel('X(m)')
ylabel('Y(m)')
zlabel('Z(m)')