function [plots] = InitOverview()
%INITOVERVIEW Summary of this function goes here
%   Detailed explanation goes here
figure('Renderer', 'painters', 'Position', [400 100 900 700])

subplot(2,2,1)
plots.CameraPlot=imshow(zeros(785,785,3));
title('Cam 1')

subplot(2,2,2)
plots.LidarPlot=scatter3(0,0,0,'.','r');
title('Lidar 1')

dim = [.15 .3 .2 .2];
str="";
plots.SensorAnno=annotation('textbox',dim,'String',str,'FitBoxToText','on')
end

