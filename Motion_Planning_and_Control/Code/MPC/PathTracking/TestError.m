clear all

pp = spline([0 1],[0,1]);
eval = [linspace(0,100,1000);linspace(0,100,1000)];
pathPoints = ppval(pp,eval);

T = 4;
