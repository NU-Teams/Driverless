function [midPoint] = ApproxMidpoint(yellowCone,blueCone)
%APPROXMIDPOINT Summary of this function goes here
%   Detailed explanation goes here
    for i=1:length(yellowCone.x)
        k = dsearchn([blueCone.x.',blueCone.y.'],[yellowCone.x(i),yellowCone.y(i)]);
        midPoint.x(i)=(yellowCone.x(i)+blueCone.x(k))/2;
        midPoint.y(i)=(yellowCone.y(i)+blueCone.y(k))/2;
    end
end

