function [obj] = setupobject(curPose)
            obj.LookaheadPoint = zeros(1,2, 'like', curPose);
            obj.LastPose = zeros(1,3, 'like', curPose);
            obj.ProjectionPoint = nan(1,2, 'like', curPose);
            obj.ProjectionLineIndex = cast(0, 'like', curPose);
end
