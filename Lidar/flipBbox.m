function gtLabels = flipBbox(gtLabels,tform)
% Function supports the flipping of 3-D bounding box along any one of the
% X, Y, and Z-axis.
    if isequal(tform.T(1,1),-1)
        gtLabels(:,1) = -gtLabels(:,1);
    elseif isequal(tform.T(2,2),-1)
        gtLabels(:,2) = -gtLabels(:,2);
        gtLabels(:,9) = -gtLabels(:,9);
    elseif isequal(tform.T(3,3),-1)
        gtLabels(:,3) = -gtLabels(:,3);
    end
end