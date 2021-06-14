function updatedPtCloud = removeIndicesFromPtCloud(ptCloud,indices)
% To remove the specified indices from the point cloud.
    updatedLocation = ptCloud.Location;
    updatedLocation(indices,:) = [];
    updatedIntensity = ptCloud.Intensity;
    updatedIntensity(indices,:) = [];
    updatedPtCloud = pointCloud(updatedLocation);   
    updatedPtCloud.Intensity = updatedIntensity;
end