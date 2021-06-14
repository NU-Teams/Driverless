function [pillarFeatures,pillarIndices,labels] = helperCreateBatchData(...
    features,indices,groundTruthBoxes,groundTruthClasses,classNames)
% Return pillar features and indices combined along the batch dimension
% and bounding boxes concatenated along batch dimension in labels.
    
    % Concatenate features and indices along batch dimension.
    pillarFeatures = cat(4,features{:,1});
    pillarIndices = cat(4,indices{:,1});
    
    % Get class IDs from the class names.
    classNames = repmat({categorical(classNames')},size(groundTruthClasses));
    [~,classIndices] = cellfun(@(a,b)ismember(a,b),groundTruthClasses,...
        classNames,'UniformOutput',false);
    
    % Append the class indices and create a single array of responses.
    combinedResponses = cellfun(@(bbox,classid) [bbox,classid],...
        groundTruthBoxes,classIndices,'UniformOutput',false);
    len = max(cellfun(@(x)size(x,1),combinedResponses));
    paddedBBoxes = cellfun(@(v) padarray(v,[len-size(v,1),0],0,'post'),...
        combinedResponses,'UniformOutput',false);
    labels = cat(4,paddedBBoxes{:,1});
end