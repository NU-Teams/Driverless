function [gradients,loss,state] = modelGradients(net,pillarFeatures,...
    pillarIndices,boxLabels,gridParams,anchorBoxes,...
    numClasses,executionEnvironment)
      
    numAnchors = size(anchorBoxes,2);
    
    % Extract the predictions from the network.
    YPredictions = cell(size(net.OutputNames));
    [YPredictions{:},state] = forward(net,pillarIndices,pillarFeatures);
    
    % Generate target for predictions from the ground truth data.
    YTargets = generatePointPillarTargets(YPredictions,boxLabels,pillarIndices,...
        gridParams,anchorBoxes,numClasses);
    YTargets = cellfun(@ dlarray,YTargets,'UniformOutput',false);
    if (executionEnvironment=="auto" && canUseGPU) || executionEnvironment=="gpu"
        YTargets = cellfun(@ gpuArray,YTargets,'UniformOutput',false);
    end
     
    [angLoss,occLoss,locLoss,szLoss,hdLoss,clfLoss] = ...
        computePointPillarLoss(YPredictions,YTargets,gridParams,...
        numClasses,numAnchors);
    
    % Compute the total loss.
    loss = angLoss + occLoss + locLoss + szLoss + hdLoss + clfLoss;
    
    % Compute the gradients of the learnables with regard to the loss.
    gradients = dlgradient(loss,net.Learnables);
 
end