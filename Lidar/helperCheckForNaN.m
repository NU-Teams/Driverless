function retValue = helperCheckForNaN(gradients,loss)
% The last convolution head 'occupancy|conv2d' is known to contain NaNs 
% the gradients. This function checks whether gradient values contain 
% NaNs. Add other convolution head values to the condition if NaNs 
% are present in the gradients. 
    gradValue = gradients.Value((gradients.Layer == 'occupancy|conv2d') & ...
        (gradients.Parameter == 'Bias'));
    if (sum(isnan(extractdata(loss)),'all') > 0) || ...
            (sum(isnan(extractdata(gradValue{1,1})),'all') > 0)
        retValue = true;
    else
        retValue = false;
    end
end