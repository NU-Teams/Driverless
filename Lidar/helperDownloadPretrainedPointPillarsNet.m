function helperDownloadPretrainedPointPillarsNet(outputFolder,pretrainedNetURL)
% Download the pretrained PointPillars network.

    preTrainedMATFile = fullfile(outputFolder,'trainedPointPillarsPandasetNet.mat');
    preTrainedZipFile = fullfile(outputFolder,'trainedPointPillarsPandasetNet.zip');
    
    if ~exist(preTrainedMATFile,'file')
        if ~exist(preTrainedZipFile,'file')
            disp('Downloading pretrained detector (8.4 MB)...');
            websave(preTrainedZipFile,pretrainedNetURL);
        end
        unzip(preTrainedZipFile,outputFolder);   
    end       
end