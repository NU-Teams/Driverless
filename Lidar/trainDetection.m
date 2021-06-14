%  https://www.mathworks.com/help/lidar/ug/object-detection-using-pointpillars-network.html
clc
clear
addpath Functions/Detection

tempdir = 'Dataset';
%% Download Lidar Data Set
outputFolder = fullfile(tempdir,'Pandaset');
% lidarURL = ['https://ssd.mathworks.com/supportfiles/lidar/data/' ...
%     'Pandaset_LidarData.tar.gz'];
% helperDownloadPandasetData(outputFolder,lidarURL);

%% Download Pretrained Network
pretrainedNetURL = ['https://ssd.mathworks.com/supportfiles/lidar/data/' ...
    'trainedPointPillarsPandasetNet.zip'];

doTraining = true;
if ~doTraining
    helperDownloadPretrainedPointPillarsNet(outputFolder,pretrainedNetURL);
end

%% load data
disp("Loading data...")
path = fullfile(outputFolder,'Lidar');
lidarData = fileDatastore(path,'ReadFcn',@(x) pcread(x));

gtPath = fullfile(outputFolder,'Cuboids','PandaSetLidarGroundTruth.mat');
data = load(gtPath,'lidarGtLabels');
Labels = timetable2table(data.lidarGtLabels);
boxLabels = Labels(:,2:3);

figure
ptCld = read(lidarData);
ax = pcshow(ptCld.Location);
set(ax,'XLim',[-50 50],'YLim',[-40 40]);
zoom(ax,2.5);
axis off;

reset(lidarData);

%% preprocess data
disp("Preprocessing data...")
xMin = 0.0;     % Minimum value along X-axis.
yMin = -39.68;  % Minimum value along Y-axis.
zMin = -5.0;    % Minimum value along Z-axis.
xMax = 69.12;   % Maximum value along X-axis.
yMax = 39.68;   % Maximum value along Y-axis.
zMax = 5.0;     % Maximum value along Z-axis.
xStep = 0.16;   % Resolution along X-axis.
yStep = 0.16;   % Resolution along Y-axis.
dsFactor = 2.0; % Downsampling factor.

% Calculate the dimensions for the pseudo-image.
Xn = round(((xMax - xMin) / xStep));
Yn = round(((yMax - yMin) / yStep));

% Define the pillar extraction parameters.
gridParams = {{xMin,yMin,zMin},{xMax,yMax,zMax},{xStep,yStep,dsFactor},{Xn,Yn}};

[croppedPointCloudObj,processedLabels] = cropFrontViewFromLidarData(...
    lidarData,boxLabels,gridParams);

pc = croppedPointCloudObj{1,1};
gtLabelsCar = processedLabels.Car{1};
gtLabelsTruck = processedLabels.Truck{1};

helperDisplay3DBoxesOverlaidPointCloud(pc.Location,gtLabelsCar,...
   'green',gtLabelsTruck,'magenta','Cropped Point Cloud');

reset(lidarData);

%% Create Datastore Objects for Training
disp("Creating Datastore...")
rng(1);
shuffledIndices = randperm(size(processedLabels,1));
idx = floor(0.7 * length(shuffledIndices));

trainData = croppedPointCloudObj(shuffledIndices(1:idx),:);
testData = croppedPointCloudObj(shuffledIndices(idx+1:end),:);

trainLabels = processedLabels(shuffledIndices(1:idx),:);
testLabels = processedLabels(shuffledIndices(idx+1:end),:);

writeFiles = true;
dataLocation = fullfile(outputFolder,'InputData');
[trainData,trainLabels] = saveptCldToPCD(trainData,trainLabels,...
    dataLocation,writeFiles);

lds = fileDatastore(dataLocation,'ReadFcn',@(x) pcread(x));

bds = boxLabelDatastore(trainLabels);

cds = combine(lds,bds);

%% data augmentation
disp("Data Augmentation")
augData = read(cds);
augptCld = augData{1,1};
augLabels = augData{1,2};
augClass = augData{1,3};

labelsCar = augLabels(augClass=='Car',:);
labelsTruck = augLabels(augClass=='Truck',:);

helperDisplay3DBoxesOverlaidPointCloud(augptCld.Location,labelsCar,'green',...
    labelsTruck,'magenta','Before Data Augmentation');
reset(cds);

gtData = generateGTDataForAugmentation(trainData,trainLabels);

samplesToAdd = struct('Car',10,'Truck',10);
cdsAugmented = transform(cds,@(x) groundTruthDataAugmenation(x,gtData,samplesToAdd));

cdsAugmented = transform(cdsAugmented,@(x) augmentData(x));

augData = read(cdsAugmented);
augptCld = augData{1,1};
augLabels = augData{1,2};
augClass = augData{1,3};

labelsCar = augLabels(augClass=='Car',:);
labelsTruck = augLabels(augClass=='Truck',:);

helperDisplay3DBoxesOverlaidPointCloud(augptCld(:,1:3),labelsCar,'green',...
    labelsTruck,'magenta','After Data Augmentation');

reset(cdsAugmented);

%% Extract Pillar Information
% Define number of prominent pillars.
P = 12000; 

% Define number of points per pillar.
N = 100;   
cdsTransformed = transform(cdsAugmented,@(x) createPillars(x,gridParams,P,N));

anchorBoxes = calculateAnchorsPointPillars(trainLabels);
numAnchors = size(anchorBoxes,2);
classNames = trainLabels.Properties.VariableNames;
numClasses = numel(classNames);

lgraph = pointpillarNetwork(numAnchors,gridParams,P,N,numClasses);

%% Specify Training Options
numEpochs = 60;
miniBatchSize = 2;
learningRate = 0.0002;
learnRateDropPeriod = 15;
learnRateDropFactor = 0.8;
gradientDecayFactor = 0.9;
squaredGradientDecayFactor = 0.999;
trailingAvg = [];
trailingAvgSq = [];

%% train model
executionEnvironment = "auto";
if canUseParallelPool
    dispatchInBackground = true;
else
    dispatchInBackground = false;
end

mbq = minibatchqueue(...
    cdsTransformed,3,...
    "MiniBatchSize",miniBatchSize,...
    "OutputEnvironment",executionEnvironment,...
    "MiniBatchFcn",@(features,indices,boxes,labels) ...
    helperCreateBatchData(features,indices,boxes,labels,classNames),...
    "MiniBatchFormat",["SSCB","SSCB",""],...
    "DispatchInBackground",true);

if doTraining
    % Convert layer graph to dlnetwork.
    net = dlnetwork(lgraph);
    
    % Initialize plot.
    fig = figure;
    lossPlotter = helperConfigureTrainingProgressPlotter(fig);    
    iteration = 0;
       
    % Custom training loop.
    for epoch = 1:numEpochs
        disp("Epoch:"+epoch)
        % Reset datastore.
        reset(mbq);
        while(hasdata(mbq))
            iteration = iteration + 1;
            
            % Read batch of data.
            [pillarFeatures,pillarIndices,boxLabels] = next(mbq);
                        
            % Evaluate the model gradients and loss using dlfeval 
            % and the modelGradients function.
            [gradients,loss,state] = dlfeval(@modelGradients,net,...
                pillarFeatures,pillarIndices,boxLabels,gridParams,...
                anchorBoxes,numClasses,executionEnvironment);
            
            % Do not update the network learnable parameters if NaN values
            % are present in gradients or loss values.
            if helperCheckForNaN(gradients,loss)
                continue;
            end
                    
            % Update the state parameters of dlnetwork.
            net.State = state;
            
            % Update the network learnable parameters using the Adam
            % optimizer.
            [net.Learnables,trailingAvg,trailingAvgSq] = ...
                adamupdate(net.Learnables,gradients,trailingAvg,...
                trailingAvgSq,iteration,learningRate,...
                gradientDecayFactor,squaredGradientDecayFactor);
            
            % Update training plot with new points.         
            addpoints(lossPlotter,iteration,double(gather(extractdata(loss))));
            title("Training Epoch " + epoch +" of " + numEpochs);
            drawnow;
        end
        
        % Update the learning rate after every learnRateDropPeriod.
        if mod(epoch,learnRateDropPeriod) == 0
            learningRate = learningRate * learnRateDropFactor;
        end
    end
else
    preTrainedMATFile = fullfile(outputFolder,'trainedPointPillarsPandasetNet.mat');
    pretrainedNetwork = load(preTrainedMATFile,'net');
    net = pretrainedNetwork.net;
end

%% Evaluate model
numInputs = numel(testData);

% Generate rotated rectangles from the cuboid labels.
bds = boxLabelDatastore(testLabels);
groundTruthData = transform(bds,@(x) createRotRect(x));

% Set the threshold values.
nmsPositiveIoUThreshold = 0.5;
confidenceThreshold = 0.25;
overlapThreshold = 0.1;

% Set numSamplesToTest to numInputs to evaluate the model on the entire
% test data set.
numSamplesToTest = 50;
detectionResults = table('Size',[numSamplesToTest 3],...
                        'VariableTypes',{'cell','cell','cell'},...
                        'VariableNames',{'Boxes','Scores','Labels'});

for num = 1:numSamplesToTest
    ptCloud = testData{num,1};
    
    [box,score,labels] = generatePointPillarDetections(net,ptCloud,anchorBoxes,...
        gridParams,classNames,confidenceThreshold,overlapThreshold,...
        P,N,executionEnvironment);
 
    % Convert the detected boxes to rotated rectangle format.
    if ~isempty(box)
        detectionResults.Boxes{num} = box(:,[1,2,4,5,7]);
    else
        detectionResults.Boxes{num} = box;
    end
    detectionResults.Scores{num} = score;
    detectionResults.Labels{num} = labels;
end

metrics = evaluateDetectionAOS(detectionResults,groundTruthData,...
    nmsPositiveIoUThreshold);
disp(metrics(:,1:2))

%% Generate detection
ptCloud = testData{45,1};
gtLabels = testLabels(45,:);

% The generatePointPillarDetections function detects the 
% bounding boxes, and scores for a given point cloud.
confidenceThreshold = 0.5;
overlapThreshold = 0.1;
[box,score,labels] = generatePointPillarDetections(net,ptCloud,anchorBoxes,...
    gridParams,classNames,confidenceThreshold,overlapThreshold,P,N,...
    executionEnvironment);

boxlabelsCar = box(labels'=='Car',:);
boxlabelsTruck = box(labels'=='Truck',:);

% Display the predictions on the point cloud.
helperDisplay3DBoxesOverlaidPointCloud(ptCloud.Location,boxlabelsCar,'green',...
    boxlabelsTruck,'magenta','Predicted Bounding Boxes');