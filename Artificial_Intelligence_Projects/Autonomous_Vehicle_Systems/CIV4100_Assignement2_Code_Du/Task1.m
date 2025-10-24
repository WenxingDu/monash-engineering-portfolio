%% CIV4100 Assignment 2 Task1

clear all; close all; clc;

%% Path 
trainPath = "C:\Users\dwx\Desktop\CIV4100_2\traffic_sign_final_dataset\traffic_sign_final_dataset\traffic_sign_final_dataset\Train";
filteredPath = "C:\Users\dwx\Desktop\CIV4100_2\traffic_sign_final_dataset\traffic_sign_final_dataset\traffic_sign_final_dataset\Filtered_9";
metaPath = "C:\Users\dwx\Desktop\CIV4100_2\traffic_sign_final_dataset\traffic_sign_final_dataset\traffic_sign_final_dataset\Meta";    
Kaggle_set = "C:\Users\dwx\Desktop\CIV4100_2\Kaggle_dataset\Kaggle_dataset\Kaggle_dataset";

%% Task1.1
% Loop through all 43 traffic sign classes (labels 0 to 42)
for classID = 0:42
    originalDir = fullfile(trainPath, num2str(classID));
    outputDir = fullfile(filteredPath, num2str(classID));

    % Create folder if it doesn't exist; if it does, clear it; so I can
    % train many times to choose the best model
    if ~exist(outputDir, 'dir')
        mkdir(outputDir);
    else
        delete(fullfile(outputDir, '*.png'));  
    end

    % Load and sort original image filenames by numeric order
    imgFiles = dir(fullfile(originalDir, '*.png'));
    fileNames = {imgFiles.name};
    numericIDs = cellfun(@(f) sscanf(f, '%d.png'), fileNames);
    [~, sortOrder] = sort(numericIDs);
    imgFiles = imgFiles(sortOrder);    % Reorder image file list

    % Copy sorted images to filtered directory
    for i = 1:length(imgFiles)
        img = imread(fullfile(originalDir, imgFiles(i).name));
        imwrite(img, fullfile(outputDir, imgFiles(i).name));
    end

    fprintf('Class %d: Copied %d sorted images.\n', classID, length(imgFiles));
end

fprintf('Initial filtering completed.\n');

%% Task 1.1 Data augmentation
for classID = 0:42
    classFolder = fullfile(filteredPath, num2str(classID));
    metaImageFile = fullfile(metaPath, sprintf('%d.png', classID));

    % Apply random gamma darkening to 200 randomly selected images
    currentImgs = dir(fullfile(classFolder, '*.png'));
    totalImgs = numel(currentImgs);
    darkenCount = 200;
    selected = randperm(totalImgs, darkenCount);

    for i = 1:darkenCount
        img = imread(fullfile(classFolder, currentImgs(selected(i)).name));
        gamma = 1.2 + rand() * 1.3;  % Generate random gamma value in [1.2,2.5]
        adjusted = imadjust(img, [], [], gamma); % Apply gamma correction
        imwrite(adjusted, fullfile(classFolder, currentImgs(selected(i)).name)); % Overwrite
    end
    fprintf('Class %d: Applied gamma correction to %d images.\n', classID, darkenCount);

    % Read the meta image and create 100 copies
    metaImg = imread(metaImageFile);
    for k = 1:100
        outName = sprintf('meta_%02d_copy_%03d.png', classID, k);
        imwrite(metaImg, fullfile(classFolder, outName));
    end
    fprintf('Class %d: Added 100 meta image copies.\n', classID);

    % Add more transformed images (blurred + brightness adjusted) until
    % reaching 1000 images for every class
    currentTotal = numel(dir(fullfile(classFolder, '*.png')));
    newIndex = 1;

    while currentTotal < 1000
        blurSigma = rand([2, 6]);  % Choose random blur strength
        blurred = imgaussfilt(metaImg, blurSigma);

        brightnessGamma = 0.2 + rand() * (3 - 0.2);  % Random brightness gamma in [0.2, 3]
        transformed = imadjust(blurred, [], [], brightnessGamma);

        newName = sprintf('gen_%02d_aug_%03d.png', classID, newIndex);
        imwrite(transformed, fullfile(classFolder, newName));

        currentTotal = currentTotal + 1;   % Update image count
        newIndex = newIndex + 1;   % Update augmentation index
    end

    fprintf('Class %d: Generated %d synthetic images to reach 1000.\n', classID, newIndex - 1);
end

fprintf('All classes processed. Dataset augmentation complete.\n');



%% Task 1.2 Transfer Learning

% Set up image datastore with folder-based labeling
ds = imageDatastore(filteredPath, 'IncludeSubfolders', true, 'LabelSource', 'foldernames');

% Count the number of images per class 
classDist = countEachLabel(ds);

% Count the number of classes
classes = categories(ds.Labels);
numLabels = numel(classes);

% Resize to match AlexNet input: 227x227x3
targetSize = [227, 227, 3];

% Split dataset (80% train / 20% validation)
[dsTrain, dsVal] = splitEachLabel(ds, 0.8, 'randomized');
fprintf('Train set: %d images, Validation set: %d images\n', ...
    numel(dsTrain.Files), numel(dsVal.Files));

% Define image augmentation parameters
augmentor = imageDataAugmenter( ...
    'RandXShear', 10*[-1 1], ...        % Random shear in X axis (±10°)
    'RandYShear', 10*[-1 1], ...        % Random shear in Y axis
    'RandXTranslation', [-2 2], ...   % Translate X by ±2 pixels
    'RandYTranslation', [-2 2], ...   % Translate Y by ±2 pixels
    'RandXScale', [0.9 1.1], ...        % Scale X by 90% to 110%
    'RandYScale', [0.9 1.1]);           % Scale Y by 90% to 110%

% Apply augmentation to training and validation sets
augTrain = augmentedImageDatastore(targetSize(1:2), dsTrain, 'DataAugmentation', augmentor);
augVal = augmentedImageDatastore(targetSize(1:2), dsVal);

% Load pretrained AlexNet
netPre = alexnet;

% Remove the last 3 layers (original classification head)
baseLayers = netPre.Layers(1:end-3);  
customHead = [
    fullyConnectedLayer(numLabels, ...
    'WeightLearnRateFactor', 10, ...
    'BiasLearnRateFactor', 10, ...
    'Name', 'fc_custom')
    softmaxLayer('Name', 'softmax')
    classificationLayer('Name', 'classOutput')
];
netLayers = [baseLayers; customHead];

% Set training options for transfer learning
trainOpts = trainingOptions('adam', ...
    'InitialLearnRate', 1e-4, ...            % Start with small learning rate
    'LearnRateSchedule', 'piecewise', ...    % Drop learning rate periodically
    'LearnRateDropFactor', 0.02, ...
    'LearnRateDropPeriod', 5, ...            % Drop every 5 epochs
    'MaxEpochs', 25, ...                     % Total training epochs
    'MiniBatchSize', 32, ...                 % Batch size per iteration
    'Shuffle', 'every-epoch', ...
    'ValidationData', augVal, ...
    'ValidationFrequency', 300, ...          % Validate every 300 iterations
    'Plots', 'training-progress', ...
    'Verbose', true, ...
    'ExecutionEnvironment', 'gpu');          % % Use GPU if available

% Shuffle the random seed for reproducibility
rng('shuffle');

% Begin training
fprintf('Starting training using transfer learning...\n');
netFinal = trainNetwork(augTrain, netLayers, trainOpts);

% Save model to file
save('Alex_2_M13.mat', 'netFinal');
disp('Model saved !');

% Evaluate model on validation set
% Compute accuracy
preds = classify(netFinal, augVal);
trueLabels = dsVal.Labels;
valAccuracy = mean(preds == dsVal.Labels);   
fprintf('Validation Accuracy: %.4f (%.2f%%)\n', valAccuracy, valAccuracy*100);

% Build confusion matrix to compute precision and F1-score
cm = confusionmat(trueLabels, preds);
numClasses = size(cm, 1);

% Initialize metric vectors
precision = zeros(numClasses, 1);
recall = zeros(numClasses, 1);
f1 = zeros(numClasses, 1);

for i = 1:numClasses
    TP = cm(i, i);
    FP = sum(cm(:, i)) - TP;
    FN = sum(cm(i, :)) - TP;
    precision(i) = TP / (TP + FP + eps);  % Avoid division by zero
    recall(i) = TP / (TP + FN + eps);
    f1(i) = 2 * precision(i) * recall(i) / (precision(i) + recall(i) + eps);
end

% Compute macro-averaged precision and F1-score
avgPrecision = mean(precision);
avgF1 = mean(f1);
fprintf('Macro Precision: %.4f\n', avgPrecision);
fprintf('Macro F1-score: %.4f\n', avgF1);


%% Task 1.3
% Define label names from label.m
classLabels = { ...
    '20 km/h', '30 km/h', '50 km/h', '60 km/h', '70 km/h', ...
    '80 km/h', '80 km/h end', '100 km/h', '120 km/h', 'No overtaking', ...
    'No overtaking for trucks', 'Crossroad with secondary way', 'Main road', ...
    'Give way', 'Stop', 'Road up', 'Road up for truck', 'Brock', ...
    'Other dangerous', 'Turn left', 'Turn right', 'Winding road', ...
    'Hollow road', 'Slippery road', 'Narrowing road', 'Roadwork', ...
    'Traffic light', 'Pedestrian', 'Children', 'Bike', 'Snow', 'Deer', ...
    'End of the limits', 'Only right', 'Only left', 'Only straight', ...
    'Only straight and right', 'Only straight and left', 'Take right', ...
    'Take left', 'Circle crossroad', 'End of overtaking limit', ...
    'End of overtaking limit for truck' ...
};

% Load Kaggle evaluation dataset
Kaggle_set = imageDatastore(kaggleRoot, ...
    'IncludeSubfolders', false, ...
    'FileExtensions', '.png');

% Create an augmented image datastore for test images (resize only)
augTestSet = augmentedImageDatastore(targetSize(1:2), Kaggle_set, ...
    'ColorPreprocessing', 'none');

% Get sorted class order from trained network (sorted by numeric class name)
trainedClassNames = categories(netFinal.Layers(end).Classes);
numericClass = str2double(trainedClassNames);
[~, sortIdx] = sort(numericClass);
sortedClassNames = trainedClassNames(sortIdx);  % ensures '0', '1', ..., '42'

% Predict labels for all test images
predCats = classify(netFinal, augTestSet);  % returns categorical values

% Map predicted categorical labels to readable traffic sign strings
[~, classIndices] = ismember(predCats, sortedClassNames);
predictedLabels = classLabels(classIndices);

% Extract original image indices from filenames (e.g. '12.png' → 12)
allFilenames = Kaggle_set.Files;
[~, nameOnly, ~] = cellfun(@fileparts, allFilenames, 'UniformOutput', false);
imgIdx = str2double(nameOnly);

% Build output table: [Image index | Label]
submission = table(imgIdx, predictedLabels(:), ...
    'VariableNames', {'Image index', 'Label'});
submission = sortrows(submission, 'Image index');  % sort by image order

% Save as CSV 
outputFile = 'submission_final_M13.csv';
writetable(submission, outputFile);

fprintf(' Submission file saved as: %s\n', outputFile);
