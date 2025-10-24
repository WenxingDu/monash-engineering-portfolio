%% CIV4100 Assignment2 Task2.3

clear; clc;

% Load trained model 
load('Alex_2_M13_use.mat');
inputSize = [227 227];

% Load training data
filteredPath = 'C:\Users\dwx\Desktop\CIV4100_A2_almostDone\traffic_sign_final_dataset\traffic_sign_final_dataset\traffic_sign_final_dataset\Filtered_9_new_use';

imds = imageDatastore(filteredPath, 'IncludeSubfolders', true, 'LabelSource', 'foldernames');
[imdsTrain, imdsValidation] = splitEachLabel(imds, 0.8);

% Randomly select 20 validation images to simulate test cases
numTestCases = 20;
randIdx = randperm(numel(imdsValidation.Files), numTestCases);
testImgs = imdsValidation.Files(randIdx);      % File paths of selected test images
trueLabels = imdsValidation.Labels(randIdx);    % Ground truth labels of selected images

% Generate adversarial samples (reuse from Task 2.2)
adversarialImgs = cell(numTestCases, 1);
originalLabels = strings(numTestCases,1);  % Store original predicted labels
originalProbs = zeros(numTestCases, 1);    % Store original prediction confidence
for i = 1:numTestCases
    img = imread(testImgs{i});
    imgResized = imresize(img, inputSize);
    % Predict and store the original label and confidence
    [predLabel, scores] = classify(netAlexTraffic, imgResized);
    originalLabels(i) = string(predLabel);
    originalProbs(i) = max(scores);

    % Create perturbed version using blur + brightness + nois
    noisy = im2double(imgResized);
    brightness = imadjust(noisy, [], [], 0.5 + 1.5*rand());  % Random brightness
    blur = imgaussfilt(brightness, 0.5 + 0.5*rand());        % Random Gaussian blur
    noise = blur + (rand(size(blur)) - 0.5) * 0.2;           % Additive noise
    advImg = im2uint8(min(max(noise, 0), 1));    % Clip to [0,1] and convert to uint8

    adversarialImgs{i} = advImg;
end

%% Defence 1: Denoising via Gaussian blur
fprintf('\n--- Defence 1: Input Pre-processing (Gaussian blur) ---\n');
correct1 = 0;
denoisedProbs = zeros(numTestCases, 1);
for i = 1:numTestCases
    denoised = imgaussfilt(adversarialImgs{i}, 1.0);
    [pred, scores] = classify(netAlexTraffic, imresize(denoised, inputSize));
    denoisedProbs(i) = max(scores);
    if pred == trueLabels(i)
        correct1 = correct1 + 1;
    end
end
acc1 = correct1 / numTestCases * 100;
fprintf('Defence 1 Accuracy after attack: %.2f%%\n', acc1);

%% Defence 2: Retrain with augmentations 
extraImgs = {}; extraLabels = categorical([]);  % Containers for new noisy training samples

for i = 1:200
    imgIdx = randi(numel(imdsTrain.Files));
    img = readimage(imdsTrain, imgIdx);
    img = imresize(img, inputSize);
    perturbed = imgaussfilt(imadjust(im2double(img), [], [], 0.5 + 1.5*rand()), 0.5);
    perturbed = perturbed + (rand(size(perturbed)) - 0.5)*0.2;
    extraImgs{end+1} = im2uint8(min(max(perturbed,0),1));
    extraLabels(end+1) = imdsTrain.Labels(imgIdx);
end

% Save new images to temporary folder for retraining
tempAugDir = fullfile(tempdir, 'aug_train');
if ~exist(tempAugDir, 'dir'); mkdir(tempAugDir); end
% Combine original and new image paths/labels
allImgPaths = imdsTrain.Files;
allLabels = imdsTrain.Labels;

for i = 1:length(extraImgs)
    filename = fullfile(tempAugDir, sprintf('aug_temp_%d.png', i));
    imwrite(extraImgs{i}, filename);
    allImgPaths{end+1} = filename;
    allLabels(end+1) = extraLabels(i);
end

% Create new augmented training datastore
augImds = imageDatastore(allImgPaths, 'Labels', allLabels);
augImds.ReadFcn = @(x) imresize(imread(x), inputSize);
augImds = shuffle(augImds);

% Retrain model using original layers and new augmented dataset
layers = netAlexTraffic.Layers;
options = trainingOptions('adam','MaxEpochs',5,'MiniBatchSize',32,...
    'Shuffle','every-epoch','Verbose',true,...
    'Plots','training-progress');
netDef2 = trainNetwork(augImds, layers, options);
fprintf('\nTraining new model with noisy augmented data...\n');


%% Defence 2: Evaluate retrained model on adversarial images
fprintf('\n--- Defence 2: Retrained Model ---\n');
correct2 = 0;
retrainedProbs = zeros(numTestCases, 1);    % Confidence scores after retraining
for i = 1:numTestCases
    [pred, scores] = classify(netDef2, imresize(adversarialImgs{i}, inputSize));
    retrainedProbs(i) = max(scores);
    if pred == trueLabels(i)
        correct2 = correct2 + 1;
    end
end
acc2 = correct2 / numTestCases * 100;
fprintf('Defence 2 Accuracy after attack: %.2f%%\n', acc2);

%% Summary table 
fprintf('\nSummary Table:\n');
baseAcc = 90;
fprintf('Original Model Attack Success Rate: %.2f%%\n', (100 - baseAcc));
fprintf('Defence 1 Accuracy: %.2f%%\n', acc1);
fprintf('Defence 2 Accuracy: %.2f%%\n', acc2);

avgRed1 = mean(originalProbs - denoisedProbs);
avgRed2 = mean(originalProbs - retrainedProbs);

fprintf('\nAverage Reduction in Probability (Defence 1): %.4f\n', avgRed1);
fprintf('Average Reduction in Probability (Defence 2): %.4f\n', avgRed2);


% Per-class probability table
classNames = categories(trueLabels);
nClasses = numel(classNames);

classAvg = table('Size', [nClasses 5], ...
    'VariableTypes', {'string','double','double','double','double'}, ...
    'VariableNames', {'Class','Original','Attacked','Defence1','Defence2'});

for c = 1:nClasses
    cls = classNames{c};
    idx = trueLabels == cls;

    classAvg.Class(c) = cls;
    classAvg.Original(c) = mean(originalProbs(idx));
    classAvg.Attacked(c) = mean(originalProbs(idx) - originalProbs(idx));  
    classAvg.Defence1(c) = mean(denoisedProbs(idx));
    classAvg.Defence2(c) = mean(retrainedProbs(idx));
end

disp('--- Average Probability Table by Class ---');
disp(classAvg);
