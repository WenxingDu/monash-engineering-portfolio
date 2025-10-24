%% CIV4100 Assignment2 Task2.1

load('Alex_2_M13_use.mat'); 

inputSize = [227 227];

% Load images from the filtered dataset path
filteredPath = 'C:\Users\dwx\Desktop\CIV4100_A2_almostDone\traffic_sign_final_dataset\traffic_sign_final_dataset\traffic_sign_final_dataset\Filtered_9_new_use';
imds = imageDatastore(filteredPath, 'IncludeSubfolders', true, 'LabelSource', 'foldernames');
[~, imdsValidation] = splitEachLabel(imds, 0.8);

% Randomly select 20 samples from validation set for testing
numTestCases = 20;
randIdx = randperm(numel(imdsValidation.Files), numTestCases);
testImgs = imdsValidation.Files(randIdx);
trueLabels = imdsValidation.Labels(randIdx);  % Corresponding ground truth labels

% Initialize correct prediction counter
correct = 0;

fprintf('\n--- Traditional Testing ---\n');
for i = 1:numTestCases
    img = imread(testImgs{i});
    resizedImg = imresize(img, inputSize(1:2));      % Resize to AlexNet input
    label = classify(netAlexTraffic, resizedImg);    % Predict label
    
    if label == trueLabels(i)
        correct = correct + 1;
    else
        fprintf('[%02d] Mismatch - GT: %s, Pred: %s\n', i, string(trueLabels(i)), string(label));
    end
end

% Print traditional accuracy
fprintf('Traditional Test Accuracy = %.2f%%\n', correct/numTestCases*100);


% Define metamorphic transformation function: blur + brightness
function imgOut = metamorphicTransform(img)
    imgOut = imgaussfilt(img, 2 + 2*rand());               % Apply Gaussian blur (in range[2,4])
    imgOut = imadjust(imgOut, [], [], 0.5 + rand()*1.5);   %  Adjust brightness using gamma in range[0.5, 2.0]
end

% Initialize counter
violationCount = 0;

fprintf('\n--- Metamorphic Testing ---\n');
for i = 1:numTestCases
    img = imread(testImgs{i});
    imgResized = imresize(img, inputSize(1:2));    % Resize original image
    label1 = classify(netAlexTraffic, imgResized);

    imgTransformed = metamorphicTransform(img);
    imgTransformedResized = imresize(imgTransformed, inputSize(1:2));
    label2 = classify(netAlexTraffic, imgTransformedResized);   % Classify transformed image

    % If labels are inconsistent, count as metamorphic violation
    if label1 ~= label2
        violationCount = violationCount + 1;
        fprintf('[%02d] Violation - Original: %s, Transformed: %s\n', i, string(label1), string(label2));
    end
end

% Print metamorphic consistency
fprintf('Metamorphic Consistency = %.2f%%\n', (1 - violationCount/numTestCases)*100);

% Function: apply only Gaussian blur with random intensity
function out = transformWithBlur(img)
    out = imgaussfilt(img, 2 + 2*rand());
end

% Function: apply only brightness transformation via gamma correction
function out = transformWithBrightness(img)
    out = imadjust(img, [], [], 0.5 + rand()*1.5);
end

% Function: horizontal flip
function out = transformWithFlip(img)
    out = flip(img, 2);  
end

% Function: random rotation between -10 and 10 degrees
function out = transformWithRotation(img)
    angle = randi([-10, 10]);  
    out = imrotate(img, angle, 'bilinear', 'crop');  % Keep same image size after rotation
end


% Define a list of transformation functions and their names
transformFuncs = {@transformWithBlur, @transformWithBrightness, @transformWithFlip, @transformWithRotation};
transformNames = {'Blur', 'Brightness', 'Flip', 'Rotation'};

% For each transformation, apply and test for label consistency
for tfIdx = 1:length(transformFuncs)
    f = transformFuncs{tfIdx};
    name = transformNames{tfIdx};
    fprintf('\n--- %s Test ---\n', name);

    violationCount = 0;
    for i = 1:numTestCases
        img = imread(testImgs{i});
        imgResized = imresize(img, inputSize(1:2));
        label1 = classify(netAlexTraffic, imgResized);   % Label before transformation

        imgTransformed = f(img);    % Apply selected transformation
        imgTransformedResized = imresize(imgTransformed, inputSize(1:2));
        label2 = classify(netAlexTraffic, imgTransformedResized);    % Label after transformation

 
        if label1 ~= label2
            violationCount = violationCount + 1;
            fprintf('[%02d] Violation - Original: %s, Transformed: %s\n', i, string(label1), string(label2));
        end
    end
    
    % Print transformation-specific consistency score
    fprintf('%s Consistency = %.2f%%\n', name, (1 - violationCount/numTestCases)*100);
end


