%% CIV4100 Assignment2 Task2.2

clear; clc;

% Load trained model 
load('Alex_2_M13_use.mat');
inputSize = [227 227];

% Load training data (Task 1.1 subset)
filteredPath = 'C:\Users\dwx\Desktop\CIV4100_A2_almostDone\traffic_sign_final_dataset\traffic_sign_final_dataset\traffic_sign_final_dataset\Filtered_9_new_use';  % Replace with actual path
imds = imageDatastore(filteredPath, 'IncludeSubfolders', true, 'LabelSource', 'foldernames');
[imdsTrain, ~] = splitEachLabel(imds, 0.8);  % Use training data for attack

% Select test samples from training set
numTestCases = 20;
randIdx = randperm(numel(imdsTrain.Files), numTestCases);
testImgs = imdsTrain.Files(randIdx);          % Get file paths
trueLabels = imdsTrain.Labels(randIdx);       % Get true labels

% Initialize attack success counter and output strings
successCount = 0;
attackResults = strings(numTestCases, 1);

fprintf('\n--- Task 2.2: Simulated Adversarial Attacks ---\n');


for i = 1:numTestCases
    % Load and preprocess image
    img = imread(testImgs{i});
    imgResized = imresize(img, inputSize);

    % Predict the original label before attack
    originalLabel = classify(netAlexTraffic, imgResized);

    % Apply composite perturbation (blur + brightness + noise)
    noisy = im2double(imgResized);
    brightness = imadjust(noisy, [], [], 0.5 + 1.5*rand());
    blur = imgaussfilt(brightness, 0.5 + 0.5*rand());
    noise = blur + (rand(size(blur)) - 0.5) * 0.2;
    advImg = im2uint8(min(max(noise, 0), 1));

    % Predict perturbed label
    advLabel = classify(netAlexTraffic, advImg);

    % Compare results
    if advLabel ~= originalLabel
        successCount = successCount + 1;
        attackResults(i) = sprintf('[%02d] Success: %s \x2192 %s', i, string(originalLabel), string(advLabel));
    else
        attackResults(i) = sprintf('[%02d] Unchanged: %s', i, string(originalLabel));
    end
end

%% Print the classification outcomes for all test cases
for i = 1:numTestCases
    fprintf('%s\n', attackResults(i));
end

fprintf('\nAttack Success Rate: %.2f%%\n', successCount / numTestCases * 100);

%% Visualize some adversarial examples for comparison
figure;
tiledlayout(2, 4);
for i = 1:4
    img = imread(testImgs{i});
    imgResized = imresize(img, inputSize);
    noisy = im2double(imgResized);
    brightness = imadjust(noisy, [], [], 0.5 + 1.5*rand());
    blur = imgaussfilt(brightness, 0.5 + 0.5*rand());
    noise = blur + (rand(size(blur)) - 0.5) * 0.2;
    advImg = im2uint8(min(max(noise, 0), 1));

    nexttile; imshow(imgResized); title('Original');
    nexttile; imshow(advImg); title('Perturbed');
end
