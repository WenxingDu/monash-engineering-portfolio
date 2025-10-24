% Task1.3

% Load the occupancy map from Task 1.1
load('task1_1_map.mat')

% Create a copy of the map and inflate it
inflatedMap = copy(map);
inflate(inflatedMap, 1.1);

% Define the state space (x, y, orientation)
ss = stateSpaceSE2;
ss.StateBounds = [0 25; 0 25; -pi pi];  % Set state boundaries

% Create a state validator
sv = validatorOccupancyMap(ss); % Create a map validator corresponding to the state space
sv.Map = inflatedMap;           % Use the inflated map for collision detection
sv.ValidationDistance = 0.1;    % To improve accuracy

% Set up the Hybrid A* planner
planner = plannerHybridAStar(sv);

% Try adjusting the following three parameters
planner.NumMotionPrimitives = 15;         
planner.AnalyticExpansionInterval = 4;    
planner.MotionPrimitiveLength = 2;     

% Set the start and end points
start = [0.2, 3.0, 0]; 
goal = [13.2, 16.0, 0];


% Set a random number
% pthObj: path object
% solnInfo: including path search tree information
% tic and toc for timing
rng(1); tic;
[pthObj, solnInfo] = plan(planner, start, goal);
planningTime = toc;

% Extract the state point sequence of the path
% vecnorm()represents Euclidean distance
% sum() gets the total length of the path
numPoints = size(pthObj.States, 1);
pathLength = sum(vecnorm(diff(pthObj.States(:,1:2)), 2, 2));

% Plot with tree and path 
% All expanded nodes (orange arrows)
% Final path (brown-red line)
% Add title
figure;
show(planner);
title(sprintf(['Task 1.3: Hybrid A* Path\n' ...
    'NumMotionPrimitives = %.0f, AnalyticExpansionInterval = %.0f, MotionPrimitiveLength = %.1f m\n' ...
    'Points: %d | Length: %.2f m | Time: %.2fs'], ...
    planner.NumMotionPrimitives, ...
    planner.AnalyticExpansionInterval, ...
    planner.MotionPrimitiveLength, ...
    numPoints, pathLength, planningTime));

xlim([-5, 25])
ylim([0, 25])