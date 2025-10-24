% Task1.2

% isValidStart = isStateValid(sv, start) 
% Visual verification of the starting point
% figure;
% show(inflatedMap)
% hold on
% plot(start(1), start(2), 'ro', 'MarkerSize', 10, 'LineWidth', 2)
% title('Check if Start State is Valid')

% load the occupancy map generated in Task 1.1
load('task1_1_map.mat')  

% Expand the obstacle boundary, considering the vehicle size
% here expand 1.1 meters
inflatedMap = copy(map);
inflate(inflatedMap, 1.1);  % 1.1m

% set up the state space (2D plane)
ss = stateSpaceSE2;
ss.StateBounds = [0 25; 0 25; -pi pi];  % Define the range of x, y, theta 

% Create stateValidator
% Used to detect whether the state in the path is in free space
sv = validatorOccupancyMap(ss);
sv.Map = inflatedMap;
sv.ValidationDistance = 0.1;  % Minimum detection interval between states

% RRT* planner
planner = plannerRRTStar(ss, sv);  
planner.ContinueAfterGoalReached = true;    % Continue to optimize the path after finding the target
planner.MaxIterations = 5000;               % Maximum number of iterations
planner.MaxConnectionDistance = 0.3;        % The maximum connection distance between nodes 
                                            % (affects the branch density of the tree)

% Set starting and goal positions 
start = [0.2, 3.0, 0];     
goal  = [13.2, 16.0, 0];      

% Planning
rng(0);  % Fixed random number seed to ensure reproducible results
tic;     % Start timing
[pthObj, solnInfo] = plan(planner, start, goal);
planningTime = toc;     % Get the time taken for path planning (in seconds)


% Show the figure
figure;
show(inflatedMap);     
hold on;

% Show the established treeestablished treeestablished tree
plot(solnInfo.TreeData(:,1), solnInfo.TreeData(:,2), '.-', ...
     'Color', [0.3, 0.7, 1.0], 'LineWidth', 0.5);

% Show paths
plot(pthObj.States(:,1), pthObj.States(:,2), 'r-', 'LineWidth', 2)

% Mark start point and  goal point
plot(start(1), start(2), 'go', 'MarkerSize', 10, 'LineWidth', 2)
plot(goal(1), goal(2), 'bx', 'MarkerSize', 10, 'LineWidth', 2)

% Number of points and length of path
numPoints = size(pthObj.States, 1);
pathLength = sum(vecnorm(diff(pthObj.States(:,1:2)), 2, 2));

% Title
title(sprintf('Task 1.2: RRT* Path | Points: %d | Length: %.2f m | Time: %.2fs', ...
     numPoints, pathLength, planningTime));
xlim([-5 25])
ylim([0 25])
legend('Tree', 'Path', 'Start', 'Goal')









