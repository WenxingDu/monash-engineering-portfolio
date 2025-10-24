% Task 1.1


% Initialize map parameters

mapWidth = 25;    % Map Width
mapHeight = 25;   % Map Height
resolution = 10;  % Each grid represents 0.1 meters

% Create empty binaryOccupancyMap
map = binaryOccupancyMap(mapWidth, mapHeight, resolution);

% Draw the stop lines
% Parking space border line on the left
% Bottom border,outer border,top border
for y = [6, 10, 14, 18]
    setOccupancy(map, [1:0.1:6; y*ones(1, length(1:0.1:6))]', 1);
end
setOccupancy(map, [1*ones(1, length(6:0.1:22)); 6:0.1:22]', 1);
setOccupancy(map, [1:0.1:6; 22*ones(1, length(1:0.1:6))]', 1);

% Parking space border line on the right
for y = [6, 10, 14, 18]
    setOccupancy(map, [12:0.1:17; y*ones(1, length(12:0.1:17))]', 1);
end
setOccupancy(map, [17*ones(1, length(6:0.1:22)); 6:0.1:22]', 1);
setOccupancy(map, [12:0.1:17; 22*ones(1, length(12:0.1:17))]', 1);


% Drawing obstacle vehicles
markRectangle(map, 2, 7.1, 2.4, 1.8);   % left 3 cars
markRectangle(map, 2, 11.1, 2.4, 1.8);
markRectangle(map, 2, 19.1, 2.4, 1.8);

markRectangle(map, 13, 7.1, 2.4, 1.8);  % right 2 cars
markRectangle(map, 13, 19.1, 2.4, 1.8);

markRectangle(map, 8.1, 7.8, 1.8, 2.4); % middle car


% Visual Map

figure;
show(map)
title('Task 1.1: Binary Occupancy Map')
xlabel('X (m)')
ylabel('Y (m)')
xlim([-5, 25])
ylim([0, 25])

save('task1_1_map.mat', 'map')

% "Brush a rectangle" on the map to represent an obstacle vehicle
function markRectangle(map, x, y, w, h)
   
    xs = x : 0.05 : (x + w);       % From left to right
    ys = y : 0.05 : (y + h);       % From bottom to top
    [X, Y] = meshgrid(xs, ys);
    setOccupancy(map, [X(:), Y(:)], 1);  % Set the entire rectangular area to be occupied
end



