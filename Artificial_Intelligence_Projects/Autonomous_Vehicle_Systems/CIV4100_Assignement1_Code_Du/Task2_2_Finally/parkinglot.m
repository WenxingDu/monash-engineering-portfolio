figure
hold on

line([1; 1], [6; 22])
line([1; 6], [6; 6])
line([1; 6], [10; 10])
line([1; 6], [14; 14])
line([1; 6], [18; 18])

line([1; 6], [22; 22])
line([12; 17], [6; 6])
line([12; 17], [10; 10])
line([12; 17], [14; 14])
line([12; 17], [18; 18])
line([17; 17], [6; 22])
line([17; 12], [22; 22])

rectangle('Position', [2 7.1 2.4 1.8])
rectangle('Position', [2 11.1 2.4 1.8])
rectangle('Position', [2 19.1 2.4 1.8])

rectangle('Position', [13 7.1 2.4 1.8])
rectangle('Position', [13 19.1 2.4 1.8])

rectangle('Position', [8.1 7.8 1.8 2.4])

% initial position
rectangle('Position', [0 2.1 2.4 1.8])

axis([0 25 0 25])
axis equal

wp = [0.2 20 20 8 8 13.2; 3 3 22 22 16 16; 0 pi/2 pi 3*pi/2 2*pi 2*pi];

plot(wp(1,:), wp(2,:), 'x')

xlabel('X (m)')
ylabel('Y (m)')

