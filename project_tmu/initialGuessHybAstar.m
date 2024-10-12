function P = initialGuessHybAstar(param)

%% Create the costmap
box = param.constraint.xy;
mapWidth  = max(box(:,1)) - min(box(:,1));
mapHeight = max(box(:,2)) - min(box(:,2));
resolution = 25;
map = binaryOccupancyMap(mapWidth, mapHeight, resolution);

% Generate mesh grid of world coordinates
[xWorld, yWorld] = meshgrid(linspace(0, mapWidth, resolution * mapWidth), ...
    linspace(0, mapHeight, resolution * mapHeight));

% Iterate over all ellipses to set occupancy
Mask = zeros(resolution * mapWidth, resolution * mapHeight);

for i = 1:length(param.constraint.ellipses)
    % Ellipse parameters
    xc = param.constraint.ellipses{i}.xc;
    yc = param.constraint.ellipses{i}.yc;
    a = param.constraint.ellipses{i}.a;
    b = param.constraint.ellipses{i}.b;

    % Center the coordinates to ellipse frame
    xCentered = xWorld - xc;
    yCentered = yWorld - yc;

    % Check if points are inside the ellipse (no rotation)
    ellipseMask = (xCentered.^2 / a^2) + (yCentered.^2 / b^2) <= 1;
    Mask = Mask + ellipseMask;
end
% Set occupancy in the map
setOccupancy(map, [xWorld(:), yWorld(:)], Mask(:));

%% Create a state space
space = stateSpaceSE2;
space.WeightXY = 1;
space.WeightTheta = 0.5;

%% Creat a validatorOccupancyMap
validator = validatorOccupancyMap(space);
validator.Map = map;
validator.XYIndices = [1,2];
 
%% Creat a HybridAStar planner
planner = plannerHybridAStar(validator);
planner.MinTurningRadius = 0.75;
planner.ForwardCost = 1;
planner.ReverseCost = 1;

startLoc = [param.start(2:3) param.start(1)];
goalLoc = [param.target(2:3) param.target(1)];

refpath = plan(planner,startLoc,goalLoc,SearchMode='exhaustive'); 
show(planner);

%% Interpolate path to obtain N evenly spaced points
path = refpath.States;
N = param.N + 1; % Number of points for the new path P

% Calculate cumulative arc length of the path
dists = sqrt(diff(path(:,1)).^2 + diff(path(:,2)).^2);
arcLength = [0; cumsum(dists)];

% Generate uniform arc length values for interpolation
uniformArcLength = linspace(0, arcLength(end), N);

% Interpolate x and y coordinates based on the uniform arc length
Px = spline(arcLength, path(:,1), uniformArcLength);
Py = spline(arcLength, path(:,2), uniformArcLength);

% Combine the interpolated x and y coordinates into the new path P
P = [Px', Py'];

% Obtain the estimated orientation
ortVec = diff(P);
Phi = zeros(N, 1);
for i = 1:N-1
Phi(i+1, 1) = atan2(ortVec(i,2), ortVec(i,1)); 
end

% Combine the interpolated phi, x and y 
P = [Phi P];

% Display the interpolated PRM Path
figure;
show(map);
hold on;
plot(startLoc(1), startLoc(2), 'go', 'MarkerSize', 10, 'LineWidth', 2);
plot(goalLoc(1), goalLoc(2), 'ro', 'MarkerSize', 10, 'LineWidth', 2);
plot(P(:,2), P(:,3), 'b-.', 'LineWidth', 2);
xlabel('X');
ylabel('Y');
title('Interpolated Hybrid A*');
hold off;
end