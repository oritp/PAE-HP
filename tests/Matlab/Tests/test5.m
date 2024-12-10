% Number of points
numPoints = 1000;

% Radius of the sphere
radius = 0.2;

% Generate random angles for spherical coordinates
theta = 2 * pi * rand(numPoints, 1); % Azimuthal angle (0 to 2*pi)
phi = acos(2 * rand(numPoints, 1) - 1); % Polar angle (0 to pi)

% Convert spherical coordinates to Cartesian coordinates
x = radius * sin(phi) .* cos(theta);
y = radius * sin(phi) .* sin(theta);
z = radius * cos(phi);

% Combine x, y, and z into a point cloud
points = [x, y, z];

% Create the point cloud object
pc = pointCloud(points);

% Visualize the point cloud
pcshow(pc);