clear
clc

%Read and store all point clouds
folderPath = 'D:\pcds_full';
[pointClouds, pcdFiles] = readPCDs(folderPath);

%Downsample all point clouds
voxelSize = 0.2;
downsampledPc = downsampling_func(voxelSize, pointClouds, pcdFiles);

%%
clc
close all
clearvars dequeCloud
tic

trajectory = zeros(395, 3); % Assuming 395 iterations, adjust as needed
cumulativeTransform = rigid3d(eye(3), [0, 0, 0]); % Start with identity transform

% Initialize the deque (point cloud buffer) if not already done
dequeSize = 1;
dequeCloud = cell(1, dequeSize);
dequeCurrentSize = 0;
tform = cell(1, dequeSize);
%accumulatedCloud;

alignedCloud = downsampledPc{1}; %aligned_cloud is the current cloud aligned
accumulatedCloud = alignedCloud; % Last 10 clouds aligned and combined

showCloud = alignedCloud;
%%%%%%%%%%%%%%%
    %previousCloud = downsampledPc{i-1};
    %inputCloud = downsampledPc{i}; % Example new point cloud

    points = showCloud.Location;

    % Filter points where z = 1
    z_filter = points(:, 3) < 1.8; % Logical index for points with z = 1
    filteredPoints = points(z_filter, :);
    showCloud = pointCloud(filteredPoints);
    
%%%%%%%%%%%%%%%%


dequeCloud{1} = alignedCloud;
dequeCurrentSize = dequeCurrentSize + 1;

% figure
% pcshow(alignedCloud)
% hold on

%pause(1);

% Parameters for ICP
maxDistance = 2; % Max correspondence distance
maxIterations = 300; % Maximum number of iterations
transformationEpsilon = 1e-6; % Transformation epsilon for convergence

%pcshow(accumulatedCloud);
% drawnow;
% hold on
%pause(1);

radius = 0.1;

initialTransform = rigid3d(eye(3), [0, 0, 0]); % Default: identity transform
initialTransform3 = rigid3d(eye(3), [0, 0, 0]); % Default: identity transform

for i = 2:395

    %previousCloud = downsampledPc{i-1};
    inputCloud = downsampledPc{i}; % Example new point cloud

    points = inputCloud.Location;

    % Filter points where z = 1
    z_filter = points(:, 3) < 1.8; % Logical index for points with z = 1

    % Create a new point cloud with the filtered points
    filteredPoints = points(z_filter, :);
    inputCloud = pointCloud(filteredPoints);

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    %Add inputCloud at the end of the deque
    dequeCloud{mod(i-1,10)+1} = inputCloud;
    dequeCurrentSize = dequeCurrentSize + 1;

    % [tform{mod(i-1,10)+1}, dequeCloud{mod(i-1,10)+1}, rmse2] = pcregistericp(dequeCloud{mod(i-1,10)+1},accumulatedCloud, ...
    %      'Metric', 'pointToPoint', ... % ICP point-to-point alignment
    %      'MaxIterations', maxIterations, ...
    %      'Tolerance', [transformationEpsilon, 0.01], ...
    %      'InlierRatio', 1);
     [tform3, dequeCloud{mod(i-1,10)+1}, rmse2] = pcregistericp(dequeCloud{mod(i-1,10)+1},accumulatedCloud, ...
         'Metric', 'pointToPoint', ... % ICP point-to-point alignment
         'MaxIterations', maxIterations, ...
         'Tolerance', [transformationEpsilon, 0.01], ...
         'InlierRatio', 1,...
         'InitialTransform', initialTransform3);

    % disp(size(dequeCloud{mod(i-1,10)+1});

    initialTransform3 = tform3;

    dequeCloud{mod(i-1,10)+1} = filter_radius(accumulatedCloud, dequeCloud{mod(i-1,10)+1}, radius);
    accumulatedCloud = pointCloud([accumulatedCloud.Location; dequeCloud{mod(i-1,10)+1}.Location]);
    
    disp(['Iteration: ', num2str(i)]);
    
    % pcshow(accumulatedCloud);
    % hold on
    % drawnow;
    %pause(1);
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    % % % Set up ICP registration
    [tform2, alignedCloud, rmse] = pcregistericp(inputCloud, accumulatedCloud, ...
    'Metric', 'pointToPoint', ... % ICP point-to-point alignment
    'MaxIterations', maxIterations, ...
    'Tolerance', [transformationEpsilon, 0.01], ...
    'InlierRatio', 1.0,...
    'InitialTransform', initialTransform);

    % if i==2
    % disp(size(inputCloud.Location, 1));
    % disp(size(accumulatedCloud.Location, 1));
    % disp(size(alignedCloud.Location, 1));
    % end

    initialTransform = tform2;

    alignedCloud = filter_radius(showCloud, alignedCloud, radius);
    showCloud = pointCloud([showCloud.Location; alignedCloud.Location]);



    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


    if dequeCurrentSize == 10
        
        dequeCurrentSize = dequeCurrentSize - 1;
        size1 = dequeCloud{mod(i-1,10)+1}.Count;
        size2 = accumulatedCloud.Count;
        accumulatedCloud = pointCloud(accumulatedCloud.Location(size1+1:end,:));

    end

   % pcshow(showCloud);
   % drawnow;
   % hold on
   

end
pcshow(showCloud);
toc




%%

points = showCloud.Location;

% Filter points where z = 1
z_filter = points(:, 3) >= 0 & points(:, 3) <= 0.1; % Logical index for points with z = 1

% Create a new point cloud with the filtered points
filteredPoints = points(z_filter, :);
newPointCloud = pointCloud(filteredPoints);

% Visualize the new point cloud
figure;
pcshow(newPointCloud);