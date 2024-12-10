clear
clc
close all

tic

%Read and store all point clouds
folderPath = 'D:\rosbags\livox_room.bag';
bag = rosbag(folderPath);

pointCloudTopic = '/livox/lidar';  % Replace with the correct topic name
pointCloudBag = select(bag, 'Topic', pointCloudTopic);

% Read all the PointCloud messages
pcdMessages = readMessages(pointCloudBag);
disp(length(pcdMessages));

% Initialize a list to store point clouds
pointCloudList = cell(1, length(pcdMessages));

% We have a cell with all the Point Clouds
for i = 1:length(pcdMessages)
    pcMsg = pcdMessages{i};
    
    % Extract point cloud data (x, y, z)
    pcData = readXYZ(pcMsg);  % Read the XYZ data from the PointCloud2 message
    
    % Convert the point cloud data to a pointCloud object
    pcObject = pointCloud(pcData);
    
    % Store the pointCloud object in the list
    pointCloudList{i} = pcObject;
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Downsample all point clouds (before doing it we remove the points above 'height'
voxelSize = 0.2;
height = 1.8;
pointCloudsDownsampled = downsampling_function(voxelSize, pointCloudList, height);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%We generate a sphere for the trajectory
% Number of points
numPoints = 0;

% Radius of the sphere
radiusSphere = 0.2;
pointCloudSphere = generateSphere(numPoints, radiusSphere);

%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Initialization of deque
dequeSize = 10;
dequeCloud = cell(1, dequeSize);
dequeCurrentSize = 0;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Initialization of clouds
alignedCloud = pointCloudsDownsampled{1}; %alignedCloud is the current cloud aligned
accumulatedCloud = alignedCloud; % Last 10 clouds aligned and combined
showCloud = alignedCloud; %showCloud is all the point clouds already aligned, the final .pcd 

dequeCloud{1} = alignedCloud;
dequeCurrentSize = dequeCurrentSize + 1;
disp('Iteration: 1');


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Parameters for ICP
maxDistance = 2; % Max correspondence distance
maxIterations = 300; % Maximum number of iterations
transformationEpsilon = 1e-6; % Transformation epsilon for convergence


radiusFilter = 0.1;

initialTransform = rigid3d(eye(3), [0, 0, 0]); % Default: identity transform
initialTransformDeque = rigid3d(eye(3), [0, 0, 0]); % Default: identity transform

for i = 2:length(pcdMessages)

    inputCloud = pointCloudsDownsampled{i};

    %Add inputCloud at the end of the deque
    dequeCloud{mod(i-1,10)+1} = inputCloud;
    dequeCurrentSize = dequeCurrentSize + 1;

    [tformDeque, dequeCloud{mod(i-1,10)+1}, rmseDeque] = pcregistericp(dequeCloud{mod(i-1,10)+1},accumulatedCloud, ...
         'Metric', 'pointToPoint', ... % ICP point-to-point alignment
         'MaxIterations', maxIterations, ...
         'Tolerance', [transformationEpsilon, 0.01], ...
         'InlierRatio', 1,...
         'InitialTransform', initialTransformDeque);

    initialTransformDeque = tformDeque;

    dequeCloud{mod(i-1,10)+1} = filter_radius(accumulatedCloud, dequeCloud{mod(i-1,10)+1}, radiusFilter);
    accumulatedCloud = pointCloud([accumulatedCloud.Location; dequeCloud{mod(i-1,10)+1}.Location]);
    
    disp(['Iteration: ', num2str(i)]);

    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    % % % Set up ICP registration
    [tformAlignment, alignedCloud, rmse] = pcregistericp(inputCloud, accumulatedCloud, ...
    'Metric', 'pointToPoint', ... % ICP point-to-point alignment
    'MaxIterations', maxIterations, ...
    'Tolerance', [transformationEpsilon, 0.01], ...
    'InlierRatio', 1.0,...
    'InitialTransform', initialTransform);

    initialTransform = tformAlignment;
    %%%%%%%%%%%%%%%%%%%

    transformedPC = pctransform(pointCloudSphere, tformAlignment);

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    alignedCloud = filter_radius(showCloud, alignedCloud, radiusFilter);
    showCloud = pointCloud([showCloud.Location; alignedCloud.Location; transformedPC.Location]);

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


    if dequeCurrentSize == 10
        
        dequeCurrentSize = dequeCurrentSize - 1;
        size1 = dequeCloud{mod(i-1,10)+1}.Count;
        size2 = accumulatedCloud.Count;
        accumulatedCloud = pointCloud(accumulatedCloud.Location(size1+1:end,:));

    end
   
end

toc

pcshow(showCloud);
