clear
clc

%Read and store all point clouds
folderPath = 'D:\pcds_full';
[pointClouds, pcdFiles] = readPCDs(folderPath);

%Downsample all point clouds
voxelSize = 0.4;
downsampledPc = downsampling_func(voxelSize, pointClouds, pcdFiles);

%%
clc
close all
clearvars dequeCloud

% Initialize the deque (point cloud buffer) if not already done
dequeSize = 10;
dequeCloud = cell(1, dequeSize);
dequeCurrentSize = 0;
%accumulatedCloud;

alignedCloud = downsampledPc{1}; %aligned_cloud is ALL the clouds
accumulatedCloud = alignedCloud; % Last 10 clouds aligned
accumulatedCloud2 = accumulatedCloud;
dequeCloud{1} = alignedCloud;
dequeCurrentSize = dequeCurrentSize + 1;

% figure
% pcshow(alignedCloud)
% hold on

%pause(1);

% Parameters for ICP
maxDistance = 2; % Max correspondence distance
maxIterations = 100; % Maximum number of iterations
transformationEpsilon = 1e-6; % Transformation epsilon for convergence

pcshow(accumulatedCloud);
drawnow;
hold on
%pause(1);

for i = 340:360

    previousCloud = downsampledPc{i-1};
    inputCloud = downsampledPc{i}; % Example new point cloud

    % Set up ICP registration
    [tform, alignedCloud, rmse] = pcregistericp(inputCloud, accumulatedCloud, ...
    'Metric', 'pointToPoint', ... % ICP point-to-point alignment
    'MaxIterations', maxIterations, ...
    'Tolerance', [transformationEpsilon, 0.01], ...
    'InlierRatio', 1.0);

    % Visualize the result
    pcshow(alignedCloud);
    drawnow;

    accumulatedCloud = alignedCloud;
    disp(['Iteration: ', num2str(i)]);
   

end






%%
% 
% 
% 
% 
% 
% 
% 
% 
% 
% pc = [pointClouds{1}.Location;pointClouds{340}.Location];
% combinedPc = pointCloud(pc);
% %pcshow(combinedPc);
% 
% % Get the number of points in pointClouds{1}
% numPointsInPc1 = size(pointClouds{1}.Location, 1);
% 
% % Remove the points corresponding to pointClouds{1} from combinedPc
% remainingPoints = combinedPc.Location(numPointsInPc1+1:end, :);
% 
% % Create a new point cloud with the remaining points
% newPc = pointCloud(remainingPoints);
% 
% pcshow(newPc);
% 
% 
% % figure
% % %ptCloud = pcread('table_scene_lms400.pcd');
% % pcshow(pointClouds{1});
% % hold on
% % pcshow(pointClouds{340});