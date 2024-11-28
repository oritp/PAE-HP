%%
clc
close all
clearvars -except downsampledPc

% Initialize the deque (point cloud buffer) if not already done
dequeSize = 3;
dequeCloud = cell(1, 3);
tform = cell(1, 3);
dequeCurrentSize = 0;
%accumulatedCloud;

% Parameters for ICP
maxDistance = 2; % Max correspondence distance
maxIterations = 100; % Maximum number of iterations
transformationEpsilon = 1e-6; % Transformation epsilon for convergence

cloud1 = downsampledPc{1};
cloud2 = downsampledPc{340};
cloud3 = downsampledPc{280};

cloud4 = downsampledPc{395};

%pcshow(cloud2);

%alignedCloud = cloud1;

%for i=1:3
     [tform{1}, alignedCloud, rmse] = pcregistericp(cloud2, cloud1, ...
    'Metric', 'pointToPoint', ... % ICP point-to-point alignment
    'MaxIterations', maxIterations, ...
    'Tolerance', [transformationEpsilon, 0.01], ...
    'InlierRatio', 1.0);
%end
    accumulatedCloud = pointCloud([cloud1.Location; alignedCloud.Location]);
    %pcshow(accumulatedCloud);
    

    [tform{2}, alignedCloud, rmse2] = pcregistericp(cloud3, accumulatedCloud, ...
    'Metric', 'pointToPoint', ... % ICP point-to-point alignment
    'MaxIterations', maxIterations, ...
    'Tolerance', [transformationEpsilon, 0.01], ...
    'InlierRatio', 1.0);

    accumulatedCloud = [accumulatedCloud.Location; alignedCloud.Location];
    pcshow(accumulatedCloud);


    % figure
    % pcshow(cloud1);
    % drawnow;
    % hold on
    %pcshow(accumulatedCloud);
    figure
    cloud2new = pctransform(cloud2, tform{1});
    cloud3new = pctransform(cloud3, tform{2});
    accumulatedCloud2 = pointCloud([cloud1.Location; cloud2new.Location; cloud3new.Location]);
    pcshow(accumulatedCloud2);

    
    



%  figure
%  pcshow(alignedCloud);
% 
%  figure
% % cloud3 = pctransform(cloud2,tform);
%  pcshow(cloud3);