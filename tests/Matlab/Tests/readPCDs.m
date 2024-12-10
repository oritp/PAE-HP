function [pointClouds, pcdFiles] = readPCDs(folderPath)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here
% Get a list of all .pcd files in the folder
pcdFiles = dir(fullfile(folderPath, '*.pcd'));

% Initialize a cell array to store the point cloud objects
pointClouds = cell(length(pcdFiles), 1);

% Loop through each .pcd file
for i = 1:length(pcdFiles)
    % Get the full file path
    filePath = fullfile(folderPath, pcdFiles(i).name);
    
    % Read the .pcd file as a point cloud object
    pointClouds{i} = pcread(filePath); % Use pcread for MATLAB's PointCloud object
    
end

% Display the number of point clouds read
disp(['Number of .pcd files read: ', num2str(length(pointClouds))]);
end