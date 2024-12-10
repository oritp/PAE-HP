function [downsampledPc] = downsampling_func(voxelSize,pointClouds,pcdFiles)
%UNTITLED3 Summary of this function goes here
%   Detailed explanation goes here

% Specify the voxel size (larger size = more aggressive downsampling)
%voxelSize = 0.5; % Adjust based on desired resolution

downsampledPc = cell(length(pcdFiles), 1);
    for i = 1:length(pcdFiles)
        % Apply the voxel grid filter
        downsampledPc{i} = pcdownsample(pointClouds{i}, 'gridAverage', voxelSize);

        % % Assuming your point cloud is stored in a variable named pc
        % points = downsampledPc{i}.Location; % Extract the point locations (Nx3 matrix)
        % 
        % % Set the z-coordinate (3rd column) to 0
        % points(:, 3) = 0;
        % 
        % % Create a new point cloud with the modified points
        % collapsedPc = pointCloud(points);
        % 
        % downsampledPc{i} = collapsedPc;
    end

end