function [pointCloudsDownsampled] = downsampling_function(voxelSize, pointCloudList, height)

pointCloudsDownsampled = cell(length(pointCloudList), 1);
    for i = 1:length(pointCloudList)
        %We filter points by height
            points = pointCloudList{i}.Location;

            % Filter out points from the roof
            z_filter = points(:, 3) < height; % Logical index for points with z = 1
            filteredPoints = points(z_filter, :);
            pointCloudList{i} = pointCloud(filteredPoints);


        % Apply the voxel grid filter
        pointCloudsDownsampled{i} = pcdownsample(pointCloudList{i}, 'gridAverage', voxelSize);
    end

end