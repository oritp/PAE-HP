function [filteredCloud] = filter_radius(accumulatedCloud, inputCloud, radius)
    % Filter points to avoid adding points too close to existing ones
    % accumulatedCloud: the existing accumulated cloud
    % inputCloud: the new input cloud
    % radius: the radius threshold for filtering

    % Get the points from the input cloud
    newCloudPoints = inputCloud.Location;

    % Handle empty accumulatedCloud
    if isempty(accumulatedCloud.Location)
        % If no points exist in the accumulated cloud, return all points
        filteredCloud = inputCloud;
        return;
    end

    % Build k-d tree for the accumulated cloud
    kdtree = KDTreeSearcher(accumulatedCloud.Location);

    % Find nearest neighbors and their distances
    [~, distances] = knnsearch(kdtree, newCloudPoints, 'K', 1);

    % Retain only points farther than the radius
    filteredPoints = newCloudPoints(distances > radius, :);

    % Check if filteredPoints is empty
    if isempty(filteredPoints)
        % Create an empty point cloud
        filteredCloud = pointCloud(zeros(0, 3));
    else
        % Create a point cloud from the filtered points
        filteredCloud = pointCloud(filteredPoints);
    end
end