function [trajectory] = simple_trajectory_generator(key_points, num_points)
    % SIMPLE_TRAJECTORY_GENERATOR generates a straight-line trajectory
    % between key points in sequence.
    %
    % Inputs:
    %   key_points - Nx2 or Nx3 array of key points (2D or 3D space)
    %   num_points - Number of points to generate between each key point
    %
    % Outputs:
    %   trajectory - Mx2 or Mx3 array of interpolated trajectory points

    % Check the dimensions of key_points
    [num_key_points, dim] = size(key_points);
    if dim < 2 || dim > 3
        error('Key points must be in 2D or 3D space.');
    end

    % Initialize the trajectory
    trajectory = [];

    % Generate trajectory segments between each pair of key points
    for i = 1:num_key_points-1
        % Extract the start and end points of the current segment
        start_point = key_points(i, :);
        end_point = key_points(i+1, :);

        % Generate linearly spaced points along the line segment
        segment = zeros(num_points, dim);
        for j = 1:dim
            segment(:, j) = linspace(start_point(j), end_point(j), num_points).';
        end
        
        % Remove duplicate points between segments
        if i > 1
            segment = segment(2:end, :);
        end

        % Append the segment to the trajectory
        trajectory = [trajectory; segment];
    end
end
