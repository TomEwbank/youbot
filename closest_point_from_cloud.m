function [p, min_dist] = closest_point_from_cloud(cloud)
% From a cloud of points taken by a depth sensor in a single shot, returns
% the closest point to the sensor.
%
% ARGUMENTS 
%   - cloud :       a 4xN matrix where the first 3 rows are the coordinates
%                   [y z x] points in the cloud, and the last row
%                   contains the distances between these points and the
%                   sensor
% OUTPUT
%   - p :           the coordinates [x y z] of the closest point to the
%                   sensor
%   - min_dist :    distance between the sensor and the closest point

p = [];
min_dist = 10;
for i = 1:length(cloud(1,:))
    if cloud(4,i) < min_dist
        p = [cloud(3,i);cloud(1,i);cloud(2,i)];
        min_dist = cloud(4,i);
    end
end

end

