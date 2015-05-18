function [p, min_dist] = closest_point_from_cloud(cloud)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here

p = [];
min_dist = 10;
for i = 1:length(cloud(1,:))
    %dist = sqrt((cloud(1,i)-pose(1))^2+(cloud(2,i)-pose(2))^2+(cloud(3,i)-pose(3))^2);
    if cloud(4,i) < min_dist
        p = [cloud(3,i);cloud(1,i);cloud(2,i)];
        min_dist = cloud(4,i);
    end
end

end

