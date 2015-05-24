function index = find_closest_point(trajectory, x, y)
% From a trajectory, returns the index of the closest point to a specified one
%
% ARGUMENTS 
%   - trajectory :  a Nx2 matrix containing the coordinates of the points 
%                   composing a trajectory
%   - x :           the x coordinate of the point to wich we search the
%                   closest in the trajectory
%   - y :           the y coordinate of the point to wich we search the
%                   closest in the trajectory
%
% OUTPUT
%   - index :       the index of the closest point to the

min_dist = 100;
    
for i = 1:length(trajectory(1,:))
    
    x_star = trajectory(1,i);
    y_star = trajectory(2,i);
    
    dist = sqrt((x_star-x)^2+(y_star-y)^2);
    if dist < min_dist
        min_dist = dist;
        index = i;
    end
end
end