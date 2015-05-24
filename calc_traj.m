function trajectory = calc_traj(map, pose, dest, cell_size, d)
% Returns a trajectory from a starting position to destination in the given map
%
% ARGUMENTS 
%   - map :         a NxN matrix representing the occupancy map in wich we  
%                   want to calculate the trajectory. The reference frame 
%                   is supposed to be at the center of the map.
%   - pose :        the starting position [x y], in meters, in the 
%                   reference frame of the map.
%   - dest :        the destination [x y], in meters, in the reference 
%                   frame of the map.
%   - cell_size :   size of the side of a cell in the occupancy map
%   - d :           distance between the reference frame and a frame in the 
%                   corner of the map.
%
% OUTPUT
%   - trajectory :  a Mx2 matrix containing the coordinates, in the 
%                   reference frame of the map, of the points composing the 
%                   trajectory.

x_ref = pose(1);
y_ref = pose(2);

size_map = size(map);

x_robot = floor((x_ref+d)/cell_size)+1;
y_robot = floor((y_ref+d)/cell_size)+1;

occupancy_grid = map';

% create navigation object with a map where obstacles have been slightly
% inflated
dx = DXform(occupancy_grid, 'inflate', 1); 
if(dx.occgrid(y_robot, x_robot) == 1)
    % if starting position considered as an obstacle in the inflated map,
    % need to find a close free point to replace it
    a = y_robot-2;
    offset_x = 3;
    offset_y = 3;
    while(a < 1)
        a = a+1;
        offset_y = offset_y-1;
    end
    b = y_robot+2;
    while(b > size_map(2))
        b = b-1;
    end
    e = x_robot-2;
    while(e < 1)
        e = e+1;
        offset_x = offset_x-1;
    end
    f = x_robot+2;
    while(f > size_map(1))
        f = f-1;
    end
    [r,c] = find(dx.occgrid(a:b, e:f) ~= 1);
    x_robot = x_robot+c(1)-offset_x;
    y_robot = y_robot+r(1)-offset_y;
    dx.occgrid(y_robot, x_robot);
end

dx.plan(floor((dest+d)/cell_size)+1); % create plan for specified goal
trajectory = dx.path([x_robot y_robot]);

trajectory = trajectory*cell_size-cell_size/2-d;
end








