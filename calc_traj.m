function [trajectory, traj_ind] = calc_traj(map, pose, dest, cell_size, d, inflate, meters, fill_point)
% Returns a trajectory from a starting position to destination in the given map
%
% ARGUMENTS 
%   - map :         a NxN matrix representing the occupancy map in wich we  
%                   want to calculate the trajectory. The reference frame 
%                   is supposed to be at the center of the map.
%   - pose :        the starting position [x y], in meters, in the 
%                   reference frame of the map.
%   - dest :        the destination [x y], in meters, in the reference 
%                   frame of the map, OR indices of the cell in the map.
%   - cell_size :   size of the side of a cell in the occupancy map
%   - d :           distance between the reference frame and a frame in the 
%                   corner of the map.
%   - inflate :     Boolean value, if true, the obstacles on the map are 
%                   slightly inflated before calcultaing the trajectory
%   - meters :      Boolean value, need to be true if the destination is 
%                   is provided in meters, and false if it is map indices.
%
% OUTPUT
%   - trajectory :  a Mx2 matrix containing the coordinates, in the 
%                   reference frame of the map, of the points composing the 
%                   trajectory.
'enter calc_traj'
x_ref = pose(1);
y_ref = pose(2);

size_map = size(map);

x_robot = floor((x_ref+d)/cell_size)+1
y_robot = floor((y_ref+d)/cell_size)+1

occupancy_grid = map';
% Before using the "distancexform" function, we need to identify
% unaccessible zones and mark it in the grid as obstacles. Without that,
% "distancexform" would enter in an infinite loop. In order to achieve that
% we perform a floodfill starting at the location of the robot so that all
% unaccessible cells will remain unchanged, and thus identifiable.
filled_grid = imfill(logical(occupancy_grid),[double(fill_point(2)) double(fill_point(1))]);
occupancy_grid(filled_grid == 0) = 1;

% create navigation object with a map where obstacles have been slightly
% inflated
if inflate
    dx = DXform(occupancy_grid, 'inflate', 1);
else
    dx = DXform(occupancy_grid);
end
% figure
% imagesc(dx.occgrid)
% drawnow
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
    b = y_robot+3;
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
end

if meters
    display('explo mode')
    dest = floor((dest+d)/cell_size)+1;
end
desti = dest
if(dx.occgrid(dest(2), dest(1)) == 1)
    % if destination considered as an obstacle in the inflated map,
    % need to find a close free point to replace it
    a = dest(2)-2;
    offset_x = 3;
    offset_y = 3;
    while(a < 1)
        a = a+1;
        offset_y = offset_y-1;
    end
    b = dest(2)+2;
    while(b > size_map(2))
        b = b-1;
    end
    e = dest(1)-2;
    while(e < 1)
        e = e+1;
        offset_x = offset_x-1;
    end
    f = dest(1)+2;
    while(f > size_map(1))
        f = f-1;
    end
    [r,c] = find(dx.occgrid(a:b, e:f) ~= 1);
    dest(1) = dest(1)+c(1)-offset_x;
    dest(2) = dest(2)+r(1)-offset_y;
end

% create plan for specified goal
dx.plan(dest);

trajectory = dx.path([x_robot y_robot]);
traj_ind = trajectory;
trajectory = trajectory*cell_size-cell_size/2-d;
'leave calc_traj'
end








