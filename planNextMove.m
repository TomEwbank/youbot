function [trajectory_coord, map, trajectory] = planNextMove(map, pose, d, cell_size, fill_point)
% Plan the next move  of the robot in its exploration of the map
%
% ARGUMENTS
%   - map :         a NxN matrix representing the occupancy map in wich we
%                   want to calculate the trajectory. The reference frame
%                   is supposed to be at the center of the map. Unexplored
%                   points in the map are equal to -1, explored ones are
%                   equal to 0 and obstacles are equal to 1.
%   - pose :        the position [x y] of the robot, in meters, and in the
%                   reference frame of the map.
%   - d :           distance between the reference frame and a frame in the
%                   corner of the map.
%   - cell_size :   size of the side of a cell in the occupancy map
%   - fill_point :  indices of point in the occupancy from where the
%                   floodflill operations needed by this function will be
%                   performed. It needs to be a point that isn't too near
%                   an obstacle.
%
% OUTPUT
%   - trajectory_coord :  a Mx2 matrix containing the coordinates, in the
%                         reference frame of the map, of the points
%                         composing the trajectory.
%   - map :               the map that has eventually been updated
%   - trajectory :        a Mx2 matrix containing the indices of the map
%                         corresponding to the points composing the trajectory.

x_ref = pose(1);
y_ref = pose(2);

size_map = size(map);

x_robot = floor((x_ref+d)/cell_size)+1;
y_robot = floor((y_ref+d)/cell_size)+1;

occupancy_grid = map';
occupancy_grid(occupancy_grid == -1) = 0;

% Before using the "distancexform" function, we need to identify
% unaccessible zones and mark it in the grid as obstacles. Without that,
% "distancexform" would enter in an infinite loop. In order to achieve that
% we perform a floodfill starting at the location of the robot so that all
% unaccessible cells will remain unchanged, and thus identifiable.
filled_grid = imfill(logical(occupancy_grid),[double(fill_point(2)) double(fill_point(1))]);
occupancy_grid(filled_grid == 0) = 1;
map(filled_grid' == 0) = 1;

if(occupancy_grid(y_robot, x_robot) == 1)
    % if robot's position considered as an obstacle in the map,
    % need to find a close free point to replace it
    a = y_robot-1;
    offset_x = 2;
    offset_y = 2;
    while(a < 1)
        a = a+1;
        offset_y = offset_y-1;
    end
    b = y_robot+1;
    while(b > size_map(2))
        b = b-1;
    end
    e = x_robot-1;
    while(e < 1)
        e = e+1;
        offset_x = offset_x-1;
    end
    f = x_robot+1;
    while(f > size_map(1))
        f = f-1;
    end
    [r,c] = find(occupancy_grid(a:b, e:f) ~= 1);
    x_robot = x_robot+c(1)-offset_x;
    y_robot = y_robot+r(1)-offset_y;
end
% Now, it's ok to use the distance transform:
Dtrans = distancexform(occupancy_grid, [x_robot, y_robot]);

% create navigation object with a map where obstacles have been slightly
% inflated
dx = DXform(occupancy_grid, 'inflate', 1);
if(dx.occgrid(y_robot, x_robot) == 1)
    % if robot's position considered as an obstacle in the map,
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
    dx.occgrid(y_robot, x_robot)
end
filled_grid = imfill(logical(dx.occgrid),[double(fill_point(2)) double(fill_point(1))]);
dx.occgrid(filled_grid == 0) = 1;
map_temp = map';

map_temp(dx.occgrid == 1) = 1;
map_temp = map_temp';
[r,c] = find(map_temp == -1); % search for unexplored zone in the map

if(isempty(r))
    
    trajectory = [];
    trajectory_coord = [];
    display('There is no more zone to explore');
    colormap([0 104/255 139/255; 0 1 127/255; 205/255 38/255 38/255]);
    imagesc(map_temp);
else
    x = r(1);
    y = c(1);
    if(dx.occgrid(y,x) == 1)
        min_dist = NaN;
    else
        min_dist = Dtrans(y, x);
    end
    
    % Selects the closest unexplored point in the map
    for k=2:length(r)
        i = r(k);
        j =  c(k);
        dist = Dtrans(j, i);
        if((dist<min_dist || isnan(min_dist)) && dx.occgrid(j,i) ~= 1)
            min_dist = dist;
            x = i;
            y = j;
        end
    end
    
    dx.plan([x y]) % create plan for specified goal
    trajectory = dx.path([x_robot y_robot]);
    trajectory_coord = trajectory*cell_size-cell_size/2-d;
end

end






