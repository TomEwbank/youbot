function trajectory = calc_traj(map, pose, dest, cell_size, d)

x_ref = pose(1);
y_ref = pose(2);

size_map = size(map);

x_robot = floor((x_ref+d)/cell_size)+1;
y_robot = floor((y_ref+d)/cell_size)+1;

occupancy_grid = map';

dx = DXform(occupancy_grid, 'inflate', 1);%floor(0.5/cell_size)); % create navigation object
if(dx.occgrid(y_robot, x_robot) == 1)
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








