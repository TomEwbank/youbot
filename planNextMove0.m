function [trajectory, map] = planNextMove(map, pose, d, cell_size)

x_ref = pose(1);
y_ref = pose(2);

size_map = size(map);

x_robot = floor((x_ref+d)/cell_size)+1
y_robot = floor((y_ref+d)/cell_size)+1

% x_frame = floor((x_ref+d-2.6)/cell_size)+1;
% if (x_frame < 1)
%     x_frame = 1;
% end
% y_frame = floor((y_ref+d-2.6)/cell_size)+1;
% if (y_frame < 1)
%     y_frame = 1;
% end
% d_frame = floor(5.2/cell_size)+1;

occupancy_grid = map';
occupancy_grid(occupancy_grid == -1) = 0;

% Before using the "distancexform" function, we need to identify
% unaccessible zones and mark it in the grid as obstacles. Without that,
% "distancexform" would enter in an infinite loop. In order to achieve that
% we perform a floodfill starting at the location of the robot so that all
% unaccessible cells will remain unchanged, and thus identifiable.
filled_grid = imfill(logical(occupancy_grid),[double(y_robot) double(x_robot)]);
occupancy_grid(filled_grid == 0) = 1;
%map(filled_grid == 0) = 1;

if(occupancy_grid(y_robot, x_robot) == 1)
        a = y_robot-1;
        offset_x = 2;
        offset_y = 2;
        while(a < 1)
            a = a+1
            offset_y = offset_y-1;
        end
        b = y_robot+1;
        while(b > size_map(2))
            b = b-1
        end
        e = x_robot-1;
        while(e < 1)
            e = e+1
            offset_x = offset_x-1;
        end
        f = x_robot+1;
        while(f > size_map(1))
            f = f-1
        end
        [r,c] = find(occupancy_grid(a:b, e:f) ~= 1);
        x_robot = x_robot+c(1)-offset_x
        y_robot = y_robot+r(1)-offset_y
end
% Now, it's ok to use the distance transform:
Dtrans = distancexform(occupancy_grid, [x_robot, y_robot]);

% Check for unexplored point in a limited frame for efficiency
%[r,c] = find(map(x_frame:x_frame+d_frame, y_frame:y_frame+d_frame) == -1);



%if(isempty(r))
    %no unexplored point found so check the full map
    %(method to review, could be better because of the previous frame explored twice)
    dx = DXform(occupancy_grid, 'inflate', 3); % create navigation object
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
        [r,c] = find(dx.occgrid(a:b, e:f) ~= 1)
        x_robot = x_robot+c(1)-offset_x
        y_robot = y_robot+r(1)-offset_y
        dx.occgrid(y_robot, x_robot)
    end
    filled_grid = imfill(logical(dx.occgrid),[double(y_robot) double(x_robot)]);
    dx.occgrid(filled_grid == 0) = 1;
    map_temp = map';
    
    map_temp(dx.occgrid == 1) = 1;
    map_temp = map_temp';
     %figure; imagesc(map_temp); drawnow;
%     figure; imagesc(map); drawnow;
%     figure; imagesc(Dtrans'); drawnow;
%     figure;
    [r,c] = find(map_temp == -1);
%end

if(isempty(r))
    
    trajectory = [];
    display('There is no more zone to explore');
    figure;
    imagesc(map_temp);
    drawnow;
    figure;
    imagesc(dx.occgrid);
    drawnow;
else
    
    
    
    x = r(1)%+x_frame-1;
    y = c(1)%y_frame-1;
    if(dx.occgrid(y,x) == 1)
        min_dist = NaN;
    else
        min_dist = Dtrans(y, x);
    end
        
    for k=2:length(r)
        i = r(k);%+x_frame-1;
        j =  c(k);%+y_frame-1;
        dist = Dtrans(j, i);
        if((dist<min_dist || isnan(min_dist)) && dx.occgrid(j,i) ~= 1)
            min_dist = dist;
            x = i;
            y = j;
        end
    end
    min_dist
    x
    y
    

    dx.plan([x y]) % create plan for specified goal
    t = dx.path([x_robot y_robot]);
    %dx.path([x_robot y_robot]);
    %t = [t(:,2) t(:,1)];
    
    st = size(t);
    trajectory = zeros(100,2);
    it = 1;
    directions = [1 2 3; 4 5 6; 7 8 9];
    
    if(st(1)>1)
    deltaY = t(2,2) - t(1,2);
    deltaX = t(2,1) - t(1,1);
    prev_dir = directions(2+deltaX, 2+deltaY);
    
    for k=2:st(1)-1
        deltaY = t(k+1,2) - t(k,2);
        deltaX = t(k+1,1) - t(k,1);
        dir = directions(2+deltaX, 2+deltaY);
        
        if(prev_dir ~= dir)
            trajectory(it,:) = t(k, :);
            it = it+1;
        end
        prev_dir = dir;
    end
    end
    
    trajectory(it,:) = t(st(1),:);
    
    % Keep only the part of the vector containing stored values, and 
    % transpose these values in the coordinate system of the map
    trajectory = trajectory(1:find(trajectory(:,1), 1, 'last'),:)
    trajectory = trajectory*cell_size-cell_size/2-d;
end

end






