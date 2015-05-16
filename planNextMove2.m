function trajectory = planNextMove2(map, pose, d, cell_size)

x_ref = pose(1);
y_ref = pose(2);

m_robot = floor((x_ref+d)/cell_size)+1;
n_robot = floor((y_ref+d)/cell_size)+1;

x_frame = floor((x_ref+d-2.6)/cell_size)+1;
if (x_frame < 1)
    x_frame = 1;
end
y_frame = floor((y_ref+d-2.6)/cell_size)+1;
if (y_frame < 1)
    y_frame = 1;
end
d_frame = floor(5.2/cell_size)+1;

occupancy_grid = map;
occupancy_grid(occupancy_grid == -1) = 0;
Dtrans = distancexform(occupancy_grid, [n_robot, m_robot]);

% Check for unexplored point in a limited frame for efficiency
[r,c] = find(map(x_frame:x_frame+d_frame, y_frame:y_frame+d_frame) == -1);



if(isempty(r))
    %no unexplored point found so check the full map
    %(method to review, could be better because of the previous frame explored twice)
    [r,c] = find(map == -1);
end

if(isempty(r))
    trajectory = [];
    display('There is no more zone to explore');
else
    
    dx = DXform(occupancy_grid, 'inflate', 1); % create navigation object
    
    m = r(1)+x_frame-1;
    n = c(1)+y_frame-1;
    if(dx.occgrid(m,n) == 1)
        min_dist = NaN;
    else
        min_dist = Dtrans(m, n);
    end
        
    for k=2:length(r)
        i = r(k)+x_frame-1;
        j =  c(k)+y_frame-1;
        dist = Dtrans(i, j);
        if((dist<min_dist || isnan(min_dist)) && dx.occgrid(i,j) ~= 1)
            min_dist = dist;
            m = i;
            n = j;
        end
    end  

    dx.plan([n m]) % create plan for specified goal
    trajectory = dx.path([n_robot, m_robot]);
    trajectory = [trajectory(:,2)*cell_size-cell_size/2-d trajectory(:,1)*cell_size-cell_size/2-d];

end

end






