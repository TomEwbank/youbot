function index = find_closest_point(trajectory, x, y)

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