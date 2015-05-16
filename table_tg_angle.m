function theta = table_tg_angle(table, pose)

alpha = angle_3pts(pose, table, [pose(1) pose(2)+1]);

if pose(1) > table(1) && pose(2) >= table(2)
    theta = -(2*pi - (alpha + pi/2));
elseif pose(1) >= table(1) && pose(2) < table(2)
    theta = alpha + pi/2;
elseif pose(1) < table(1) && pose(2) <= table(2)
    theta = pi/2 - alpha;
else
    theta = -(alpha - pi/2);
end

end