function theta = table_tg_angle(table, pose)
% Returns the angle that orients a robot to be tangent to a circular table
%
% ARGUMENTS
%   - table :  the coordinates [x y] of the table
%   - pose :   the position [x y] of the robot
%
% OUTPUT
%   - theta :  the angle that orients a robot to be tangent to the table

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