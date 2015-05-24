function alpha = angle_3pts(vertex, p2, p3)
% Returns the angle formed by 3 points 
%
% ARGUMENTS 
%   - vertex :    coordinates [x y] of the vertex of the angle
%   - p2 and p3 : coordinates [x y] of the two point forming the angle with
%                 the vertex
%
% OUTPUT
%   - alpha :     the smallest angle formed by the vertex, p2 and p3


P12 = sqrt((vertex(1) - p2(1))^2 + (vertex(2) - p2(2))^2);
P13 = sqrt((vertex(1) - p3(1))^2 + (vertex(2) - p3(2))^2);
P23 = sqrt((p2(1) - p3(1))^2 + (p2(2) - p3(2))^2);

alpha = acos((P12^2 + P13^2 - P23^2) / (2 * P12 * P13));

end