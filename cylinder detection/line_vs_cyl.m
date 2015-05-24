%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% [points,id,L_r,cyl]=line_vs_cyl(l,cyl)
%
% This function searches for intersection points between (endless) line,
% defined by two points l = [p1,p2] (3x2), and a cylinder cyl (7x1).
%
% Input:
% line l (3x2)
% cylinder cyl (7x1)
%
% Output
% points is a 3x2 matrix, containing intersection points, if the function
% was successful.
% id is a 1x2 vector, containing following values for the first and the second intersection point respectively:
% 0 : line too far from cylinder, no intersection
% 1 : intersection with lateral surface
% 2 : intersection with base surface
% L_r and cyl are the same as input line and cylinder, but in other
% coordinates. This is a byproduct of this function and can be
% used outside, to avoid the coordinate transformation to be
% calculated twice.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [points,id,L_r,cyl]=line_vs_cyl(l,cyl)
% parameters
angle_cos_tol = 0.985; % cos(pi/180) ~ 0.985
% declarations
points = ones(3,2)*inf;
id = [0 0];
r = cyl(7);
%repositioning cylinders, so that cyl starts at the origin.
p_0 = cyl(1:3);
cyl(1:6) = cyl(1:6) - [p_0;p_0];
l = l - [p_0 p_0];
% First rotation: x-axis = cylinder axis
[R,R_back] = rot_matrix(cyl(4:6) - cyl(1:3));
C_r = R*[cyl(1:3) cyl(4:6)];
L_r = R*l;
cyl = [C_r(:,1);C_r(:,2);cyl(7)];
% Second rotation: line parallel to X-Y plane
alpha = atan2(-(L_r(3,2)-L_r(3,1)),L_r(2,2)-L_r(2,1));
R2 = [ 1 0 0;
 0 cos(alpha) -sin(alpha);
 0 sin(alpha) cos(alpha)];
L_r = R2*L_r;
z = L_r(3,1); % L(3,1) == L(3,2) at this point
if abs(z)>=r
 % no intersection between the line and the cylinder possible.
 return;
end
% intersections with lateral surface
if (L_r(1,2)-L_r(1,1))/norm(L_r(:,2)-L_r(:,1))<angle_cos_tol
 % angle not too small
 y1 = sqrt(r^2 - z^2);
 y2 = -y1;

 lamb1 = (y1 - L_r(2,1))/(L_r(2,2)-L_r(2,1));
 lamb2 = (y2 - L_r(2,1))/(L_r(2,2)-L_r(2,1));

 x1 = L_r(1,1) + lamb1*(L_r(1,2)-L_r(1,1));
 x2 = L_r(1,1) + lamb2*(L_r(1,2)-L_r(1,1));


 if x1>0 && x1<cyl(4)
 id(1) = 1;
 points(:,1) = [x1 y1 z]';
 end
 if x2>0 && x2<cyl(4)
 id(2) = 1;
 points(:,2) = [x2 y2 z]';
 end

end
% intersections with base surfaces
if abs((L_r(1,2)-L_r(1,1))/norm(L_r(:,2)-L_r(:,1)))>(1-angle_cos_tol)
 %angle not too big
 x1 = 0;
 x2 = cyl(4);

 lamb1 = (x1 - L_r(1,1))/(L_r(1,2)-L_r(1,1));
 lamb2 = (x2 - L_r(1,1))/(L_r(1,2)-L_r(1,1));

 y1 = L_r(2,1) + lamb1*(L_r(2,2)-L_r(2,1));
 y2 = L_r(2,1) + lamb2*(L_r(2,2)-L_r(2,1));

 if sqrt(y1^2+z^2)<r
 if id(1) == 0
 id(1) = 2;
 points(:,1) = [x1 y1 z]';
 else
 id(2) = 2;
 points(:,2) = [x1 y1 z]';
 end
 end
 if sqrt(y2^2+z^2)<r
 if id(1) == 0
 id(1) = 2;
 points(:,1) = [x2 y2 z]';
 else
 id(2) = 2;
 points(:,2) = [x2 y2 z]';
 end

 end
end
% sorting the points, so that the first point is the one closer to the
% middle of l.
mid = (L_r(:,1)+L_r(:,2))/2;
if norm(points(:,1)-mid) > norm(points(:,2)-mid)
 points = [points(:,2) points(:,1)];
 id = [id(2) id(1)];
end
% transforming the points back into original coordinates
points = R_back*R2'*points + [p_0 p_0];
end
