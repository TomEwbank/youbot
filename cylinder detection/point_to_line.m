 function [n,d] = point_to_line(p,l)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% [n,d] = point_to_line(p,l)
%
% Input:
% p is a 3x1 vector containing a point,
% l is a 3x2 matrix containing two points, which define a line.
%
% Output:
% d is the shortest distance from the point p to the line l
% n is the direction unit vector of this connection.
%
%
% This function is very often used by RANSAC and finding a faster
% alternative would significantly improve the total performance.
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
 x = l(:,1);
 y = l(:,2)-x;
 t = (y'*(p-x)/norm(y)^2);
 n = p - x - y*t;
% following works only for the distance, not the direction
% n = cross((p-l(:,1)),(p-l(:,2)))/norm(l(:,1)-l(:,2));
d = norm(n);
n = n/d;
end
