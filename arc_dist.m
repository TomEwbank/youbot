function dist = arc_dist(center, r, p2, p3)
% Returns the length of an arc of circle
%
% ARGUMENTS 
%   - center :    coordinates [x y] of the center of the circle
%   - r :         radius of the circle
%   - p2 and p3 : coordinates [x y] of the two point delimiting the arc
%
% OUTPUT
%   - dist :      the length of the shortest arc delimited by p2 and p3

alpha = angle_3pts(center, p2, p3);
dist = pi*r*alpha/pi;

end