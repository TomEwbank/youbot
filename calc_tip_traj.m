function traj = calc_tip_traj(p,d_before,d_after,n)
% Returns a trajectory  
%    1) following the equation of the line going through the
%       points O = [0 0 p(3)] and p, 
%    2) starting between O and p, at a certain distance of p
%    3) ending at a certain distance after p
%
% ARGUMENTS 
%   - p :         the point by wich the trajectory must go through
%   - d_before :  the distance from p where the trajectory starts 
%   - d_after :   the distance from p after which the trajectory ends
%   - n :         the number of points composing the trajectory
%
% OUTPUT
%   - traj :      a 3xn matrix containing the n points composing the trajectory 

pente = p(2)/p(1);
alpha = atan(pente);

p_start = [p(1)-d_before*cos(alpha); p(2)-d_before*sin(alpha); p(3)];
p_end = [p(1)+d_after*cos(alpha); p(2)-d_after*sin(alpha); p(3)];

x = p_start(1):abs(p_start(1)-p_end(1))/n:p_end(1);
y = pente*x;
z = zeros(1,length(x))+p(3);

traj = [x;y;z];

end