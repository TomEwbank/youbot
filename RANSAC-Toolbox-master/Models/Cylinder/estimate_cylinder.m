function [Theta, k] = estimate_cylinder(X, s, parameters)

% [Theta k] = estimate_circle(X, s, parameters)
%
% DESC:
% Circle estimation give the radius. See:
% Gander, W., Golub, G. H., & Strebel, R. (1996). Least-squares fitting of 
% circles and ellipses. Bulletin of the Belgian Mathematical .
%
% INPUT:
% X                 = 2D point coordinates
% s                 = indices of the points used to estimate the 
%                     parameter vector. If empty all the points 
%                     are used
% parameters        = parameters.radius is the radius of the circle
%
% OUTPUT:
% Theta             = estimated parameter vector Theta = H(:). If  
%                     the estimation is not successful return an 
%                     empty vector. i.e. Theta = [];
% k                 = dimension of the minimal subset

% here we define the size of the MSS
k = 3;

% check if the input parameters are valid
if (nargin == 0) || isempty(X)
    Theta = [];
    return;
end;

% select the points to estimate the parameter vector
if (nargin >= 2) && ~isempty(s)
    X = X(:, s);
end;

if rank(X) == 1
    Theta = [];
    return;
end

% check if we have enough points
N = size(X, 2);
if (N < k)
    error('estimate_foo:inputError', ...
        'At least k point correspondences are required');
end;

[center, r, v1, v2] = circlefit3d(X(:,1)',X(:,2)',X(:,3)');
Theta(1) = center(1);
Theta(2) = center(2);
Theta(3) = center(3);
Theta(4) = r;
v_cyl_axe = cross(v1,v2);
Theta(5) = v_cyl_axe(1);
Theta(6) = v_cyl_axe(2);
Theta(7) = v_cyl_axe(3);

return;
