 function F=cyl_obj_fun (x_in,A_found)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% F=cyl_obj_fun (x_in,A_found)
%
% Objective function for non-linear least squares refitting.
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
x = x_in(1:3);
a = x_in(4:6);
r = x_in(7);
a = a/norm(a);
number_of_points = size(A_found,2);
X = A_found(1,:);
Y = A_found(2,:);
Z = A_found(3,:);
F = zeros(1,number_of_points);
for i=1:number_of_points
 u = a(3)*(Y(i)-x(2))-a(2)*(Z(i)-x(3));
 v = a(1)*(Z(i)-x(3))-a(3)*(X(i)-x(1));
 w = a(2)*(X(i)-x(1))-a(1)*(Y(i)-x(2));
 F(i) = sqrt(u^2+v^2+w^2) - r;
end
end