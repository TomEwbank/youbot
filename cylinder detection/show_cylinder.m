 function []=show_cylinder(cyl)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% []=show_cylinder(cyl)
%
% Plot the input cylinder cyl in the current figure.
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
ax = cyl(4:6) - cyl(1:3);
ax_length = norm(ax);
ax_dir = ax/ax_length;
rad = cyl(7);
[Z,Y,X] = cylinder(rad);
X = X*ax_length;
[R,R_back] = rot_matrix(ax_dir);
A1 = [X(1,:);Y(1,:);Z(1,:)];
A1 = R_back*A1;
A2 = [X(2,:);Y(2,:);Z(2,:)];
A2 = R_back*A2;
X = [A1(1,:)+cyl(1);A2(1,:)+cyl(1)];
Y = [A1(2,:)+cyl(2);A2(2,:)+cyl(2)];
Z = [A1(3,:)+cyl(3);A2(3,:)+cyl(3)];
surface(X,Y,Z);
end