 function []=show_hole(hole)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% []=show_hole(hole)
%
% Displays a hole feature as a red circle in the current figure.
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
hole_dir = hole(1:3,2);
rad = hole(7,1);
[Z,Y,X] = cylinder(rad);
[R,R_back] = rot_matrix(hole_dir);
A1 = [X(1,:);Y(1,:);Z(1,:)];
A1 = R_back*A1;
plot3(A1(1,:)+hole(1,1),A1(2,:)+hole(2,1),A1(3,:)+hole(3,1),'color','r','LineWidth',5);
end