 function []=show_step(step)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% []=show_step(step)
%
% Displays a step feature as a blue disc in the current figure.
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
ax_dir = step(4:6,1);
rad = step(7,1);
[Z1,Y1,X1] = cylinder(rad);
[R,R_back] = rot_matrix(ax_dir);
A1 = [X1(1,:);Y1(1,:);Z1(1,:)];
A1 = R_back*A1;
A2 = [X1(2,:);Y1(2,:);Z1(2,:)];
A2 = R_back*A2;
X1 = [A1(1,:)+step(1,1);A2(1,:)+step(1,1)];
Y1 = [A1(2,:)+step(2,1);A2(2,:)+step(2,1)];
Z1 = [A1(3,:)+step(3,1);A2(3,:)+step(3,1)];
rad = step(7,2);
[Z2,Y2,X2] = cylinder(rad);
A1 = [X2(1,:);Y2(1,:);Z2(1,:)];
A1 = R_back*A1;
A2 = [X2(2,:);Y2(2,:);Z2(2,:)];
A2 = R_back*A2;
X2 = [A1(1,:)+step(1,2);A2(1,:)+step(1,2)];
Y2 = [A1(2,:)+step(2,2);A2(2,:)+step(2,2)];
Z2 = [A1(3,:)+step(3,2);A2(3,:)+step(3,2)];
surface([X1(1,:); X2(1,:)],[Y1(1,:); Y2(1,:)],[Z1(1,:); Z2(1,:)],'FaceColor',[0.3 0 1]);