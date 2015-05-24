 function [R,R_back] = rot_matrix (dir_vector)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% [R,R_back] = rot_matrix (dir_vector)
%
% Matrix R rotates coordinate system, so that the x-axis points in
% dir_vector direction, while y-axis and z-axis span a plane with
% dir_vector as the normal.
% Matrix R_back is used for backwards transformation.
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
if size(dir_vector) ~= 3, error('Wrong input!'); end
dir_vector = dir_vector/norm(dir_vector);
a = dir_vector(1);
b = dir_vector(2);
c = dir_vector(3);
x = sqrt(1-c^2);
if x==0
 %Checkpoint for the rare case c= +-1 <=> x=0
 R = [ 0 0 c;
 0 1 0;
 -c 0 0];

 R_back = [0 0 -c;
 0 1 0;
 c 0 0];
else
 R = [ a b c;
 -b/x a/x 0;
 -a*c/x -b*c/x x];

 R_back = R';
end
end