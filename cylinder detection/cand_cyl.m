 function [cyl_axis,cyl_radius]= cand_cyl (sample1,sample2)
% Calculates parameters of a cylinder based on two oriented points.
% Contains some redundant steps, can be simplified.
cand_dir = cross(sample1(4:6,1),sample2(4:6,1));
[R,R_back] = rot_matrix(cand_dir);
line1 = [ sample1(1:3) sample1(1:3)+sample1(4:6)];
line2 = [ sample2(1:3) sample2(1:3)+sample2(4:6)];
line1_r = R*line1;
line2_r = R*line2;
A = [ line1_r(2,1)-line1_r(2,2) line2_r(2,1)-line2_r(2,2); line1_r(3,1)-line1_r(3,2) line2_r(3,1)-line2_r(3,2)];
B = [ line2_r(2,1)-line1_r(2,1) line2_r(3,1)-line1_r(3,1)]';
t = linsolve(A,B); 

axis_point1_r = [line1_r(1,1)+t(1)*(line1_r(1,1)-line1_r(1,2));...
    line1_r(2,1)+t(1)*(line1_r(2,1)-line1_r(2,2));...
 line1_r(3,1)+t(1)*(line1_r(3,1)-line1_r(3,2))];
axis_point2_r = [line2_r(1,1)+t(2)*(line2_r(1,2)-line2_r(1,1));...
 line2_r(2,1)+t(2)*(line2_r(2,2)-line2_r(2,1));...
 line2_r(3,1)+t(2)*(line2_r(3,2)-line2_r(3,1))];
cyl_radius = norm(axis_point1_r - line1_r(:,1));
axis_point1 = R_back*axis_point1_r;
axis_point2 = R_back*axis_point2_r;
% output
cyl_axis = [axis_point1 axis_point2];
end
