%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% [lcc_points_global] = extract_lcc(A_acceptable,point_global_id,cyl,cloud_size,beta)
%
% This function makes a projection of 3D points on cylinder surface and
% extracts largest connected component from the resulting bitmap.
% Resolution beta defines the size of a pixel.
%
% Input: - 6xn matrix A, contains a cloud of oriented points.
% First three numbers in a column are coordinates of the point,
% the other three are its direction unit vector. This is only the
% part of a point cloud, which fits to the current cylinder
% candidate cyl.
% - (not used in this version) norm_vec and dist values are taken from the main funtion to
% avoid computing them again. dist(k) is length and norm_vec(k) is
% the direction of the shortest connection from the k-th point of
% A to the axis of cylinder candidate cyl.
% - 1xn vector point_global_id.
% - 7x1 vector cyl,
% the first three numbers are coordinates of one point,
% the next three numbers are coordinates of the second point,
% which together define an axis of the cylinder.
% The seventh number is a radius of the cylinder.
% - beta is a resolution parameter. It must be carefully chosen to
% match the sampling resolution of the point cloud.
% Output:
% - 1xn vector lcc_points. If k-th point of A belongs to
% largest connected component on cylinder candidate, then k-th
% element of points is 1. Otherwise it is 0.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [lcc_points_global] = extract_lcc(A_acceptable,point_global_id,cyl,cloud_size,beta)
size_A = size(A_acceptable,2);
cyl = set_axis_length(A_acceptable(1:3,:),cyl);
l = norm(cyl(1:3)-cyl(4:6));
r = cyl(7);
% making the axis 0.1% longer on each side to avoid if-operators
cyl(1:3) = cyl(4:6)+1.001*(cyl(1:3)-cyl(4:6));
cyl(4:6) = cyl(1:3)+1.001*(cyl(4:6)-cyl(1:3));
l = 1.002*l;
norm_vec = zeros(3,size_A);
dist = zeros(1,size_A);
for i=1:size_A
 [norm_vec(:,i),dist(i)] = point_to_line(A_acceptable(1:3,i),[cyl(1:3)
cyl(4:6)]);
end
n_rows = round(2*pi*r/beta);
n_columns = round(l/beta);

disp(['extract_lcc: Bitmap size ',num2str(n_rows),'x',num2str(n_columns)]);
if n_rows < 6
 disp ('extract_lcc: Warning! Resolution too low! Setting n_rows=6');
 n_rows = 6;
end
if n_columns < 4
 disp ('extract_lcc: Warning! Resolution too low! Setting n_columns = 4');
 n_columns = 4;
end

BW = zeros(n_rows,n_columns);
[R,R_back] = rot_matrix((cyl(1:3)-cyl(4:6))/l);
dir = R*norm_vec;
pix_adress = zeros(2,size_A);
for i = 1:size_A
 angle = atan2(dir(3,i),dir(2,i)) +pi;
 %if angle ==0 % if unlucky, angle = 0 -> crash at line 82: if BW(p_x...
 % p_x = 1; % On real and/or noisy data it is extremely unprobable.
 %else
 p_x = ceil(angle*n_rows/(2*pi));
 %end
 ax_pos = norm(A_acceptable(1:3,i)-norm_vec(1:3,i)*dist(i)-cyl(1:3))/l;
 %if ax_pos ==0 % if unlucky, ax_pos = 0 -> crash at line 82: if BW(p_x...
 % p_y = 1; % this happens at least with one point (as a result of set_axis_length.m)
 %else % Alternative solution used: make the axis 0.1% longer on
each side
 p_y = ceil(ax_pos*n_columns);
 %end
 if BW(p_x,p_y)~=1
 BW(p_x,p_y) = 1;
 end
 pix_adress(:,i) = [p_x;p_y];
end
[L,m]=bwlabeln(BW);
conn_comp_size = zeros(1,m);
for i = 1:n_rows
 for j = 1:n_columns
 if L(i,j) ~= 0
 conn_comp_size(L(i,j)) = conn_comp_size(L(i,j)) +1;
 end
 end
end
[C,largest_comp_id] = max(conn_comp_size);
lcc_points = zeros(1,size_A);
for i=1:size_A
 if L(pix_adress(1,i),pix_adress(2,i)) == largest_comp_id
 lcc_points(i) = 1;
 end
end
lcc_points_global = zeros(1,cloud_size);
lcc_points_global(point_global_id(lcc_points ==1))=1;
end