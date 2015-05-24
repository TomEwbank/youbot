 function cyl_new = set_axis_length(A,cyl)
%A = noisy_cylinder([0 -7 0]',[0 -5 0]',1,1000,0.01);
%cyl = [0 0 0 0 1 0 1]';
% The following is a brute force method for finding the ends of a cylinder
% candidate. It requires rotating the whole point cloud.
% It will do until something faster is found. It is still fast
% enough, compared to the total execution time of cylinder detection.
p1 = cyl(1:3);
ax_dir = (cyl(4:6)-cyl(1:3))/norm(cyl(4:6)-cyl(1:3));
[R,R_back] = rot_matrix(ax_dir);
A_rot = R*A(1:3,:);
[mx_x,id_1] = max(A_rot(1,:));
[mn_x,id_2] = min(A_rot(1,:));
ax1 = p1 + ax_dir*(ax_dir'*(A(1:3,id_1)-p1));
ax2 = p1 + ax_dir*(ax_dir'*(A(1:3,id_2)-p1));
cyl_new = [ax1; ax2; cyl(7)];
end