 function A = noisy_cylinder(p1,p2,rad,number_points,noise_span)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% A = noisy_cylinder(p1,p2,rad,number_points,noise_span)
%
% Creates a homogeneous randomly sampled point cloud of a cylinders, with
% axis defined by two points p1 and p2 and radius rad. number_points
% defines the size of the point cloud.
%
% Uniformly distributed noise of +- noise span is applied to each point.
%
% Output is a 3xnumber_points matrix A, containing a point cloud of a
% cylinder.
%
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
A = zeros(3,number_points);
ax = p2-p1;
ax_length = norm(ax);
ax_dir = ax/ax_length;
for i=1:size(A,2)
 rand_a = rand(1)*2*pi;
 rand_x = rand(1)*ax_length;
 A(:,i)= [rand_x;
 rad*cos(rand_a);
 rad*sin(rand_a)]+(noise_span)*(rand([3 1])-[0.5;0.5;0.5]);
end
[R,R_back]=rot_matrix(ax_dir);
A = R_back*A;
A = [A(1,:)+p1(1);A(2,:)+p1(2);A(3,:)+p1(3)];
end