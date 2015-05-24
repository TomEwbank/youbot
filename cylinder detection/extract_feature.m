%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% [feature_id,feature_parameters]=extract_feature(cyl1,cyl2)
%
% Input: two cylinders cyl1 and cyl2. Standard 7x1 cylinder vectors, as
% detected by ransac_cylinder.m
%
%
% Output:
% Feature id: -1 incompatible cylinders
% 0 cylinders unrelated
% 1 convex step
% 2 concave step
% 3 gap
% 4 hole
% 5 merging
%
% Feature_parameters: a vector or matrix containing parameters of detected
% feature. See chapter 5.2 of the report for details.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [feature_id,feature_parameters]=extract_feature(cyl1,cyl2)
% the one with bigger radius should be cyl1
if cyl2(7)>cyl1(7)
 tmp = cyl1;
 cyl1 = cyl2;
 cyl2 = tmp;
end
%% parameters and declarations
% parameters
h = 0.13; %height of the robot (ground to sensor), in meter
min_gap_length = 0.02; %in meter
max_gap_length = 0.1; % in meter
dist_thr = 0.01; % in meter
merge_thr = 0.3; % in parts of 1 (i.e. merge_thr = 0.5 means the shorter
 % cylinder must be at least 50% inside the longer one to
provoke merging).
rad_thr = 0.01; % in meter, radii are considered equal, if their difference is
lower than rad_thr.
angle_thr = 5; % in degrees, used to decide if axes are parallel;
% declarations
feature_id = 0;
feature_parameters = 0;
angle_thr = abs(cos(angle_thr*pi/180));
axis1 = [cyl1(1:3) cyl1(4:6)];
axis2 = [cyl2(1:3) cyl2(4:6)];
axis1_vec = cyl1(4:6)-cyl1(1:3);
axis2_vec = cyl2(4:6)-cyl2(1:3);
axis1_dir = axis1_vec/norm(axis1_vec);
axis2_dir = axis2_vec/norm(axis2_vec);
cos_alpha = abs(axis1_dir'*axis2_dir);
r1 = cyl1(7);
r2 = cyl2(7);
[n1,d1]=point_to_line(cyl2(1:3),axis1);
[n2,d2]=point_to_line(cyl2(4:6),axis1);
%% Go
[p1,id1,a2r,c1r] = line_vs_cyl (axis2,cyl1);
if a2r(1,1)>a2r(1,2) , a2r(1,:) = [a2r(1,2) a2r(1,1)]; end %sorting so that a2r points in positive x-direction

if id1(1) == 1 % possible hole
 % making sure that the first point of cyl2 is the one closer to cyl1 axis
 % (axis2 is pointing away from cyl1)
 if d1>d2
 cyl2(1:6) = [cyl2(4:6); cyl2(1:3)];
 axis2_vec = - axis2_vec;
 axis2_dir = - axis2_dir;
 d_tmp = d1; d1 = d2; d2 = d_tmp;
 end

 hc = hole_condition(a2r,c1r,dist_thr);

 if hc ==1
 % disp('hole detected!');
 hole_to_axis = point_to_line(p1(:,1),axis1);
 hole = [[p1(:,1); axis2_dir; r2] [hole_to_axis; axis1_dir; r1] ];
 feature_parameters = hole;
 feature_id = 4;
 return;
 elseif hc == -1 % both ends of cyl2 inside cyl1
 % disp('incompatible cylinders');
 feature_parameters = 0;
 feature_id = -1;
 return;
 else
 % disp('too far for a hole');
 feature_parameters = 0;
 feature_id = 0;
 return;
 end
end
[XYpr,YZpr,overlap_part,overlap_length] = projection_analysis(c1r,a2r,r2,d1,d2,cos_alpha,dist_thr);
% gap and merging
if cos_alpha>angle_thr && (d1<dist_thr && d2<dist_thr )
 % cylinders are parallel and concentric
 if XYpr == 1 && abs(r1-r2)<rad_thr
 %cylinders have almost the same radius and overlap
 if overlap_part > merge_thr
 % disp('Merging suggested');
 merged_cyls = merge_cyls (cyl1,cyl2);
 feature_parameters = merged_cyls;
 feature_id = 5;
 return;
 end
 elseif XYpr == 0 && abs(r1-r2)<rad_thr
 % cylinders have almost the same radius and do not overlap
 if gap_condition (c1r,a2r,min_gap_length,max_gap_length) == 1
 % disp('Gap detected');
 feature_parameters = build_gap (cyl1,cyl2);
 feature_id = 3;
 return;
 end
 elseif XYpr == 1 && abs(r1-r2)>rad_thr && overlap_part > merge_thr
 % cylinders overlap, but have different radii
 % disp('Overlapping concentric cylinders: incompatible');
 feature_parameters = 0;
 feature_id = -1;
 return;
 end
end
if id1(1) == 2 && overlap_length<=dist_thr+r2*sin(acos(cos_alpha))%possible step


 % making sure that the first points of both cylinder vectors are those,
which are
 % closer to the step. (<-- --> axes are pointing away from each other)
 if norm(cyl1(1:3)-p1(:,1))>norm(cyl1(4:6)-p1(:,1))
 cyl1(1:6)= [cyl1(4:6);cyl1(1:3)];
 axis1=-axis1;
 axis1_dir = -axis1_dir;
 end
 if norm(cyl2(1:3)-p1(:,1))>norm(cyl2(4:6)-p1(:,1))
 cyl2(1:6)= [cyl2(4:6);cyl2(1:3)];
 axis2=-axis2;
 axis2_dir = -axis2_dir;
 end

 if (r1-norm(cyl1(1:3)-p1(:,1)))>abs(r2/cos_alpha)
 % disp('possible step detected');
 stp = find_step_type(cyl1,cyl2);
 if stp ==1 && norm(cyl2(1:3)-p1(:,1))<dist_thr+r2*tan(acos(cos_alpha))
%convex step
 % disp('convex step');
 step = [[cyl1(1:3);axis1_dir;r1],[p1(:,1);axis2_dir;r2]];
 feature_parameters = step;
 feature_id = 1;
 return;
 elseif stp ==2 %concave step
 c = 2*r1 - 2*r2/cos_alpha;
 b = norm(cyl2(1:3)'*axis2_dir);
 max_dist = c*tan(pi/2 - acos(cos_alpha) - atan(h/b));
 if max_dist<0, max_dist = 0; end
 if norm(axis1_dir'*(cyl2(1:3)-p1(:,1)))<max_dist+dist_thr
 % disp(['concave step. max_dist + tol = ',num2str(max_dist +dist_thr),', distance between cylinders = ',num2str(norm(axis1_dir'*(cyl2(1:3)-p1(:,1))))]);
 step_center = cyl2(1:3)+axis2_dir*r2*tan(acos(cos_alpha));
 step = [[cyl1(1:3)+axis1_dir*(((cyl1(1:3)-step_center)'*axis2_dir)/cos_alpha);axis1_dir;r1],[step_center;axis2_dir;r2]];
 feature_parameters = step;
 feature_id = 2;
 return;
 else
 % disp('concave step would be possible, but cylinders are too far');
 feature_parameters = 0;
 feature_id = 0;
 return;
 end
 end

 end

end
end

%% additional functions
function gap = build_gap (cyl1,cyl2)
% output: 7x1 vector containing those ends of cyl1 and cyl2, which are
% closer to each other and a radius which is 50mm larger than radius of input cylinders.
 gp = zeros(6,1);
if norm(cyl1(1:3)-((cyl2(1:3)+cyl2(4:6))/2))<norm(cyl1(4:6)-((cyl2(1:3)+cyl2(4:6))/2))
 gp(1:3) = cyl1(1:3);
else
 gp(1:3) = cyl1(4:6);
end
if norm(cyl2(1:3)-((cyl1(1:3)+cyl1(4:6))/2))<norm(cyl2(4:6)-((cyl1(1:3)+cyl1(4:6))/2))
 gp(4:6) = cyl2(1:3);
else
 gp(4:6) = cyl2(4:6);
end
gap_ax_dir = (cyl1(4:6)-cyl1(1:3))/norm(cyl1(4:6)-cyl1(1:3));
gp(4:6) = gp(1:3) + gap_ax_dir*((gp(4:6)-gp(1:3))'*gap_ax_dir);
r_new = 0.05 + (cyl1(7)+cyl2(7))/2;
gap = [gp(1:3);gp(4:6);r_new];
end
function satisfied = gap_condition (c1r,a2r,min_gap_length,max_gap_length)
if a2r(4)<-min_gap_length && a2r(4)>-max_gap_length ||...
 a2r(1)-c1r(4)>min_gap_length && a2r(1)-c1r(4)<max_gap_length
 satisfied =1;
else
 satisfied = 0;
end
end
function satisfied = hole_condition (axis2_rotated,cyl1_rotated,dist_tol)
% this function is called only if axis2 goes through lateral surface of cyl1 and
% this fact is used in following logical decisions.
% Output: 1 Hole possible
% 0 Cylinders too far
% -1 Cylinders are incompatible
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
 if abs(axis2_rotated(5))<abs(axis2_rotated(2))
 axis2_rotated = [axis2_rotated(:,2),axis2_rotated(:,1)];
 end

 if abs(axis2_rotated(5))<cyl1_rotated(7)
 satisfied = -1; %both ends of axis2 inside cyl1
 return;
 end

 if abs(axis2_rotated(5))-abs(axis2_rotated(2)) < abs(axis2_rotated(5)-axis2_rotated(2))
 satisfied = -1; %axis2 reaches too far into cyl1
 return;
 end


 if abs(axis2_rotated(5)-axis2_rotated(2))<abs(axis2_rotated(5)) && abs(axis2_rotated(5)-axis2_rotated(2))>abs(axis2_rotated(5))-cyl1_rotated(7)-dist_tol
 satisfied = 1; % one end of axis2 is inside of or close enough to cyl1,
the other one is outside.
 else
 satisfied = 0; % both ends too far away
 end
end
function [XYpr,YZpr,overlap_part,overlap_x_length] = projection_analysis(c1r,a2r,r2,d1,d2,cos_alpha,dist_thr)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Input variables have the same names as in the main function
% (extract_feature)
%
% Output values:
%
% XYpr = 0 - cylinder projections on X-Y plane do not overlap
% 1 - cylinder projections on X-Y plane overlap and output value
% overlap_part shows how much of the shorter cylinder lies
% inside the longer one.
%
% YZpr = 0 - cylinder projections on Y-Z plane do not overlap
% 1 - At least one end of cyl2 is inside cyl1 on Y-Z-projection
% -1 - cyl1 and cyl2 intersect on Y-Z-projection'
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
r1 = c1r(7);
overlap_part = 0;
overlap_x_length = 0;
% disp(' ');disp('% analysing Y-Z-projection'); disp(' ');
if d1>r1+r2*cos_alpha && d2>r1+r2*cos_alpha
 % disp('c1 is far from c2, overlapping absent or insignificant');
 YZpr = 0;
elseif (d1+(r2*cos_alpha)<r1+dist_thr) || (d2+(r2*cos_alpha)<r1+dist_thr)
 YZpr = 1;
 % if (d1+r2*cos_alpha<r1+dist_thr), disp('Left end of c2 is inside c1'),end;
 % if (d2+r2*cos_alpha<r1+dist_thr), disp('Right end of c2 is inside c1'),end;
else
 % disp('c1 and c2 intersect');
 YZpr = -1;
end
% disp(' ');disp('% analysing X-Y-projection');disp(' ');
if a2r(4)<0 || a2r(1)>c1r(4)
 % disp('No overlapping');
 XYpr = 0;
elseif (a2r(1)>0 && a2r(4)<c1r(4)) ||... % cyl2 inside cyl1 OR
 (a2r(1)<0 && a2r(4)>c1r(4)) % cyl1 inside cyl2
 % disp('full overlapping');
 overlap_part = 1;
 XYpr = 1;
else

 if (a2r(4)>0 && a2r(4)<=c1r(4))
 overlap_x_length = a2r(4);
 elseif (a2r(1)>0 && a2r(1)<=c1r(4))
 overlap_x_length = c1r(4)-a2r(1);
 else
 overlap_x_length = 0;
 end
 if (a2r(4)-a2r(1)) == 0
 overlap_part = 1;
 else
 overlap_part = max(overlap_x_length/c1r(4),overlap_x_length/(a2r(4)-a2r(1)));
 end
 XYpr = 1;
end
end
function stp = find_step_type(cyl1,cyl2)
%%%%%%%%%%%%%%%%%%%%%%%%%%
% This function calculates, which of the input cylinders is closer to the
% sensor (to the origin) and gives a type of possible step as output.
%
% Output:
% stp = 1 : Possible convex step
% stp = 2 : Possible concave step
% stp = -1 : No step expected
%%%%%%%%%%%%%%%%%%%%%%%%%%
axis1_vec = cyl1(4:6)-cyl1(1:3);
axis2_vec = cyl2(4:6)-cyl2(1:3);
r1 = cyl1(7);
r2 = cyl2(7);
%if (cyl1(1:3)'*axis1_vec)*(cyl1(4:6)'*axis1_vec)<0, disp('cyl1 goes through the origin'),end;
%if (cyl2(1:3)'*axis2_vec)*(cyl2(4:6)'*axis2_vec)<0, disp('cyl2 goes through the origin'),end;
if (cyl1(1:3)'*axis1_vec)*(cyl1(4:6)'*axis1_vec)<0 && (cyl2(1:3)'*axis2_vec)*(cyl2(4:6)'*axis2_vec)>=0
 %disp('cyl1 is closer to the origin');
 closer_cyl = 1;
elseif (cyl1(1:3)'*axis1_vec)*(cyl1(4:6)'*axis1_vec)>=0 && (cyl2(1:3)'*axis2_vec)*(cyl2(4:6)'*axis2_vec)<0
 %disp('cyl2 is closer to the origin');
 closer_cyl = 2;
elseif (cyl1(1:3)'*axis1_vec)*(cyl1(4:6)'*axis1_vec)<0 && (cyl2(1:3)'*axis2_vec)*(cyl2(4:6)'*axis2_vec)<0
 %disp('Strange stuff: both cylinders go through the origin');
 closer_cyl = -1;
else
 if(norm(cyl1(1:3)+cyl1(4:6))<=norm(cyl2(1:3)+cyl2(4:6)))
 %disp('cyl1 is closer to the origin');
 closer_cyl = 1;
 else
 %disp('cyl2 is closer to the origin');
 closer_cyl = 2;
 end
end
if (r1>r2 && closer_cyl ==1) || (r2>r1 && closer_cyl ==2)
 %disp('Potential convex step');

 stp = 1;
elseif (r1<r2 && closer_cyl ==1) || (r2<r1 && closer_cyl ==2)
 %disp('Potential concave step');
 stp = 2;
else
 %disp('No step expected');
 stp = -1;
end
end
