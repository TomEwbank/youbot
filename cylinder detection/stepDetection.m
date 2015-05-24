%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% Main script file for obstacle detection in 3D point clouds of tube-like
% environments. The first part of the script contains multiple options for
% loading or generating input point clouds. The second part is the execution.
%
% Alex Trofimov, ETH Zurich, June 2010
%
%
% There are multiple options for input. Uncomment the needed one, leave the
% others commented.
%
%
% After the execution of the script, following data can be considered
% output:
% - 7xn matrix cyl_det: contains parameters of n detected cylinder.
% - nxN matrix ap_glob, containing n vectors of length N, where N is the
% size of the initial point cloud. A value ap_glob(i,j) is 1, if j-th
% point of the point cloud belongs to i-th cylinder. Zero otherwise.
% - 1xk vector f_id, where k=n*(n-1)/2 possible cylinder pairs. Stores feature_id output of extract_feature.m for every cylinder combination.
% - 1xk cell par. Stores feature_parameters output of extract_feature.m for every
% cylinder combination.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% clc
% %close all
% figure(1);
% figure(2);
% close(1);
% close(2);
% % % Add folders to search path
% addpath('pipes');
% addpath('kdtree1.2/kdtree');
% %profile on;
t_total = tic;
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% %% Load/generate 3D scan data (file format: three columns with x, y, z)
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% % 1) Real scans of steam chest environment (robot position: (0, 0, 0))
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% %data = load('3Dtest_129_xyzf11.txt');
% %data = load('3Dtest_150_xyzf11.txt');
% data = load('3Dtest_1201_xyzf11.txt');
% %Systematic subsampling (for example take every 10th point)
% nTotSamples = size(data, 1);
% filter = 1:10:nTotSamples;
% A = data(filter, :)';
% % Random subsampling (as alternative)
% % filter = rand(size(data, 1), 1); % random vector of same data size
% % A = data(filter < 0.1, :); % keep 10% of the points
% % 2) Computer-generated uniformly sampled point sets
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% % % Concave Step
% % axis1 = [-0.5 0 0; 0.5 0 0]';
% % axis2 = [0.6 0 0; 1.2 0 0]';
% % set1 = noisy_cylinder(axis1(:,1),axis1(:,2),0.25,5000,0);
% % set2 = noisy_cylinder(axis2(:,1),axis2(:,2),0.32,4000,0);
% % A = [set1 set2];
% % % Convex Step
% % axis1 = [-0.5 0 0; 0.5 0 0]';
% % axis2 = [0.505 0 0; 1.2 0 0]';
% % set1 = noisy_cylinder(axis1(:,1),axis1(:,2),0.32,5000,0);
% % set2 = noisy_cylinder(axis2(:,1),axis2(:,2),0.25,4000,0);
% % A = [set1 set2];
% % % Hole
% % axis1 = [-0.5 0 0; 0.5 0 0]';
% % axis2 = [0 0.25 0; 0 1 0]';
% % set1 = noisy_cylinder(axis1(:,1),axis1(:,2),0.32,5000,0);
% % set2 = noisy_cylinder(axis2(:,1),axis2(:,2),0.25,4000,0);
% % A = [set1 set2];
% % % Gap
% % axis1 = [-0.5 0 0; 0.5 0 0]';
% % axis2 = [0.6 0 0; 1.2 0 0]';
% % set1 = noisy_cylinder(axis1(:,1),axis1(:,2),0.25,5000,0);
% 
% % set2 = noisy_cylinder(axis2(:,1),axis2(:,2),0.25,4000,0);
% % A = [set1 set2];
% % 3) Simulated scans of ideal environment
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% %build_scan_los; %Change the parameters in build_scan_los file to generate other
% scans.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Cylinder detection
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Parameters and declarations
% At this point, a 3xn matrix A must have been loaded to
% workspace.
% Find surface orientation for every point

load('cylcloud.mat')
pts = [pts2(3,:);pts2(1,:);pts2(2,:)];
pts = pts(:,1:84);

figure
plot3(pts(1,:),pts(2,:),pts(3,:),'*')


A = orient_cloud(double(pts));
% parameters
max_number_of_objects = 10;
trash_percentage_threshold = 0.01; % in parts of 1. Default value 0.01, i.e. 1%.
% The search stops as soon as there are less
% unassigned point left, than
% trash_percentage_threshold part of original size.
% declarations
time_ransac = zeros(1,max_number_of_objects);
n = 0;
size_A = size(A,2);
A_unassigned = A;
size_A_unassigned = size_A;
id_unassigned = 1:size_A;
cyl_det = zeros(7,0);
ap_glob = zeros(0,size_A);
it = zeros(1,max_number_of_objects);
%% ransac
% While-loop keeps searching for objects and plotting until either it
% reaches the desired number of objects
% or
% the remaining point cloud is too small to continue
% or
% RANSAC fails to find another cylinder.
while (n<max_number_of_objects && (size_A_unassigned/size_A)>trash_percentage_threshold)
    n = n+1;
    t_ran = tic;
    [cyl_det(:,n),ap,it(n)] = ransac_cylinder(A_unassigned);
    time_ransac(n) = toc(t_ran);
    
    
    % breaking the while loop if no object was found
    if sum(ap)==0
        cyl_det = cyl_det(:,1:n-1);
        n = n-1; %in the end, n is the number of detected objects.
        break
    end
    
    
    % saving the global assignment vector for cylinder n
    ap_glob(n,id_unassigned(:,ap==1)) = 1;
    % assigning the values for the next iteration
    id_unassigned = id_unassigned(:,ap==0);
    A_unassigned = A_unassigned(:,ap==0);
    size_A_unassigned = size(A_unassigned,2);
    
    
end
%% plotting
figure(1);
axis equal;
hold on;
for i = 1:n
    show_cylinder(cyl_det(:,i));
end
figure(2);
axis equal;
hold on;
% plotting the unassigned points in red
X = A_unassigned(1,:);
Y = A_unassigned(2,:);
Z = A_unassigned(3,:);
plot3(X,Y,Z,'o','MarkerEdgeColor','k','MarkerFaceColor','r','MarkerSize',3)
for i = 1:n
    % plotting points of the cylinder in random color
    A_cyl = A(:,ap_glob(i,:)==1);
    X = A_cyl(1,:);
    Y = A_cyl(2,:);
    Z = A_cyl(3,:);
    
    cyl_color = [0.8 0.9 0.9] - rand([1 3]).*[0.8 0.9 0.9];
    
    plot3(X,Y,Z,'o','MarkerEdgeColor','k','MarkerFaceColor',cyl_color,'MarkerSize',3);
    line(cyl_det([1 4],i),cyl_det([2 5],i),cyl_det([3
        6],i),'color',cyl_color,'LineWidth',5);
    %show_cylinder_col(cyl_det(:,i),cyl_color);
end
%% feature extraction
% The results are plotted in figure 2
t_feat=tic;
k = 0;
par = cell(1);
for i = 1:n-1
    for j = i+1:n
        k = k+1;
        [f_id(k),par_out] = extract_feature(cyl_det(:,i),cyl_det(:,j));
        par(k) = {par_out};
        if f_id(k)~=0 % display the message only for related cylinder pairs
            disp(['Cylinders ',num2str(i),' and ',num2str(j),': feature id = ',num2str(f_id(k))]);
                end
                if f_id(k) == 1 || f_id(k) == 2 %step
                disp('Step detected');
                show_step(cell2mat(par(k)));
                elseif f_id(k) == 4 %hole
                    disp('Hole detected');
                    show_hole(cell2mat(par(k)));
                elseif f_id(k) == 5 %merge
                    disp('Merge suggested');
                elseif f_id(k) == 3 %gap
                    gap_cyl = cell2mat(par(k));
                    for g = 1:n
                        gap_check_id=extract_feature(gap_cyl,cyl_det(:,g));
                        if gap_check_id ==-1
                            disp('Gap found, but will not be displayed, since it is incompatible with other cylinders');
                            break;
                        end
                    end
                    if gap_check_id ~= -1
                        show_cylinder_col(gap_cyl,[0 1 0]);
                        disp('Gap detected');
                    end
                end
        end
    end
    disp(['Time for feature detection on ',num2str(n),' cylinders = ',num2str(toc(t_feat))]);
    disp(['Finished after ', num2str(toc(t_total))]);
    %profile viewer
    % Remove folder from search path
    rmpath('pipes');
    rmpath('kdtree1.2/kdtree');
    
