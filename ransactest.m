%cyl_options.epsilon = 1e-6;
cyl_options.P_inlier = 0.5;
%cyl_options.sigma = 0.1;
cyl_options.est_fun = @estimate_cylinder;
cyl_options.man_fun = @error_cylinder;
cyl_options.mode = 'RANSAC';
%cyl_options.Ps = [];
%cyl_options.notify_iters = [];
cyl_options.min_iters = 1000;
%cyl_options.fix_seed = false;
%cyl_options.reestimate = true;
%cyl_options.stabilize = false;
cyl_options.T_noise_squared = 0.009;

load('cylcloud.mat')
pts = [pts2(3,:);pts2(1,:);pts2(2,:)];
pts = pts(:,1:84);

figure
plot3(pts(1,:),pts(2,:),pts(3,:),'*')

[results, opt] = RANSAC(pts, cyl_options);

v = [results.Theta(5) results.Theta(5) results.Theta(5)];
r = results.Theta(4)
c = [results.Theta(1) results.Theta(1) results.Theta(3)];
p = c+v;
cyl = [results.Theta(1) results.Theta(1) results.Theta(3) p(1) p(2) p(3) r];
figure
drawCylinder(cyl)
