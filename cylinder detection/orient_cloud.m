%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% oriented_cloud = orient_cloud (cloud)
%
%Input: - 3xn matrix cloud.
% Each column is a point in 3D space.
%Output: - 6xn matrix oriented_cloud.
% First three numbers in a column are coordinates of the point (same as input),
% the other three are the surface normal vector approximation.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function oriented_cloud = orient_cloud (cloud)
% parameters
k = 20; %Number of points taken to approximate the normal vector
% declarations
sampleSize = size(cloud, 2);
t1 = tic;
treeScan = kdtree_build(cloud');
t = toc(t1);
disp(['Time needed to build a kd-tree = ',num2str(t)]);
oriented_cloud = [cloud; zeros(3,sampleSize)];
t2 = tic;
for i = 1:sampleSize
 p = cloud(:,i); % pick a point in scanData
 idsScan = kdtree_k_nearest_neighbors(treeScan, p,k);
 nearestPts = cloud(:,idsScan);

 % Center data at origin

 p0 = [nearestPts(1,:) - p(1,:);
 nearestPts(2,:) - p(2,:);
 nearestPts(3,:) - p(3,:)];

 % Covariance of the points
 C = p0 * p0';

 % Eigenvector decomposition: compute Eigenvalues (D) and Eigenvectors (V)
 [V, D] = eig(C);

 % The normal to the plane will be the Eigenvector with the smallest
 % Eigenvalue
 if ((D(1, 1) < D(2, 2)) && (D(1, 1) < D(3, 3)))
 n = V(:, 1)';
 else
 if ((D(2, 2) < D(1, 1)) && (D(2, 2) < D(3, 3)))
 n = V(:, 2)';
 else
 n = V(:, 3)';
 end
 end
 oriented_cloud(4:6,i) = n;
end
t = toc(t2);
disp(['Time needed to calculate ',num2str(sampleSize),' normal vectors = ',num2str(t)]);
end
