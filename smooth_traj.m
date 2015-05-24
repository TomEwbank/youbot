function new_traj = smooth_traj(traj)
% This function smooth the trajectory, by adding more intermediate points
% to it.
%
% ARGUMENTS
%   - traj :      a Nx2 matrix representing the trajectory
%
% OUTPUT
%   - new_traj :  a Mx2 matrix representing the smoothed trajectory

l = length(traj(:,1));
x_smooth_traj = interp1(1:l, traj(:,1)', 1:0.2:l);
y_smooth_traj = interp1(1:l, traj(:,2)', 1:0.2:l);
new_traj = [x_smooth_traj', y_smooth_traj'];

end