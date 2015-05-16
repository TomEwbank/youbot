function new_traj = smooth_traj(traj)
% This function smooth the trajectory, by first interpolating the x
% coordinates of the trajectory, and then interpolate the y coordinates as
% a function of the interpolated x coordinates

l = length(traj(:,1));
%
% % The interpolation function can't be called with x values that are not
% % stricly monotonic increasing, so the following loop apply the
% % interpolation with each monotonic part of the x vector, separately.
% prev_x = traj(1,1);
% state = 'increasing';
% i = 2;
% start_i = 1;
% y_smooth_traj = [];
% x_smooth_traj = [];
% while i <= l
%
%     if prev_x > traj(i,1)
%         new_state = 'decreasing';
%     elseif prev_x == traj(i,1);
%         new_state = state;
%     else
%         new_state = 'increasing';
%     end
%
%     % if there is a change of state, this is the point where the values
%     % stop increasing stricly monotonic
%     if not(strcmp(new_state, state))
%
%         xq = interp1(start_i:i-1, traj(start_i:i-1,1)', start_i:0.2:i-1,  'spline');
%         figure
%         plot(start_i:0.2:i-1, xq)
%         yq = interp1(traj(start_i:i-1,1)', traj(start_i:i-1,2)', xq, 'spline');
%
%         x_smooth_traj = [x_smooth_traj yq];
%         y_smooth_traj = [y_smooth_traj yq];
%
%         start_i = i;
%     end
%
%     state = new_state;
%     prev_x = traj(i,1);
%     i = i + 1;
% end
%
% if start_i == l
%     y_smooth_traj = [y_smooth_traj traj(l,2)];
% else
%     xq = interp1(start_i:i-1, traj(start_i:i-1,1)', start_i:0.2:i-1, 'spline');
%     yq = interp1(traj(start_i:i-1,1)', traj(start_i:i-1,2)', xq, 'spline');
%
%     x_smooth_traj = [x_smooth_traj yq];
%     y_smooth_traj = [y_smooth_traj yq];
% end

x_smooth_traj = interp1(1:l, traj(:,1)', 1:0.2:l);
y_smooth_traj = interp1(1:l, traj(:,2)', 1:0.2:l);

% figure
% plot(x_smooth_traj,y_smooth_traj)

new_traj = [x_smooth_traj', y_smooth_traj'];

end