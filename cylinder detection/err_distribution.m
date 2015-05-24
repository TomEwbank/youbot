function [mu,sigma,dist_array]=err_distribution(sys_noise_multiplier,random_noise_multiplier)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%[mu,sigma,dist_array]=err_distribution(sys_noise_multiplier,random_noise_multiplier)
%
% Calculates parameters of error normal distribution (mu,sigma) for each distance of dist_array.
%
% Multiplies values of mu with sys_noise_multiplier and values of sigma
% with random_noise_multiplier.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Ugly way to get error distribution functions
err = [91 11 2.5; 1500 14 3; 2992 -10 2; 4992 3 5]/1000;
err(:,2) = err(:,2)*sys_noise_multiplier;
err(:,3) = err(:,3)*random_noise_multiplier;
l = err(1,1):0.02:err(2,1);
m = (err(2,2)-err(1,2))/(err(2,1)-err(1,1));
b = (err(1,2)*err(2,1) - err(2,2)*err(1,1))/(err(2,1)-err(1,1));
m_s = (err(2,3)-err(1,3))/(err(2,1)-err(1,1));
b_s = (err(1,3)*err(2,1) - err(2,3)*err(1,1))/(err(2,1)-err(1,1));
dist_array = l;
mu = l*m + b;
sigma = l*m_s+b_s;
l = err(2,1):0.020:err(3,1);
m = (err(2,2)-err(3,2))/(err(2,1)-err(3,1));
b = (err(3,2)*err(2,1) - err(2,2)*err(3,1))/(err(2,1)-err(3,1));
m_s = (err(2,3)-err(3,3))/(err(2,1)-err(3,1));
b_s = (err(3,3)*err(2,1) - err(2,3)*err(3,1))/(err(2,1)-err(3,1));
dist_array = [dist_array l];
mu = [mu l*m + b];
sigma = [sigma l*m_s+b_s];
l = err(3,1):0.020:err(4,1);
m = (err(4,2)-err(3,2))/(err(4,1)-err(3,1));
b = (err(3,2)*err(4,1) - err(4,2)*err(3,1))/(err(4,1)-err(3,1));
m_s = (err(4,3)-err(3,3))/(err(4,1)-err(3,1));
b_s = (err(3,3)*err(4,1) - err(4,3)*err(3,1))/(err(4,1)-err(3,1));
dist_array = [dist_array l];
mu = [mu l*m + b];
sigma = [sigma l*m_s+b_s];
% figure(2);plot(dist_array,[mu;sigma]);
% xlabel('Distance to the target, [m]'); ylabel('Normal distribution parameter,[m]');
% legend('mu','sigma');
end