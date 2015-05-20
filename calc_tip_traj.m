function traj = calc_tip_traj(p,d_before,d_after,n)

pente = p(2)/p(1);
alpha = atan(pente);

p_start = [p(1)-d_before*cos(alpha); p(2)-d_before*sin(alpha); p(3)];
p_end = [p(1)+d_after*cos(alpha); p(2)-d_after*sin(alpha); p(3)];

x = p_start(1):abs(p_start(1)-p_end(1))/n:p_end(1);
y = pente*x;
z = zeros(1,length(x))+p(3);

traj = [x;y;z];

end