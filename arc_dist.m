function dist = arc_dist(center, r, p2, p3)

alpha = angle_3pts(center, p2, p3);
dist = pi*r*alpha/pi;

end