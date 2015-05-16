function alpha = angle_3pts(center, p2, p3)

P12 = sqrt((center(1) - p2(1))^2 + (center(2) - p2(2))^2);
P13 = sqrt((center(1) - p3(1))^2 + (center(2) - p3(2))^2);
P23 = sqrt((p2(1) - p3(1))^2 + (p2(2) - p3(2))^2);

alpha = acos((P12^2 + P13^2 - P23^2) / (2 * P12 * P13));

end