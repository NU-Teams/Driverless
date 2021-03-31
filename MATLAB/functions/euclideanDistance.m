function ed = euclideanDistance(s, mu, z, lm_num)


% make this a bit more ribust for different applications

% Convert measurement to x y coordinates
x_meas = mu(1) + z(1).*sin(mu(3)+z(2));
y_meas = mu(2) + z(1).*cos(mu(3)+z(2));


% Check Euclidean Distance, find dx dy
dx = x_meas - mu(s.ny*lm_num+s.nx-1);
dy = y_meas - mu(s.ny*lm_num+s.nx);
% distance
ed = sqrt(dx^2+dy^2);
end