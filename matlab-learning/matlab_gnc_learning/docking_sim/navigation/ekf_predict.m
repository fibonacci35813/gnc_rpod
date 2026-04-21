function ekf = ekf_predict(ekf, u, dt)
n=ekf.n; c=cos(n*dt); s=sin(n*dt);
Phi=[4-3*c,      0, 0, s/n,        2*(1-c)/n,    0;
     6*(s-n*dt), 1, 0, -2*(1-c)/n, (4*s-3*n*dt)/n,0;
     0,          0, c, 0,           0,             s/n;
     3*n*s,      0, 0, c,           2*s,           0;
     -6*n*(1-c), 0, 0, -2*s,        4*c-3,         0;
     0,          0,-n*s,0,           0,             c];
B=[dt^2/2*eye(3); dt*eye(3)];
ekf.x = Phi*ekf.x + B*u;
ekf.P = Phi*ekf.P*Phi' + ekf.Q;
end
