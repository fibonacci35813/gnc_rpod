function ekf = ekf_init(pos0, vel0, n)
ekf.x = [pos0;vel0];
ekf.P = diag([9,9,9,0.01,0.01,0.01]);
ekf.Q = diag([1e-4,1e-4,1e-4,1e-6,1e-6,1e-6]);
ekf.n = n;
end
