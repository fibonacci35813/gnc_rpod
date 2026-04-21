function meas = lidar_rae(pos_true, rng_obj)
r   = norm(pos_true);
az  = atan2(pos_true(2),pos_true(1));
el  = asin(pos_true(3)/max(r,0.001));
sr  = max(0.002, 0.003*r);
sa  = 0.001; se = 0.001;
n   = rng_obj.randn(3,1);
meas.range = r  + sr*n(1);
meas.az    = az + sa*n(2);
meas.el    = el + se*n(3);
meas.valid = r > 0.005;
meas.sigma = [sr;sa;se];
end
