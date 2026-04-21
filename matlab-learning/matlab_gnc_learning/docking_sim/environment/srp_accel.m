function a = srp_accel(r_vec, A, m, Cr)
P_srp = 4.56e-6;
r_sun = [1;0;0];
if dot(r_vec/norm(r_vec),r_sun)>0
    a = (Cr*P_srp*A/m)*r_sun;
else
    a = zeros(3,1);
end
end
