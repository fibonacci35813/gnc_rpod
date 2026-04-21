function a = j2_accel(r_vec, mu, Re, J2)
x=r_vec(1); y=r_vec(2); z=r_vec(3); r=norm(r_vec);
c = -(3/2)*J2*(mu/r^2)*(Re/r)^2;
fxy = 1-5*(z/r)^2; fz = 3-5*(z/r)^2;
a = c*[(x/r)*fxy; (y/r)*fxy; (z/r)*fz];
end
