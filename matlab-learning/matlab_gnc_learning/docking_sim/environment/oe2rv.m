function [r_eci, v_eci] = oe2rv(a, e, inc, RAAN, omega, nu, mu)
p     = a*(1-e^2);
r_mag = p/(1+e*cos(nu));
r_pqw = r_mag*[cos(nu); sin(nu); 0];
v_pqw = sqrt(mu/p)*[-sin(nu); e+cos(nu); 0];
R     = Rz(-RAAN)*Rx(-inc)*Rz(-omega);
r_eci = R*r_pqw;
v_eci = R*v_pqw;
end
