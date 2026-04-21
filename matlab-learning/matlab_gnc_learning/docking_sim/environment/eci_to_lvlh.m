function [pos_lvlh, vel_lvlh, R_eci2lvlh] = eci_to_lvlh(r_tgt,v_tgt,r_chs,v_chs)
r_hat = r_tgt/norm(r_tgt);
h_vec = cross(r_tgt,v_tgt);
h_hat = h_vec/norm(h_vec);
y_hat = cross(h_hat,r_hat);
R_eci2lvlh = [r_hat,y_hat,h_hat]';
dr = r_chs - r_tgt;
dv = v_chs - v_tgt;
omega_vec = h_hat*norm(h_vec)/norm(r_tgt)^2;
pos_lvlh = R_eci2lvlh*dr;
vel_lvlh = R_eci2lvlh*(dv - cross(omega_vec,dr));
end
