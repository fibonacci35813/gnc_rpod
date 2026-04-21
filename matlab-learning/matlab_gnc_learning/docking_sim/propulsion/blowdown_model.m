function [F_act, tank] = blowdown_model(F_cmd, tank)
V_gas = tank.V_total - tank.prop_remain/tank.rho_prop;
P     = tank.P0*tank.V_gas0/max(V_gas,tank.V_gas0);
P     = max(P, tank.P_min);
F_act = F_cmd*(P/tank.P_nom);
g0    = 9.80665;
dv    = norm(F_act)/tank.mass;
dm    = tank.mass*(1-exp(-dv/(tank.Isp*g0)));
tank.prop_remain = max(0, tank.prop_remain-dm);
tank.mass        = tank.mass-dm;
tank.P_current   = P;
tank.dv_total    = tank.dv_total+dv;
end
