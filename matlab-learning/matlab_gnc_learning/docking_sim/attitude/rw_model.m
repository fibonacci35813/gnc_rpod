function [h_new,tau_act]=rw_model(h_rw,tau_cmd,params,dt)
sat=abs(h_rw/params.I_rw)>params.omega_max;
tau_act=tau_cmd; tau_act(sat)=0;
h_new=h_rw+tau_act*dt;
end
