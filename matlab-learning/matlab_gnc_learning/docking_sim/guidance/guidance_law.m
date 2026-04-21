function [pos_ref,vel_ref,phase,v_cmd] = guidance_law(pos_est,vel_est,params)
r=norm(pos_est);
if r>50;     phase=0; vmax=params.v_phase_max(1);
elseif r>10; phase=1; vmax=params.v_phase_max(2);
elseif r>1;  phase=2; vmax=params.v_phase_max(3);
else;        phase=3; vmax=params.v_phase_max(4);
end
v_cmd=min(vmax, params.K_V*r);
if r>0.001; vel_ref=-v_cmd*pos_est/r; else; vel_ref=zeros(3,1); end
pos_ref=pos_est;
end
