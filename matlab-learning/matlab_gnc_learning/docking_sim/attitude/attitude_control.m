function [tau,err_deg]=attitude_control(q,q_ref,omega,params)
q_ref_inv=[q_ref(1);-q_ref(2:4)];
q_err=quat_multiply(q_ref_inv,q);
err_deg=2*acosd(min(1,abs(q_err(1))));
% Ensure shortest path: if q_err(1) == 0, use small positive to avoid sign(0)=0 singularity
s = sign(q_err(1)); if s==0; s=1; end
ev=q_err(2:4)*s;
tau=-params.Kp_att*ev - params.Kd_att*omega;
tau=sign(tau).*min(abs(tau),2.0);
end
