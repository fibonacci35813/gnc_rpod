function F = pd_controller(pos_err, vel_err, phase, params)
if phase==3; Kp=params.Kp_terminal; Kd=params.Kd_terminal; mib=params.mib_terminal;
else;        Kp=params.Kp_normal;   Kd=params.Kd_normal;   mib=params.mib_normal; end
F=Kp(:).*pos_err(:) + Kd(:).*vel_err(:);
F=sign(F).*min(abs(F),50);
F(abs(F)*1.0 < mib)=0;
end
