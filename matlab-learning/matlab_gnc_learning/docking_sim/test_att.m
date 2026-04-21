addpath('attitude');
% Test attitude control with identity reference
params.Kp_att=0.05; params.Kd_att=1.0; params.I_rw=0.1; params.omega_max=500;
q=[1;0;0;0]; omega=zeros(3,1); hrw=zeros(3,1);
dt=1.0;
fprintf('Testing attitude with identity reference:\n');
for k=1:10
    R_des=eye(3);
    qr=rot2quat(R_des);
    [tc,ae]=attitude_control(q,qr,omega,params);
    [hrw,ta]=rw_model(hrw,tc,params,dt);
    omega=omega+ta/(3*params.I_rw)*dt;
    q=quat_kinematics(q,omega,dt);
    fprintf('k=%d q=[%.4f %.4f %.4f %.4f] att=%.4f omega=[%.4f %.4f %.4f]\n',...
        k,q(1),q(2),q(3),q(4),ae,omega(1),omega(2),omega(3));
end

% Now test with pos=[0,0.1,0]
fprintf('\nTesting attitude with pe=[0,0.1,0] yb=pe/|pe|:\n');
q=[1;0;0;0]; omega=zeros(3,1); hrw=zeros(3,1);
pe=[0;0.1;0];
for k=1:10
    rng_l=norm(pe);
    yb=pe/rng_l; xb=cross(yb,[0;0;1]);
    if norm(xb)<1e-6; xb=[1;0;0]; end
    xb=xb/norm(xb); R_des=[xb,yb,cross(xb,yb)];
    fprintf('R_des=\n'); disp(R_des);
    qr=rot2quat(R_des);
    fprintf('qr=[%.4f %.4f %.4f %.4f]\n',qr(1),qr(2),qr(3),qr(4));
    [tc,ae]=attitude_control(q,qr,omega,params);
    [hrw,ta]=rw_model(hrw,tc,params,dt);
    omega=omega+ta/(3*params.I_rw)*dt;
    q=quat_kinematics(q,omega,dt);
    fprintf('k=%d att=%.4f\n',k,ae);
    break; % just show first step
end
exit
