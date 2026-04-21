function q=quat_kinematics(q,omega,dt)
q0=q(1);v=q(2:4);
Xi=[-v';q0*eye(3)+skew(v)];
q=q+0.5*dt*[dot(-v,omega);q0*omega+cross(v,omega)];
q=q/norm(q);
end
function S=skew(v)
S=[0,-v(3),v(2);v(3),0,-v(1);-v(2),v(1),0];
end
