function dX = eom_perturbed(~,X,mu,Re,J2,Cd,A,m,Cr)
r=X(1:3); v=X(4:6); rm=norm(r);
ag = -(mu/rm^3)*r;
aj = j2_accel(r,mu,Re,J2);
ad = drag_accel(r,v,Cd,A,m,Re);
as = srp_accel(r,A,m,Cr);
dX = [v; ag+aj+ad+as];
end
