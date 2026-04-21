function dX = two_body(~,X,mu)
r=X(1:3); v=X(4:6); rm=norm(r);
dX=[v; -(mu/rm^3)*r];
end
