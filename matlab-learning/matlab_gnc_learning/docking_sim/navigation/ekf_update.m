function ekf = ekf_update(ekf, meas)
if ~meas.valid; return; end
p=ekf.x(1:3); r=norm(p);
if r<0.01; return; end
rxy=max(sqrt(p(1)^2+p(2)^2),1e-6);
h=[r; atan2(p(2),p(1)); asin(p(3)/r)];
H=zeros(3,6);
H(1,1:3)=p'/r;
H(2,1)=-p(2)/rxy^2; H(2,2)=p(1)/rxy^2;
H(3,1)=-p(1)*p(3)/(r^2*rxy);
H(3,2)=-p(2)*p(3)/(r^2*rxy);
H(3,3)=rxy/r^2;
z=[meas.range;meas.az;meas.el];
inn=z-h; inn(2)=mod(inn(2)+pi,2*pi)-pi;
R=diag(meas.sigma.^2);
S=H*ekf.P*H'+R; K=ekf.P*H'/S;
ekf.x=ekf.x+K*inn;
ekf.P=(eye(6)-K*H)*ekf.P;
end
