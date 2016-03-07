syms tx real 
syms ty real
syms tz real
syms phi real
syms theta real
syms psi real
R = [cos(phi)*cos(theta) cos(phi)*sin(theta)*sin(psi)-sin(phi)*cos(psi) cos(phi)*sin(theta)*cos(psi)+sin(phi)*sin(psi);
    sin(phi)*cos(theta) sin(phi)*sin(theta)*sin(psi)+cos(phi)*cos(psi) sin(phi)*sin(theta)*cos(psi)-cos(phi)*sin(psi);
    -sin(theta) cos(theta)*sin(psi) cos(theta)*cos(psi)];
syms x real
syms y real
syms z real 
X = R*[x;y;z]-[tx;ty;tz]
syms x_i real
syms y_i real
syms z_i real
X_i = [x_i;y_i;z_i]
syms x_j real 
syms y_j real
syms z_j real
X_j = [x_j;y_j;z_j]
syms x_l real
syms y_l real
syms z_l real
X_l = [x_l;y_l;z_l]


n = cross(X_i - X_j, X_i - X_l);
r = norm((X-X_i)'*n)/norm(n);
%F=sqrt(3).*(2.*(X.^2+Y.^2)-1);
diff(r,tx)
diff(r,ty)
diff(r,tz)
diff(r,phi)
diff(r,theta)
diff(r,psi)
jacobian(r*r,[tx,ty,tz,phi,theta,psi]')