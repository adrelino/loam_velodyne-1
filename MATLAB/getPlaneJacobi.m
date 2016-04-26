function J = getPlaneJacobi(theta, normal_k, point_k, point_k1 )
%GETPLANEJACOBI Summary of this function goes here
%   Detailed explanation goes here

d = normal_k;
d_2_x = d(1)^2;
d_2_y = d(2)^2;
d_2_z = d(3)^2;
n = d_2_x + d_2_y + d_2_z;
R = getR(theta(4),theta(5),theta(6));
t = [theta(1);theta(2);theta(3)];
m = point_k;
a = (R*point_k1+t-m);
a_x = a(1);
a_y = a(2);
a_z = a(3);




J = ( 2 * a_x * [1 0 0 a_x_theta(theta(4),theta(5),theta(6),point_k1)'] * d_2_x ...
    + 2 * a_y * [0 1 0 a_y_theta(theta(4),theta(5),theta(6),point_k1)'] * d_2_y ...
    + 2 * a_z * [0 0 1 a_z_theta(theta(4),theta(5),theta(6),point_k1)'] * d_2_z) / sqrt(n);

J = (1 / (2 * norm(a' * normal_k'))) * J;

end

function r = w_11(phi,theta,psi)
r = cos(phi) * cos(theta);
end

function r = w_12(phi,theta,psi)
r = cos(phi) * sin(theta) * sin(psi) - sin(phi) * cos(psi);
end

function r = w_13(phi,theta,psi)
r = cos(phi) * sin(theta) * cos(psi) + sin(phi) * sin(psi);
end

function r = w_21(phi,theta,psi)
r = sin(phi) * cos(theta);
end

function r = w_22(phi,theta,psi)
r = sin(phi) * sin(theta) * sin(psi) + cos(phi) * cos(psi);
end

function r = w_23(phi,theta,psi)
r = sin(phi) * sin(theta) * cos(psi) - cos(phi) * sin(psi);
end

function r = w_31(phi,theta,psi)
r = - sin(theta);
end

function r = w_32(phi,theta,psi)
r = cos(theta) * sin(psi);
end

function r = w_33(phi,theta,psi)
r = cos(theta) * cos(psi);
end
%%
function r = w_11_phi(phi,theta,psi)
r = -w_21(phi,theta,psi);
end
function r = w_11_theta(phi,theta,psi)
r = w_31(phi,theta,psi) * cos(phi);
end
function r = w_11_psi(phi,theta,psi)
r = 0;
end

function r = w_12_phi(phi,theta,psi)
r = -w_22(phi,theta,psi);
end
function r = w_12_theta(phi,theta,psi)
r = w_33(phi,theta,psi) * cos(phi);
end
function r = w_12_psi(phi,theta,psi)
r = w_13(phi,theta,psi);
end

function r = w_13_phi(phi,theta,psi)
r = -w_23(phi,theta,psi);
end
function r = w_13_theta(phi,theta,psi)
r = w_33(phi,theta,psi) * cos(phi);
end
function r = w_13_psi(phi,theta,psi)
r = -w_12(phi,theta,psi);
end

function r = w_21_phi(phi,theta,psi)
r = w_11(phi,theta,psi);
end
function r = w_21_theta(phi,theta,psi)
r = w_31(phi,theta,psi) * sin(phi);
end
function r = w_21_psi(phi,theta,psi)
r = 0;
end

function r = w_22_phi(phi,theta,psi)
r = w_12(phi,theta,psi);
end
function r = w_22_theta(phi,theta,psi)
r = w_31(phi,theta,psi) * sin(phi);
end
function r = w_22_psi(phi,theta,psi)
r = w_23(phi,theta,psi);
end

function r = w_23_phi(phi,theta,psi)
r = w_13(phi,theta,psi);
end
function r = w_23_theta(phi,theta,psi)
r = w_33(phi,theta,psi) * sin(phi);
end
function r = w_23_psi(phi,theta,psi)
r = -w_22(phi,theta,psi);
end

function r = w_31_phi(phi,theta,psi)
r = 0;
end
function r = w_31_theta(phi,theta,psi)
r = - w_33(phi,theta,psi) * cos(psi);
end
function r = w_31_psi(phi,theta,psi)
r = 0;
end

function r = w_32_phi(phi,theta,psi)
r = 0;
end
function r = w_32_theta(phi,theta,psi)
r = - cos(theta);
end
function r = w_32_psi(phi,theta,psi)
r = w_33(phi,theta,psi);
end

function r = w_33_phi(phi,theta,psi)
r = 0;
end
function r = w_33_theta(phi,theta,psi)
r = w_31(phi,theta,psi) * cos(psi);
end
function r = w_33_psi(phi,theta,psi)
r = -w_32(phi,theta,psi);
end
%%
function r = a_x_theta(phi,theta,psi,X)
r = [ w_11_phi(phi,theta,psi) w_12_phi(phi,theta,psi) w_13_phi(phi,theta,psi);
    w_11_theta(phi,theta,psi) w_12_theta(phi,theta,psi) w_13_theta(phi,theta,psi);
    w_11_psi(phi,theta,psi) w_12_psi(phi,theta,psi) w_13_psi(phi,theta,psi)]*X;
end
function r = a_y_theta(phi,theta,psi,X)
r = [ w_21_phi(phi,theta,psi) w_22_phi(phi,theta,psi) w_23_phi(phi,theta,psi);
    w_21_theta(phi,theta,psi) w_22_theta(phi,theta,psi) w_23_theta(phi,theta,psi);
    w_21_psi(phi,theta,psi) w_22_psi(phi,theta,psi) w_23_psi(phi,theta,psi)]*X;
end
function r = a_z_theta(phi,theta,psi,X)
r = [ w_31_phi(phi,theta,psi) w_32_phi(phi,theta,psi) w_33_phi(phi,theta,psi);
    w_31_theta(phi,theta,psi) w_32_theta(phi,theta,psi) w_33_theta(phi,theta,psi);
    w_31_psi(phi,theta,psi) w_32_psi(phi,theta,psi) w_33_psi(phi,theta,psi)]*X;
end

