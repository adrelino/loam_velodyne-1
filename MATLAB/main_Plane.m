function r = main_Plane()

% generate some points
alpha_x = 0.3*pi;
alpha_y = 0.1*pi;
alpha_z = 0.4*pi;
R = getR(0,0,0)
t = rand(3,1)*10;

c_min = 0;
for i=1:1600
    m = rand(3,1);
    v = rand(3,1)*100;
    % 
    points{i} = [(3 * m + v)  (0 * m + v) (-3 * m + v)];
    v(1,1) = rand;
    points2{i} = rand(3,1);
    c{i} = mean([points{i}(:,1) points{i}(:,3) points2{i}(:,1)],2);
    
    c_min = c_min + getPlaneDistance(c{i},points{i}(:,1),points{i}(:,3),points2{i}(:,1));
    plane{i} = R * c{i} + t;
end
c_min

theta = [0;0;0;0;0;0];
theta(1:2) = -t(1:2);
lambda = 0.1;
plane_temp = plane;
for j = 1:1000
    R_temp = getR(theta(4),theta(5),theta(6));
t_temp = [theta(1);theta(2);theta(3)];
for i = 1:length(plane)
plane_temp{i} = R_temp*plane{i} +t_temp;
end
    
    
for i = 1:length(plane)
X = plane{i};
X_i = points{i}(:,1);
X_j = points{i}(:,3);
X_l = points2{i}(:,1);
J(i,:) = getJ(theta,X,X_i,X_j,X_l);
d(i,1) = getPlaneDistance(plane_temp{i},X_i,X_j,X_l);
end
sum(d)
up = pinv(J'*J+lambda*diag(diag(J'*J))) * J' * d.^2;
%up = pinv(J) * d.^2;
theta = theta - up;

end

theta;
R_temp = getR(theta(4),theta(5),theta(6));
t_temp = [theta(1);theta(2);theta(3)];
-pinv(R_temp)*t_temp -t

[t;alpha_x;alpha_y;alpha_z];


for i=1:length(plane)
plot3(c{i}(1,1),c{i}(2,1),c{i}(3,1),'rX');
hold on;
plot3(plane_temp{i}(1,1),plane_temp{i}(2,1),plane_temp{i}(3,1),'bO');
end
pause(100);

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

function J = getJ(theta,X,X_i,X_j,X_l)
d = cross( X_i - X_j , X_i - X_l );
d_2_x = d(1)^2;
d_2_y = d(2)^2;
d_2_z = d(3)^2;
n = d_2_x + d_2_y + d_2_z;
R = getR(theta(4),theta(5),theta(6));
t = [theta(1);theta(2);theta(3)];
a = (R*X+t-X_i);
a_x = a(1);
a_y = a(2);
a_z = a(3);




J = ( 2 * a_x * [1 0 0 a_x_theta(theta(4),theta(5),theta(6),X)'] * d_2_x ...
    + 2 * a_y * [0 1 0 a_y_theta(theta(4),theta(5),theta(6),X)'] * d_2_y ...
    + 2 * a_z * [0 0 1 a_z_theta(theta(4),theta(5),theta(6),X)'] * d_2_z) / n;
end

