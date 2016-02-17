function r = main()

% generate some points
alpha_x = 0.3*pi;
alpha_y = 0.1*pi;
alpha_z = 0.4*pi;
R = getR(alpha_x,alpha_y,alpha_z)
t = rand(3,1)*10;



for i=1:1600
    m = rand(3,1);
    v = rand(3,1)*100;
    points{i} = [(2 * m + v)  (0 * m + v) (-2 * m + v)];
    %points2{i} = [R*(2 * m + v)+ t  R*(0 * m + v)+t R*(-2 * m + v)+ t];
    edge{i} = R * points{i}(:,2) + t;
    %edge{i} =  pinv(R) * (points{i}(:,2) - t);
end

% for i=1:length(edge)
% plot3(points{i}(1,2),points{i}(2,2),points{i}(3,2),'rX');
% hold on;
% plot3(edge{i}(1,1),edge{i}(2,1),edge{i}(3,1),'bO');
% end
% pause(100);
%getEdgeDistance(R * points{i}(:,2) ,points{i}(:,1),points{i}(:,3))
%plot(points{1}(1,1),points{1}(3,1),'X');hold on; plot(points{1}(1,2),points{1}(3,2),'rO');hold on; plot(points{1}(1,3),points{1}(3,3),'X');hold on;plot(edge{1}(1,1),edge{1}(3,1),'O')
% for i=1:10
%     d_E(i) = getEdgeDistance(edge{i},points{i}(:,1),points{i}(:,3));
%     d_E2(i) = getEdgeDistance(points{i}(:,2),points2{i}(:,1),points2{i}(:,3));
%     %plot3(points{i}(1,1),points{i}(2,1),points{i}(3,1),'bX');hold on;
%     %plot3(points{i}(1,2),points{i}(2,2),points{i}(3,2),'rX');hold on;
%     %plot3(points{i}(1,3),points{i}(2,3),points{i}(3,3),'bX');
% end
theta = [0;0;0;0;0;0];
lambda = 0.0001;
edge_temp = edge;
for j = 1:100
for i = 1:length(edge)
X = edge{i};
X_i = points{i}(:,1);
X_j = points{i}(:,3);
J(i,:) = getJ(theta,X,X_i,X_j);
d(i,1) = getEdgeDistance(edge_temp{i},X_i,X_j);
end
sum(d)
up = pinv(J'*J+lambda*diag(diag(J'*J))) * J' * d.^2;
%up = pinv(J) * d.^2;
theta = theta - up;
R_temp = getR(theta(4),theta(5),theta(6));
t_temp = [theta(1);theta(2);theta(3)];
for i = 1:length(edge)
%edge_temp{i} = inv(R)*(edge{i}-t_temp);
edge_temp{i} = R_temp*edge{i} +t_temp;
end
end

theta;
R_temp = getR(theta(4),theta(5),theta(6));
t_temp = [theta(1);theta(2);theta(3)];
-pinv(R_temp)*t_temp -t

[t;alpha_x;alpha_y;alpha_z];

for i=1:length(edge)
plot3(points{i}(1,2),points{i}(2,2),points{i}(3,2),'rX');
hold on;
plot3(edge_temp{i}(1,1),edge_temp{i}(2,1),edge_temp{i}(3,1),'bO');
end
pause(100);

%edgenew = inv(R)*(edge{1}-t);
%plot(points{1}(1,1),points{1}(3,1),'X');hold on; plot(points{1}(1,2),points{1}(3,2),'rO');hold on; plot(points{1}(1,3),points{1}(3,3),'X');hold on;plot(edgenew(1,1),edgenew(3,1),'O')

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
% function r = a_x(t_x,t_y,t_z,phi,theta,psi,X,X_i)
% r = [w_11(phi,theta,psi) w_12(phi,theta,psi) w_13(phi,theta,psi)]*X + t_x - X_i(1);
% end
% function r = a_y(t_x,t_y,t_z,phi,theta,psi,X,X_i)
% r = [w_21(phi,theta,psi) w_22(phi,theta,psi) w_23(phi,theta,psi)]*X + t_y - X_i(2);
% end
% function r = a_z(t_x,t_y,t_z,phi,theta,psi,X,X_i)
% r = [w_31(phi,theta,psi) w_32(phi,theta,psi) w_33(phi,theta,psi)]*X + t_z - X_i(3);
% end
% 
% function r = b_x(t_x,t_y,t_z,phi,theta,psi,X,X_j)
% r = [w_11(phi,theta,psi) w_12(phi,theta,psi) w_13(phi,theta,psi)]*X + t_x - X_j(1);
% end
% function r = b_y(t_x,t_y,t_z,phi,theta,psi,X,X_j)
% r = [w_21(phi,theta,psi) w_22(phi,theta,psi) w_23(phi,theta,psi)]*X + t_y - X_j(2);
% end
% function r = b_z(t_x,t_y,t_z,phi,theta,psi,X,X_j)
% r = [w_31(phi,theta,psi) w_32(phi,theta,psi) w_33(phi,theta,psi)]*X + t_z - X_j(3);
% end

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


function r = c_x_theta(t_x,t_y,t_z,phi,theta,psi,X,X_i,X_j)
r1 = 0;
r2 = X_i(3) - X_j(3);
r3 = X_j(2) - X_i(2);
d_a_y = a_y_theta(phi,theta,psi,X);
d_a_z = a_z_theta(phi,theta,psi,X);
r4 = d_a_y(1) * (X_i(3) - X_j(3)) + d_a_z(1) * (X_j(2) - X_i(2));
r5 = d_a_y(2) * (X_i(3) - X_j(3)) + d_a_z(2) * (X_j(2) - X_i(2));
r6 = d_a_y(3) * (X_i(3) - X_j(3)) + d_a_z(3) * (X_j(2) - X_i(2));
r = [r1 r2 r3 r4 r5 r6];
end
function r = c_y_theta(t_x,t_y,t_z,phi,theta,psi,X,X_i,X_j)
r1 = X_j(3) - X_i(3);
r2 = 0;
r3 = X_i(1) - X_j(1);
d_a_z = a_z_theta(phi,theta,psi,X);
d_a_x = a_x_theta(phi,theta,psi,X);
r4 = d_a_z(1) * (X_i(1) - X_j(1)) + d_a_x(1) * (X_j(3) - X_i(3));
r5 = d_a_z(2) * (X_i(1) - X_j(1)) + d_a_x(2) * (X_j(3) - X_i(3));
r6 = d_a_z(3) * (X_i(1) - X_j(1)) + d_a_x(3) * (X_j(3) - X_i(3));
r = [r1 r2 r3 r4 r5 r6];
end
function r = c_z_theta(t_x,t_y,t_z,phi,theta,psi,X,X_i,X_j)
r1 = X_i(2) - X_j(2);
r2 = X_j(1) - X_i(1);
r3 = 0;
d_a_x = a_x_theta(phi,theta,psi,X);
d_a_y = a_y_theta(phi,theta,psi,X);
r4 = d_a_x(1) * (X_i(2) - X_j(2)) + d_a_y(1) * (X_j(1) - X_i(1));
r5 = d_a_x(2) * (X_i(2) - X_j(2)) + d_a_y(2) * (X_j(1) - X_i(1));
r6 = d_a_x(3) * (X_i(2) - X_j(2)) + d_a_y(3) * (X_j(1) - X_i(1));
r = [r1 r2 r3 r4 r5 r6];
end
%%
function J = getJ(theta,X,X_i,X_j)
n = (X_i - X_j)' * (X_i - X_j);
R = getR(theta(4),theta(5),theta(6));
t = [theta(1);theta(2);theta(3)];
a = (R*X+t-X_i);
b = (R*X+t-X_j);
c = cross(a,b);
c_theta = [c_x_theta(theta(1),theta(2),theta(3),theta(4),theta(5),theta(6),X,X_i,X_j);
    c_y_theta(theta(1),theta(2),theta(3),theta(4),theta(5),theta(6),X,X_i,X_j);
    c_z_theta(theta(1),theta(2),theta(3),theta(4),theta(5),theta(6),X,X_i,X_j)];
J = (2 * c' * c_theta ) / n;
end