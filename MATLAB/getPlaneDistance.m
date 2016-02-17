function r = getPlaneDistance(X, X_i, X_j, X_l)
%GETPLANEDISTANCE Summary of this function goes here
%   Detailed explanation goes here
% n = cross( X_i - X_j, X_i - X_l ) / norm (cross( X_i - X_j, X_i - X_l )) ;
% r = abs(  (X - X_i)' *  n ) / norm(n);

n = cross(X_i - X_j, X_i - X_l);
r = norm((X-X_i)'*n)/norm(n);
end

