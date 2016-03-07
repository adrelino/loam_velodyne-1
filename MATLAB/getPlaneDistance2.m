function r = getPlaneDistance2(norm_k, point_k, point_k1)
%GETPLANEDISTANCE Summary of this function goes here
%   Detailed explanation goes here
% n = cross( X_i - X_j, X_i - X_l ) / norm (cross( X_i - X_j, X_i - X_l )) ;
% r = abs(  (X - X_i)' *  n ) / norm(n);
r = ((point_k1-point_k)'*norm_k)/norm(norm_k);
end

