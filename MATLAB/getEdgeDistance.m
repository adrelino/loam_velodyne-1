function r = getEdgeDistance( X_0, X_i, X_j )
%GETEDGEDISTANCE Summary of this function goes here
%   Detailed explanation goes here
r = norm( cross( (X_0 - X_i), (X_0 - X_j) ) ) / norm( X_i - X_j );
end

