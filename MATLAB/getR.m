function R = getR(phi,theta,psi)
R = [cos(phi)*cos(theta) cos(phi)*sin(theta)*sin(psi)-sin(phi)*cos(psi) cos(phi)*sin(theta)*cos(psi)+sin(phi)*sin(psi);
    sin(phi)*cos(theta) sin(phi)*sin(theta)*sin(psi)+cos(phi)*cos(psi) sin(phi)*sin(theta)*cos(psi)-cos(phi)*sin(psi);
    -sin(theta) cos(theta)*sin(psi) cos(theta)*cos(psi)];
end