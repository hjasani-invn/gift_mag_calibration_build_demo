function [C] = GetLocal2BodyMatrix(ang)

psi = ang(1);
theta = ang(2);
fi = ang(3);

A1 = [ cos(psi) -sin(psi)   0;
    sin(psi)    cos(psi)    0;
    0   0   1 ];

A2 = [ 1    0   0;
       0    cos(theta)  -sin(theta);
       0    sin(theta)  cos(theta) ];

A3 = [ cos(fi) -sin(fi) 0;
    sin(fi)  cos(fi)    0;
    0   0   1];

C = A1 * A2 * A3;
C = C'; % C is local to phone now

% C = eye(3,3);

end