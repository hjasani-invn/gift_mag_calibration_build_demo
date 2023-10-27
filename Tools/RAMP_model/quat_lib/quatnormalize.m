function qout = quatnormalize( q )

q_abs = sqrt(q(1)^2 + q(2)^2 + q(3)^2 + q(4)^2);


qout = q * q_abs;