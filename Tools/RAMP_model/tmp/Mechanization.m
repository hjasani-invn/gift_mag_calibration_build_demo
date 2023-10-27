function [ q_next, V_next, R_next ] = Mechanization( q_L2B, delta_q, A, b_A, sf_A, W, b_W, V_l, delta_V_l, R_l, tau )

% this function only propagates for 1 step ("tau" is delta between two gyro measurements)
% gyro and acc measurements are considered to be synchronized
q_L2B = q_L2B + delta_q;
A = A - b_A;

A(1) = A(1) * sf_A(1);
A(2) = A(2) * sf_A(2);
A(3) = A(3) * sf_A(3);

% input acceleration A is in body frame, we need to rotate it using the current orientation quat

quat_A = [0; A]';
quat_A_l = quatmultiply(q_L2B , quatmultiply(quat_A, quatconj(q_L2B)));
A_l = quat_A_l(2:4)';

g_l = [0; 0; 9.81]; % check that g value here is the same as in model
A_dyn_l = A_l - g_l;

V_comp_l = V_l + delta_V_l; % 3d vector

V_next = V_comp_l + A_dyn_l * tau;

R_next = R_l + V_comp_l*tau + 1/2 * (A_dyn_l * tau^2);

W = W - b_W;
qw = [0 W(1) W(2) W(3)];

q_L2B = q_L2B/norm(q_L2B);

dq = 0.5 * quatmultiply(q_L2B, qw);
q_L2B = q_L2B + dq*tau;

q_next = q_L2B;

q_next = q_next / norm(q_next);
end

