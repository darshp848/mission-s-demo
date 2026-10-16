function K = ms_lqr_axis(tau, q, r)
%MS_LQR_AXIS  LQR gain for one attitude axis: double integrator + first-order
%   thruster lag, with integral action. State [int(e); e; e_dot; a], input a_cmd.
A = [0 1 0 0; 0 0 1 0; 0 0 0 1; 0 0 0 -1/tau];
B = [0; 0; 0; 1/tau];
K = lqr(A, B, diag(q), r);
end
