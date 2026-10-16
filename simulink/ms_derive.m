function P = ms_derive(P)
%MS_DERIVE  Recompute quantities that depend on the masses (call after editing P).
P.m_f0 = P.m_prop0/(1 + P.OF);
P.m_o0 = P.m_prop0 - P.m_f0;
P.V_f = P.m_f0/P.rho_f/(1 - P.ullage0);
P.V_o = P.m_o0/P.rho_o/(1 - P.ullage0);
P.V_c = 2*P.copv_V;
V_u0 = P.V_f - P.m_f0/P.rho_f + P.V_o - P.m_o0/P.rho_o;
P.m_he = (P.copv_P0*P.V_c + P.P_reg*V_u0)/(P.R_he*P.T_gas);
[M0, cg0, Ixx0, Iyy0, Izz0] = ms_mass_props(P, P.m_f0, P.m_o0);
P.M0 = M0;
P.h_cg0 = P.h_ref + cg0(3);
P.pos0 = [0; 0; P.h_cg0];
P.z_free = P.h_cg0 + M0*P.g/P.k_gnd;
P.c_gnd = 2*P.zeta_gnd*sqrt(P.k_gnd*M0);
w = P.w_att_gnd;
P.kr_gnd = Ixx0*w^2; P.cr_gnd = 2*P.zeta_gnd*w*Ixx0;
P.kp_gnd = Iyy0*w^2; P.cp_gnd = 2*P.zeta_gnd*w*Iyy0;
P.cy_gnd = 2*P.zeta_gnd*w*Izz0;
end
