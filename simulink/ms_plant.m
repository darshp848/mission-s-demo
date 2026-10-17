function [acc, angacc, mdot, wheel_rate, aux, qdot] = ms_plant(pos, vel, att, rate, mprop, T_act, F_drive, brake, PP)
%MS_PLANT 6DOF world translation and right-handed body angular dynamics.
% att is scalar-first BODY->WORLD quaternion; rate is body [p;q;r].
% Variable-mass approximation: fixed-location tank depletion, no slosh;
% d(I*w)/dt + w x (I*w) = applied torque. Thus Idot*w is retained.
% Exhaust momentum is included in channel thrust; there is no extra mdot*v
% force or separate co-rotating exhaust angular-momentum flux model.
% aux(1:13) retains legacy order; aux(14)=Izz.
g = PP.g;
m_f = max(mprop(1),0); m_o = max(mprop(2),0);
thrust = max(T_act(:),0);
if m_f <= 0 || m_o <= 0, thrust(:)=0; end
tot = sum(thrust./(PP.Isp*g));
mdot = -[tot/(1+PP.OF); tot*PP.OF/(1+PP.OF)];
[M,cg,Ixx,Iyy,Izz,I,Idot] = ms_mass_props(PP,m_f,m_o,mdot);
R = ms_rot(att);
wrench = ms_wrench(PP,cg)*thrust;
[Fa,Ma]=ms_aero(PP,pos,vel,att,rate,cg);
F=R*(wrench(1:3)+Fa);
torque=wrench(4:6)+Ma;

% Suspension carries the upright vehicle; tyres provide longitudinal force.
defl = PP.z_free-pos(3);
contact = defl > 0 && R(3,3) > 0.7;
if contact
    F(3)=F(3)+max(0,PP.k_gnd*defl-PP.c_gnd*vel(3));
    F(1)=F(1)+F_drive-brake*PP.a_brake*M*tanh(vel(1)/0.3);
    F(2)=F(2)-PP.c_lat*M*vel(2);
    tilt_error=cross([0;0;1],R.'*[0;0;1]);
    torque=torque+[PP.kr_gnd*tilt_error(1)-PP.cr_gnd*rate(1); ...
        PP.kp_gnd*tilt_error(2)-PP.cp_gnd*rate(2); -PP.cy_gnd*rate(3)];
    wheel_rate=vel(1)/PP.wheel_r;
else
    wheel_rate=0;
end
acc=F/M-[0;0;g];
angacc=I\(torque-cross(rate,I*rate)-Idot*rate);
qdot=0.5*ms_quat_mul(att,[0;rate(:)])+PP.q_norm_gain*(1-dot(att,att))*att;

V_u=PP.V_f-m_f/PP.rho_f+PP.V_o-m_o/PP.rho_o;
RT=PP.R_he*PP.T_gas;
P_copv=(PP.m_he-PP.P_reg*V_u/RT)*RT/PP.V_c;
if P_copv >= PP.P_reg
    P_tank=PP.P_reg;
else
    P_copv=PP.m_he*RT/(PP.V_c+V_u); P_tank=P_copv;
end
f_b=R.'*(acc+[0;0;g])/g;
h_bot=ms_hbot(PP,pos,att,cg);
aux=[M;cg(:);Ixx;Iyy;double(contact);h_bot;P_copv;P_tank;f_b;Izz];
end
