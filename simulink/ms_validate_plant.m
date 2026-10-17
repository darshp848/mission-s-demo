function report = ms_validate_plant()
%MS_VALIDATE_PLANT Bounded math checks without model compilation or simulation.
[P,~]=ms_params(); P.wind_mean(:)=0;P.wind_gust(:)=0; q0=[1;0;0;0]; mp=[P.m_f0;P.m_o0];
[a,wd,~,~,aux,qd]=ms_plant(P.pos0,zeros(3,1),q0,zeros(3,1),mp,zeros(P.n_thr,1),0,0,P);
assert(norm(a)<1e-10 && norm(wd)<1e-10 && norm(qd)<1e-10);
assert(numel(aux)==14 && P.n_thr==28 && numel(P.mem0)==22+3*(P.n_thr-2));
q=[sqrt(.5);0;0;sqrt(.5)];
assert(norm(ms_rot(q)*[1;0;0]-[0;1;0])<1e-12);
[~,cg,~,~,~,I,Id]=ms_mass_props(P,mp(1),mp(2),[-1;-2]);
[~,~,~,~,~,I2]=ms_mass_props(P,mp(1)-1e-5,mp(2)-2e-5);
report.Idot_error=norm((I2-I)/1e-5-Id);
assert(report.Idot_error<1e-5 && norm(I-I.')<1e-12 && min(eig(I))>0);
B=ms_wrench(P,cg); report.wrench_rank=rank(B);
assert(report.wrench_rank==6 && B(5,1)<0 && B(5,2)>0);
[~,~,~,~,~,qd]=ms_plant([0;0;50],zeros(3,1),q,[.1;.2;.3],mp,zeros(P.n_thr,1),0,0,P);
assert(abs(dot(q,qd))<1e-12);
assert(all(ms_feed(P.Tmax,-1,mp,P)==0));
assert(all(ms_feed(P.Tmax,P.P_reg,[0;mp(2)],P)==0));
assert(norm(ms_feed(P.Tmax,P.P_reg,mp,P)-P.Tmax)<1e-10);
% Free rigid body conserves kinetic energy instantaneously with no depletion.
w=[.7;-.4;.3]; P.aero_cda(:)=0;P.aero_cl_area=0;
[~,wd]=ms_plant([0;0;50],zeros(3,1),q0,w,mp,zeros(P.n_thr,1),0,0,P);
report.energy_rate=w.'*I*wd; assert(abs(report.energy_rate)<1e-10);
% RK4 quaternion integration spans two complete turns around an oblique axis.
w=[.3;-.4;.5]; dt=.001; n=ceil(4*pi/norm(w)/dt); qt=q0;
for k=1:n
    k1=qrate(qt,w,P); k2=qrate(qt+.5*dt*k1,w,P);
    k3=qrate(qt+.5*dt*k2,w,P); k4=qrate(qt+dt*k3,w,P);
    qt=qt+dt*(k1+2*k2+2*k3+k4)/6;
end
report.quaternion_norm_error=abs(norm(qt)-1);
assert(report.quaternion_norm_error<1e-10);
report.pass=true; disp(report);
end
function d=qrate(q,w,P)
d=.5*ms_quat_mul(q,[0;w])+P.q_norm_gain*(1-dot(q,q))*q;
end
