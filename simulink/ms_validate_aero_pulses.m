function report=ms_validate_aero_pulses()
[P,~]=ms_params();P.wind_mean(:)=0;P.wind_gust(:)=0;q=[1;0;0;0];pos=[0;0;20];cg=zeros(1,3);
[F,M]=ms_aero(P,pos,zeros(3,1),q,zeros(3,1),cg);assert(norm([F;M])==0);
[F,~]=ms_aero(P,pos,[20;0;0],q,zeros(3,1),cg);assert(F(1)<0 && F(3)<0);
P.aero_cl_area=0;old=rng;cleanup=onCleanup(@()rng(old));rng(12);
for k=1:80
    v=20*randn(3,1);w=3*randn(3,1);[F,M]=ms_aero(P,pos,v,q,w,cg);
    assert(dot(F,v)+dot(M,w)<=1e-8,'Panel drag must dissipate point-relative kinetic energy');
end
n=P.n_thr-2;st=[zeros(2*n,1);P.pulse_min_off*ones(n,1)];N=2000;g=zeros(N,n);
for k=1:N
    duty=.37;[u,st]=ms_pulse_schedule(duty*P.Tmax(3:end),1,st,P,true);g(k,:)=u.'./P.Tmax(3:end).';
end
assert(all(g==0|g==1,'all'));
for j=1:n
    edges=find(diff([0;g(:,j);0]));lengths=diff(edges); if g(end,j)==1,lengths=lengths(1:end-1);end
    assert(all(lengths(1:2:end)*P.Ts>=P.pulse_min_on-1e-9));
    if numel(lengths)>1,assert(all(lengths(2:2:end)*P.Ts>=P.pulse_min_off-1e-9));end
end
report.pulse_impulse_fraction=sum(g(:,1))/N;assert(abs(report.pulse_impulse_fraction-.37)<.005);
report.panel_dissipation=true;report.min_on_s=P.pulse_min_on;report.min_off_s=P.pulse_min_off;report.pass=true;disp(report);
end
