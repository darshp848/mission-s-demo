function Qs=ms_mc_draws(P,N)
%MS_MC_DRAWS Repeatable, bounded physical plant dispersion; controller nominal.
old=rng;cleanup=onCleanup(@()rng(old));rng(7);
Qs=repmat(P,N,1);
for i=1:N
    Q=P;Q.m_car=P.m_car*(1+.03*randn);
    Q.cg_car_off=[.05*randn .02*randn .02*randn];
    Q.I_scale=max(.5,min(1.5,1+.10*randn(1,3)));
    Q.thr_scale=max(.7,min(1.3,1+.03*randn(P.n_thr,1)));
    Q.Isp=P.Isp.*max(.8,min(1.2,1+.02*randn(P.n_thr,1)));
    Q.tau=P.tau*(1+.3*(2*rand-1));
    Q.aero_cda=P.aero_cda.*max(.5,min(1.5,1+.2*randn(6,1)));
    Q.aero_cl_area=P.aero_cl_area*max(.5,min(1.5,1+.2*randn));
    Q.aero_pos=P.aero_pos+.04*randn(6,3);
    Q.wind_mean=P.wind_mean+[.5*randn;.75*randn;.2*randn];
    Q.wind_gust=P.wind_gust.*(.5+rand(3,1));
    Qs(i)=ms_derive(Q);
end
end
