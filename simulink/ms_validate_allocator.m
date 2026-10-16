function ms_validate_allocator()
[P,~]=ms_params();[~,cg]=ms_mass_props(P,P.m_f0,P.m_o0);B=ms_wrench(P,cg);B=B(:,3:end);ub=P.Tmax(3:end);w=P.alloc_weights;
old=rng;cleanup=onCleanup(@()rng(old));rng(42);
A=(B.*ub.').*w; opts=optimoptions('lsqlin','Display','off','OptimalityTolerance',1e-9);
maxgap=0;
for k=1:40
    if k<=25,v=B*(ub.*rand(numel(ub),1));else,v=randn(6,1)./[1/3000;1/3000;1/6000;1/10000;1/12000;1/2000];end
    u=ms_allocate(B,v,zeros(size(ub)),ub,w);
    oracle=lsqlin(A,w.*v,[],[],[],[],zeros(size(ub)),ones(size(ub)),[],opts);
    cost=norm(w.*(B*u-v))^2;ref=norm(A*oracle-w.*v)^2;
    gap=(cost-ref)/max(1,ref);maxgap=max(maxgap,gap);
    assert(all(u>=-1e-9 & u<=ub+1e-9),'Actuator bound failure');
    assert(gap<2e-5,'Allocator differs from independent lsqlin optimum: gap %g',gap);
end
fprintf('Allocator PASS:40 targets against lsqlin, maximum relative objective gap %.3g\n',maxgap);
end
