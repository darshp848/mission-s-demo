function u = ms_allocate(B, v, lb, ub, weights)
%MS_ALLOCATE Weighted bounded least squares with releasable active bounds.
% Normalize actuator commands to their ranges; solve the free problem in
% wrench space. Unlike one-way clipping, a previously clamped actuator can
% re-enter when its KKT multiplier has the wrong sign.
n = size(B,2); m=size(B,1);
span=max(ub-lb,1e-12);
A=(B.*repmat(span.',m,1)).*repmat(weights,1,n);
b=weights.*(v-B*lb);
z=zeros(n,1); state=-ones(n,1);
state(ub<=lb)=2;
ridge=1e-10;
for outer=1:6*n
    grad=A.'*(A*z-b)+ridge*z;
    violation=zeros(n,1);
    for j=1:n
        if state(j)==-1, violation(j)=max(0,-grad(j));
        elseif state(j)==1, violation(j)=max(0,grad(j)); end
    end
    [worst,j]=max(violation);
    if worst<1e-8, break; end
    state(j)=0;
    for inner=1:n+1
        free=state==0;
        Af=A.*repmat(double(free.'),m,1);
        rhs=b-A*(z.*double(~free));
        candidate=Af.'*((Af*Af.'+ridge*eye(m))\rhs);
        candidate=candidate.*double(free)+z.*double(~free);
        delta=candidate-z; step=1; hit=0; bound=0;
        for k=1:n
            if free(k) && delta(k)<-1e-12 && candidate(k)<0
                a=-z(k)/delta(k);
                if a<step,step=max(0,a);hit=k;bound=-1;end
            elseif free(k) && delta(k)>1e-12 && candidate(k)>1
                a=(1-z(k))/delta(k);
                if a<step,step=max(0,a);hit=k;bound=1;end
            end
        end
        z=min(max(z+step*delta,0),1);
        if hit==0,break;end
        state(hit)=bound;
        if bound==-1,z(hit)=0;else,z(hit)=1;end
    end
end
u=min(max(lb+span.*z,lb),ub);
end
