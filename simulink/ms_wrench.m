function B = ms_wrench(P,cg)
%MS_WRENCH Body wrench per newton of channel thrust: [force; moment].
% Directions mean force on vehicle; each SuperDraco channel is one chamber.
n = size(P.thr_pos,1); B = zeros(6,n);
for k = 1:n
    f=P.thr_dir(k,:).'; arm=P.thr_pos(k,:).'-cg(:);
    B(:,k)=[f;cross(arm,f)];
end
end
