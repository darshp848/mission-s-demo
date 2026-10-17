function [force,moment,air]=ms_aero(P,pos,vel,q,rate,cg)
%MS_AERO All-attitude, dissipative panel surrogate; coefficients are assumed.
% Point velocity includes rigid-body rotation. Each exposed windward surface
% experiences quadratic normal drag; force is applied at its panel center.
R=ms_rot(q);air=R.'*(vel-ms_wind(P,pos));
force=zeros(3,1);moment=zeros(3,1);
for j=1:6
    r=P.aero_pos(j,:).'-cg(:); normal=P.aero_normal(j,:).';
    local=air+cross(rate,r);vn=max(0,dot(local,normal));
    f=-.5*P.rho*P.aero_cda(j)*vn^2*normal;
    force=force+f;moment=moment+cross(r,f);
end
% Downforce is perpendicular to relative flow and hence does no air-relative
% translational work. It smoothly vanishes for purely lateral/vertical flow.
speed=norm(air);
if speed>1e-6
    vhat=air/speed; lift_axis=[0;0;1]-vhat*vhat(3);
    f=-.5*P.rho*P.aero_cl_area*air(1)^2*lift_axis;
    force=force+f;moment=moment+cross(P.aero_lift_pos(:)-cg(:),f);
end
end
