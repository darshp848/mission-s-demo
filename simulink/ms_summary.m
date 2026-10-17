function [S,T]=ms_summary(L,P,info,PP)
%MS_SUMMARY Actual state, completion, collision, landing and trajectory export.
if nargin<4,PP=P;end
L=ms_log_matrix(L,P); c=ms_log_columns(P); assert(size(L,2)==c.width,'Unexpected simlog schema');
t=L(:,1);pos=L(:,c.pos);vel=L(:,c.vel);quat=L(:,c.q);rate=L(:,c.rate);
mode=round(L(:,c.mode));aux=L(:,c.aux);thrust=L(:,c.T);n=numel(t);
angles=zeros(n,3); upright=zeros(n,1); ground_clear=inf; minclr=inf; over='';
flying=mode>=2 & mode<=10;
for i=1:n
    R=ms_rot(quat(i,:).');
    angles(i,:)=[atan2(R(3,2),R(3,3)) asin(max(-1,min(1,R(3,1)))) atan2(R(2,1),R(1,1))];
    upright(i)=acos(max(-1,min(1,R(3,3))));
    if ~flying(i),continue;end
    cg=aux(i,2:4);[h,xmin,xmax]=ms_hbot(PP,pos(i,:).',quat(i,:).',cg);
    ground_clear=min(ground_clear,h);
    yrange=abs(R(2,1))*PP.L/2+abs(R(2,2))*PP.W/2+abs(R(2,3))*PP.H/2;
    for j=1:size(P.lineup,1)
        if xmax>=P.lineup_x(j,1) && xmin<=P.lineup_x(j,2) && abs(pos(i,2))<=P.lineup(j,2)/2+yrange
            if h-P.lineup(j,3)<minclr,minclr=h-P.lineup(j,3);over=info.lineup_names{j};end
        end
    end
end
S=struct();ev=struct();fe=@(m)t(find(mode==m,1));
ev.launch=fe(1);ev.liftoff=fe(2);ev.roll=fe(4);ev.pitch_kick=fe(6);ev.landing_burn=fe(10);
[ev.max_z_cg,k]=max(pos(:,3).*flying);ev.apex=t(k);
itd=find(mode==12,1);crashed=true;
if ~isempty(itd)
    ev.touchdown=t(itd);ev.touchdown_vz=vel(itd,3);ev.touchdown_y=pos(itd,2);
    ev.touchdown_roll_err_deg=rad2deg(angles(itd,1));ev.touchdown_pitch_err_deg=rad2deg(angles(itd,2));
    ev.touchdown_yaw_err_deg=rad2deg(angles(itd,3));ev.touchdown_tilt_deg=rad2deg(upright(itd));
    ev.touchdown_mode=info.mode_names{mode(max(1,itd-1))+1};
    crashed=~strcmp(ev.touchdown_mode,'landing_burn');
end
ist=find(mode==13,1);if ~isempty(ist),ev.stopped=t(ist);ev.stop_x=pos(ist,1);end
% Integrated body-axis rates measure the sequenced 360-degree maneuvers;
% acceptance also checks actual upright terminal quaternion, not mode entry.
rollmask=mode==4 | mode==5; pitchmask=mode>=6 & mode<=9;
S.roll_rotation_deg=rad2deg(trapz(t,rate(:,1).*rollmask));
S.pitch_rotation_deg=rad2deg(trapz(t,-rate(:,2).*pitchmask));
S.roll_done=abs(S.roll_rotation_deg-P.roll_dir*360)<20;
S.flip_done=abs(S.pitch_rotation_deg-360)<20 && any(mode==8) && any(mode==9);
S.min_clearance_m=minclr;S.min_clearance_over=over;S.min_ground_clearance_m=ground_clear;
S.max_lateral_drift_m=max(abs(pos(flying,2)));S.events_s=ev;
S.prop_used_kg.SuperDraco=trapz(t,sum(thrust(:,1:2)./PP.Isp(1:2).',2))/P.g;
S.prop_used_kg.Draco=trapz(t,sum(thrust(:,3:end)./PP.Isp(3:end).',2))/P.g;
S.prop_loaded_kg=P.m_prop0;S.prop_remaining_kg=sum(L(end,c.mprop));
S.copv_bar_end=aux(end,9)/1e5;S.mass_kg=struct('launch',aux(1,1),'end',aux(end,1));S.crashed=crashed;
S.finite_states=all(isfinite(L),'all');S.max_quaternion_norm_error=max(abs(vecnorm(quat,2,2)-1));
S.lineup_cleared=any(pos(:,1)>P.lineup_x(end,2)+P.L/2);
S.safe_landing=~crashed && ~isempty(itd) && abs(ev.touchdown_vz)<2.5 && ...
    ev.touchdown_tilt_deg<10 && abs(ev.touchdown_yaw_err_deg)<10 && minclr>.3 && ground_clear>-.01 && S.finite_states;
S.success=S.safe_landing && S.roll_done && S.flip_done && S.lineup_cleared;
if nargout<2,return;end
idx=zeros(0,1);next=0;
for i=1:n
    if t(i)+1e-9>=next,idx(end+1,1)=i;next=next+1/P.fps;end %#ok<AGROW>
end
m=numel(idx);rpm=zeros(m,1);gear=zeros(m,1);
for j=1:m
    if mode(idx(j))==1,[~,rpm(j),gear(j)]=ms_tractive(P,vel(idx(j),1));end
end
phase=info.mode_names(mode(idx)+1).';
fb=movmean(aux(:,11:13),[7 0]);
T=table(t(idx),pos(idx,1),pos(idx,2),pos(idx,3),angles(idx,1),angles(idx,2),angles(idx,3), ...
    L(idx,c.wheel),vecnorm(vel(idx,:),2,2),sum(thrust(idx,1:2),2),sum(thrust(idx,3:end),2),phase, ...
    fb(idx,1),fb(idx,2),fb(idx,3),vecnorm(fb(idx,:),2,2), ...
    100*L(idx,c.mprop(1))/P.m_f0,100*L(idx,c.mprop(2))/P.m_o0,aux(idx,9)/1e5,aux(idx,10)/1e5, ...
    aux(idx,2),aux(idx,3),aux(idx,4),aux(idx,1),rpm,gear,L(idx,c.F_drive), ...
    'VariableNames',{'t','x','y','z','roll','pitch','yaw','wheel_angle','speed','thrust_vert_N','thrust_diff_N', ...
    'phase','g_x','g_y','g_z','g_total','fuel_pct','ox_pct','copv_bar','tank_bar','cg_x','cg_y','cg_z','mass_kg','rpm','gear','F_drive_N'});
T.q_w=quat(idx,1);T.q_x=quat(idx,2);T.q_y=quat(idx,3);T.q_z=quat(idx,4);
for k=1:P.n_thr
    T.(['thr_' info.groups{k}])=100*thrust(idx,k)/P.Tmax(k);
    T.(['valve_' info.groups{k}])=L(idx,c.command(k))/P.Tmax(k);
end
end
