function [T_cmd,F_drive,brake,mem_out,mode,done] = ms_flight_computer(t,pos,vel,q,rate,mprop,aux,mem,P)
%MS_FLIGHT_COMPUTER Quaternion 3-axis feedback, mission sequence, 6D allocation.
% Memory contract is documented in ms_params. Rates/moments are RIGHT HANDED;
% the display convention uses positive nose-up pitch, opposite body y rate.
mode=mem(1); tm=mem(2); roll_end=mem(3); pitch_ref=mem(4);
inte=mem(5:7); ahat=mem(8:10); tstop=mem(11); tdmode=mem(12);
Tr=mem(13); sdhat=mem(14); roll_progress=mem(15); pitch_progress=mem(16);
[M,cg,Ixx,Iyy,Izz,I]=ms_mass_props(P,max(mprop(1),0),max(mprop(2),0));
R=ms_rot(q); estimate=P;estimate.wind_gust=P.control_wind_gust;
[Fa,Ma]=ms_aero(estimate,pos,vel,q,rate,cg); Faw=R*Fa;
hbot=ms_hbot(P,pos,q,cg); B=ms_wrench(P,cg);
T_cmd=zeros(size(P.Tmax,1),1); F_drive=0; brake=0; done=0; next=mode;
g=P.g; Ts=P.Ts; SDlo=P.SD_T*P.SD_tmin;
if mode==0
    if t>=P.t_hold,next=1;end
elseif mode==1
    v=vel(1); Fmax=interp1(P.trac_v,P.trac_F,min(max(v,P.trac_v(1)),P.trac_v(end)));
    F_drive=max(0,min([Fmax,P.mu*M*g,.5*P.rho*P.CdA*v*v+M*4*(P.v_cruise-v)]));
    if pos(1)>=P.x_liftoff,next=2;F_drive=0;end
elseif mode<=10
    rollref=roll_end; wr=zeros(3,1); ar=zeros(3,1);
    if mode<4,rollref=0;
    elseif mode==4
        [s,ds,dds]=min_jerk(tm/Tr); A=P.roll_dir*2*pi;
        rollref=A*s; wr(1)=A*ds/Tr; ar(1)=A*dds/Tr^2;
        roll_progress=roll_progress+rate(1)*Ts;
    end
    % Feedforward sidestep during the roll; side rows then hold the target.
    if mode==4
        [s,ds,dds]=min_jerk(tm/Tr);
        yr=P.y_side*s; vyr=P.y_side*ds/Tr; ayr=P.y_side*dds/Tr^2;
    elseif mode<4,yr=0;vyr=0;ayr=0;
    else,yr=P.y_side;vyr=0;ayr=0;end
    vy_eff=vel(2)+R(2,3)*sdhat/M*P.SD_tau;
    ay=max(-P.ay_max,min(P.ay_max,ayr+P.k_vy*(vyr+P.k_y_pos*(yr-pos(2))-vy_eff)));
    sdmode=mode==2 || mode==10;
    if sdmode
        tilt=P.tilt_max*min(1,max(hbot,0)/P.tilt_fade_h);
        rollref=rollref-asin(max(-sin(tilt),min(sin(tilt),ay/g)));
    end
    flip=mode>=6 && mode<=8;
    pref=pitch_ref;
    if flip
        pitch_progress=pitch_progress-rate(2)*Ts;
        pref=pitch_progress;
        wr(2)=rate(2); % pitch is flown by explicit bang/coast/catch below
    end
    qref=ms_quat_mul([cos(pref/2);0;-sin(pref/2);0],[cos(rollref/2);sin(rollref/2);0;0]);
    qe=ms_quat_mul([q(1);-q(2:4)],qref);
    e=-2*qe(2:4); if qe(1)<0,e=-e;end
    inte=max(-P.int_lim,min(P.int_lim,inte+Ts*e));
    accel=zeros(3,1);
    gains=[P.K_roll;P.K_pitch;P.K_yaw];
    if sdmode,gains(2,:)=P.K_pitch_SD;end
    for axis=1:3
        accel(axis)=ar(axis)-gains(axis,:)*[inte(axis);e(axis);rate(axis)-wr(axis);ahat(axis)-ar(axis)];
    end
    if flip,inte(2)=0;accel(2)=0;end
    moments=I*accel+cross(rate,I*rate)-Ma;
    % Main chambers retain independent throttle, each split into equal outlets.
    Tsd=zeros(2,1);
    if sdmode
        if mode==2
            Fz=(M*(g+P.a_boost)-Faw(3))/max(R(3,3),.5);
        else
            vzref=-max(P.v_touch,sqrt(2*P.a_burn*max(hbot,0)));
            Fz=(M*(g+P.k_vz*(vzref-vel(3)))-Faw(3))/max(R(3,3),.3);
        end
        TF=(moments(2)-B(5,2)*Fz)/(B(5,1)-B(5,2));
        Tsd=[min(max(TF,SDlo),P.SD_T);min(max(Fz-TF,SDlo),P.SD_T)];
        if mode==10 && Fz<1.8*SDlo,Tsd(:)=0;end
    elseif mode==4 && abs(roll_progress)>=P.roll_sd_window && R(2,3)*ay>P.ay_sd_on
        Fz=2*P.roll_sd_throttle*P.SD_T;
        TF=(moments(2)-B(5,2)*Fz)/(B(5,1)-B(5,2));
        Tsd=[min(max(TF,SDlo),P.SD_T);min(max(Fz-TF,SDlo),P.SD_T)];
    elseif mode==6,Tsd=[P.SD_T;0];
    elseif mode==8,Tsd=[0;P.SD_T];end
    % Match world horizontal forces while attitude has first call on authority.
    world_force=[M*P.k_vx*(P.v_cruise-vel(1));M*ay;0]-Faw;
    fbody=R.'*world_force;
    if sum(Tsd)>0,fbody(3)=sum(Tsd);end
    desired=[fbody;moments];
    if flip,desired(5)=B(5,1:2)*Tsd;end
    residual=desired-B(:,1:2)*Tsd;
    rcs=ms_allocate(B(:,3:end),residual,zeros(size(P.Tmax,1)-2,1),P.Tmax(3:end),P.alloc_weights);
    T_cmd=[Tsd;rcs];
    delivered=B*T_cmd;
    acc_est=I\(delivered(4:6)+Ma-cross(rate,I*rate));
    tau=[P.DR_tau;P.DR_tau;P.DR_tau]; if sdmode,tau(2)=P.SD_tau;end
    ahat=ahat+(1-exp(-Ts./tau)).*(acc_est-ahat);
    sdhat=sdhat+(1-exp(-Ts/P.SD_tau))*(sum(Tsd)-sdhat);
    % No integration of saturation-induced attitude error.
    for axis=1:3
        if abs(accel(axis)-acc_est(axis))>2 && sign(e(axis))==sign(-accel(axis))
            inte(axis)=mem(4+axis);
        end
    end
    switch mode
        case 2
            if tm>=P.t_boost,next=3;end
        case 3
            if tm>=P.t_roll_delay
                rollcap=sum(max(0,P.roll_dir*B(4,3:10)).*P.Tmax(3:10).')/Ixx;
                Tr=max(P.T_roll,sqrt(5.7735*2*pi/max(P.roll_auth_frac*rollcap,.1)));
                roll_progress=0;next=4;
            end
        case 4
            if tm>=Tr,roll_end=P.roll_dir*2*pi;next=5;end
        case 5
            settled=norm(e)<deg2rad(5) && norm(rate)<.5;
            if tm>=P.pitch_gap && (settled || tm>=P.pitch_gap+P.settle_timeout)
                if hbot>=P.h_flip_min,next=6;pitch_progress=0;else,next=9;end
            end
        case 6
            if -rate(2)>=P.omega_pitch,next=7;end
        case 7
            alpha=abs(B(5,2))*P.SD_T/Iyy;
            if pitch_progress>=2*pi-rate(2)^2/(2*alpha)-abs(rate(2))*P.SD_tau,next=8;end
        case 8
            alpha=abs(B(5,2))*P.SD_T/Iyy;
            if -rate(2)<=alpha*P.SD_tau,pitch_ref=2*pi;inte(2)=0;ahat(2)=0;next=9;end
        case 9
            if vel(3)<0 && hbot<=vel(3)^2/(2*P.a_burn)+P.burn_margin,next=10;end
    end
    if aux(7)>.5 && mode>=3,tdmode=mode;next=12;T_cmd(:)=0;end
elseif mode==12
    brake=1;
    if abs(vel(1))<.05,tstop=t;next=13;end
else
    brake=1;
    if t-tstop>=P.t_end_hold,done=1;end
end
% Pulse-duration control is downstream of continuous wrench allocation.
[pulsed,pulse_state]=ms_pulse_schedule(T_cmd(3:end),aux(10)/P.P_reg,mem(23:end),P,mode>=2 && mode<=10 && next<12);
T_cmd(3:end)=pulsed;
if next~=mode,tm=0;else,tm=tm+Ts;end
mem_out=[next;tm;roll_end;pitch_ref;inte;ahat;tstop;tdmode;Tr;sdhat;roll_progress;pitch_progress;q(:);0;0;pulse_state];
end

function [s,ds,dds]=min_jerk(tau)
tau=min(max(tau,0),1);
s=10*tau^3-15*tau^4+6*tau^5;
ds=30*tau^2-60*tau^3+30*tau^4;
dds=60*tau-180*tau^2+120*tau^3;
end
