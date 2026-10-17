function R=ms_robustness(N,over,outDir)
%MS_ROBUSTNESS Linear local margins and seeded nonlinear dispersed missions.
if nargin<1,N=50;end
if nargin<2,over=struct();end
validateattributes(N,{'numeric'},{'scalar','integer','positive'});
here=fileparts(mfilename('fullpath'));addpath(here);
if nargin<3,outDir=fullfile(here,'..','out','aero_pulsed');end
if ~exist(outDir,'dir'),mkdir(outDir);end
[P,info]=ms_params();
for f=fieldnames(over).',P.(f{1})=over.(f{1});end
P=ms_derive(P);
axes_={'roll',P.K_roll,P.DR_tau;'pitch',P.K_pitch,P.DR_tau;'yaw',P.K_yaw,P.DR_tau;'pitch_SD',P.K_pitch_SD,P.SD_tau};
rows=zeros(24,7);j=0;
for a=1:4
    K=axes_{a,2};tc=axes_{a,3};
    C=tf([K(3) K(2) K(1)],[1 0])*tf([tc 1],[tc 1+K(4)]);
    for kI=[.7 1 1.3]
        for lagf=[1 1.5]
            G=tf(1/kI,[tc*lagf 1 0 0]);Lp=C*G;Lp.InputDelay=P.Ts;m=allmargin(Lp);
            j=j+1;rows(j,:)=[a kI 1e3*tc*lagf 20*log10(min([m.GainMargin(m.GainMargin>1),inf])) ...
                min([m.PhaseMargin,inf]) 1e3*min([m.DelayMargin,inf]) double(m.Stable)];
        end
    end
end
R.margins=array2table(rows,'VariableNames',{'axis','inertia_ratio','lag_ms','GM_dB','PM_deg','DM_ms','stable'});
writetable(R.margins,fullfile(outDir,'loop_margins.csv'));
% These linear margins exclude allocation saturation and trajectory coupling;
% the actual nonlinear plant below supplies mission-level evidence.
Qs=ms_mc_draws(P,N);in=repmat(Simulink.SimulationInput('mission_s'),N,1);
for i=1:N
    in(i)=in(i).setVariable('P',P).setVariable('PP',Qs(i));validate(in(i));
end
fprintf('Starting %d Monte Carlo runs, seed 7\n',N);tic;
outs=sim(in,'UseFastRestart','on','ShowProgress','on','StopOnError','off');
wall=toc;metrics=nan(N,14);errors=strings(N,1);
for i=1:N
    if strlength(string(outs(i).ErrorMessage))>0
        errors(i)=string(outs(i).ErrorMessage);metrics(i,1:2)=0;continue;
    end
    S=ms_summary(outs(i).simlog,P,info,Qs(i));e=S.events_s;
    metrics(i,[1 2 7:14])=[S.success S.safe_landing S.min_clearance_m S.roll_rotation_deg S.pitch_rotation_deg ...
        S.prop_remaining_kg S.max_quaternion_norm_error S.finite_states S.lineup_cleared S.crashed];
    if isfield(e,'touchdown')
        metrics(i,3:6)=[e.touchdown_vz e.touchdown_tilt_deg e.touchdown_yaw_err_deg e.touchdown_y-P.y_side];
    end
end
R.mc=array2table([(1:N).' metrics],'VariableNames',{'run','success','safe_landing','touchdown_vz','touchdown_tilt_deg', ...
    'touchdown_yaw_deg','y_miss_m','min_clearance_m','roll_rotation_deg','pitch_rotation_deg','prop_remaining_kg', ...
    'quaternion_norm_error','finite_states','lineup_cleared','crashed'});
R.mc.error=errors;R.seed=7;R.wall_time_s=wall;
R.full_stunt_pass=sum(R.mc.success==1);R.safe_landing_pass=sum(R.mc.safe_landing==1);R.N=N;
writetable(R.mc,fullfile(outDir,'monte_carlo.csv'));
save(fullfile(outDir,'monte_carlo.mat'),'R','P','Qs','info','-v7.3');
fid=fopen(fullfile(outDir,'robustness.json'),'w');fprintf(fid,'%s',jsonencode(rmfield(R,{'mc','margins'}),'PrettyPrint',true));fclose(fid);
fprintf('Full stunt %d/%d | safe landing %d/%d | elapsed %.1fs\n',R.full_stunt_pass,N,R.safe_landing_pass,N,wall);
if any(R.mc.success~=1),disp(R.mc(R.mc.success~=1,:));end
end
