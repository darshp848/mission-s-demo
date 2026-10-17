function S=run_mission_s(outDir)
%RUN_MISSION_S Simulate the revised 6DOF model and export a matching scene.
here=fileparts(mfilename('fullpath'));addpath(here);
if nargin<1,outDir=fullfile(here,'..','out','aero_pulsed');end
if ~exist(outDir,'dir'),mkdir(outDir);end
[P,info]=ms_params();PP=P;
in=Simulink.SimulationInput('mission_s');
in=in.setVariable('P',P).setVariable('PP',PP);
validate(in);tic;out=sim(in);elapsed=toc;
L=ms_log_matrix(out.simlog,P);[S,T]=ms_summary(L,P,info,PP);S.wall_time_s=elapsed;
S.lineup=struct('name',info.lineup_names.','x0',num2cell(P.lineup_x(:,1)), ...
    'x1',num2cell(P.lineup_x(:,2)),'len',num2cell(P.lineup(:,2)),'h',num2cell(P.lineup(:,3)));
S.car=struct('L',P.L,'W',P.W,'H',P.H,'h_cg',P.h_cg0,'h_ref',P.h_ref,'wheel_r',P.wheel_r,'m',P.M0);
S.pulse_aero=ms_pulse_report(L,PP,info,outDir);
S.environment=ms_environment_config(P);
S.fps=P.fps;S.source='Simulink R2026a quaternion 6DOF with panel aero and pulsed RCS';
S.thruster_layout=jsondecode(fileread(fullfile(here,'..','thruster_layout.json')));
S.provenance=struct('matlab',version,'model','simulink/mission_s.slx','generated',char(datetime('now','TimeZone','UTC')));
writetable(T,fullfile(outDir,'trajectory.csv'));save(fullfile(outDir,'nominal.mat'),'L','P','PP','S');
fid=fopen(fullfile(outDir,'scene.json'),'w');fprintf(fid,'%s',jsonencode(S,'PrettyPrint',true));fclose(fid);
fig=figure('Name','Revised Mission S 6DOF','Visible','off','Color','w','Position',[50 50 1100 950]);
c=ms_log_columns(P);tiledlayout(fig,5,1,'TileSpacing','compact');
nexttile;plot(T.x,T.z);grid on;ylabel('CG height m');title('Mission S: quaternion 6DOF, shared integrated thruster layout');
nexttile;plot(T.t,T.y);grid on;ylabel('Lateral m');yline(P.y_side,'--');
nexttile;plot(L(:,1),rad2deg(L(:,c.rate)));grid on;ylabel('Body rate deg/s');legend('p','q','r');
nexttile;plot(T.t,[T.thrust_vert_N T.thrust_diff_N]/1000);grid on;ylabel('Thrust kN');legend('Main','RCS');
nexttile;stairs(L(:,1),L(:,c.mode));grid on;xlabel('Time s');ylabel('Mode');
exportgraphics(fig,fullfile(outDir,'mission_s_simulink.png'),'Resolution',120);close(fig);
fprintf('Nominal %.1fs wall | success %d | safe %d | roll %.1fdeg | flip %.1fdeg | clearance %.2fm\n',elapsed,S.success,S.safe_landing,S.roll_rotation_deg,S.pitch_rotation_deg,S.min_clearance_m);
disp(S.events_s);
end
