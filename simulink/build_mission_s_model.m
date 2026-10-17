function build_mission_s_model()
%BUILD_MISSION_S_MODEL Open and compile the checked-in revised model safely.
% Structural model source is mission_s.slx. This entry point intentionally
% preserves the checked-in model without rebuilding it.
here=fileparts(mfilename('fullpath'));addpath(here);f=fullfile(here,'mission_s.slx');
assert(isfile(f),'MissionS:MissingModel','Restore the revised mission_s.slx before running.');
[P,~]=ms_params();open_system(f);
in=Simulink.SimulationInput('mission_s');
in=in.setVariable('P',P).setVariable('PP',P).setModelParameter('StopTime','0');
validate(in);out=sim(in);L=ms_log_matrix(out.simlog,P); %#ok<NASGU>
fprintf('Revised mission_s.slx compiled: quaternion4, body rates3, %d channels\n',P.n_thr);
end
