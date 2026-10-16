%% Run the authoritative quaternion 6DOF model.
here=fileparts(mfilename('fullpath'));
addpath(fullfile(here,'..','simulink'));
build_mission_s_model;
run_mission_s;
