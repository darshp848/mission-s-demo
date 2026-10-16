function S=ms_debug_run(i,N)
%MS_DEBUG_RUN Reproduce and preserve an individual dispersed mission.
if nargin<2,N=50;end
[P,info]=ms_params();Qs=ms_mc_draws(P,N);
in=Simulink.SimulationInput('mission_s');in=in.setVariable('P',P).setVariable('PP',Qs(i));validate(in);out=sim(in);
L=ms_log_matrix(out.simlog,P);S=ms_summary(L,P,info,Qs(i));disp(S);
od=fullfile(fileparts(mfilename('fullpath')),'..','out','revised');
save(fullfile(od,sprintf('debug_run_%d.mat',i)),'L','S');
c=ms_log_columns(P);k=find(diff(round(L(:,c.mode)))~=0);
for j=k.'
    fprintf('t=%.3f mode%d -> %d z=%.2f vz=%.2f rates=%s\n',L(j+1,1),L(j,c.mode),L(j+1,c.mode),L(j+1,4),L(j+1,7),mat2str(L(j+1,c.rate),3));
end
end
