function stats=ms_pulse_report(L,P,info,outDir)
%MS_PULSE_REPORT High-rate evidence; rendered frame sampling cannot resolve20ms.
c=ms_log_columns(P);t=L(:,1);gate=L(:,c.command(3:end))./P.Tmax(3:end).';
assert(all(abs(gate-round(gate))<1e-10,'all'),'RCS command is not binary');
counts=zeros(P.n_thr-2,1);minon=inf(P.n_thr-2,1);minoff=minon;
for j=1:size(gate,2)
    changes=find(diff([0;gate(:,j);0]));
    for k=1:numel(changes)-1
        if changes(k+1)>numel(t),continue;end
        duration=t(changes(k+1))-t(changes(k));
        if mod(k,2)==1,counts(j)=counts(j)+1;minon(j)=min(minon(j),duration);
        else,minoff(j)=min(minoff(j),duration);end
    end
end
stats.total_pulses=sum(counts);stats.min_on_s=min(minon);stats.min_off_s=min(minoff);
stats.on_time_pass=stats.min_on_s>=P.pulse_min_on-1e-8;
stats.off_time_pass=stats.min_off_s>=P.pulse_min_off-1e-8;
% Immediate forceoff at touchdown is a permitted abort; record it explicitly.
Fa=zeros(numel(t),3);Ma=Fa;wind=Fa;air=Fa;
for i=1:numel(t)
    [f,m,a]=ms_aero(P,L(i,c.pos).',L(i,c.vel).',L(i,c.q).',L(i,c.rate).',L(i,c.aux(2:4)));
    Fa(i,:)=f.';Ma(i,:)=m.';air(i,:)=a.';wind(i,:)=ms_wind(P,L(i,c.pos).').';
end
stats.max_aero_force_N=max(vecnorm(Fa,2,2));stats.max_aero_moment_Nm=max(vecnorm(Ma,2,2));
stats.max_air_speed_mps=max(vecnorm(air,2,2));
if nargin<4,return;end
pulseTable=array2table([t gate L(:,c.T(3:end))./P.Tmax(3:end).'],'VariableNames', ...
    [{'t'} strcat('valve_',info.groups(3:end)) strcat('delivered_',info.groups(3:end))]);
writetable(pulseTable,fullfile(outDir,'pulse_telemetry.csv'));
A=array2table([t Fa Ma wind air],'VariableNames',{'t','Fx','Fy','Fz','Mx','My','Mz','wind_x','wind_y','wind_z','air_x','air_y','air_z'});
writetable(A,fullfile(outDir,'aero_telemetry.csv'));
fig=figure('Visible','off','Color',[.05 .06 .08],'Position',[50 50 1200 850]);tiledlayout(3,1);
nexttile;plot(t,Fa);grid on;ylabel('Body aero force N');legend('Fx','Fy','Fz');
nexttile;plot(t,Ma);grid on;ylabel('Body aero torque Nm');legend('Mx','My','Mz');
nexttile;sel=t>=5&t<=5.4;stairs(t(sel),gate(sel,4),'LineWidth',1.3);hold on;plot(t(sel),L(sel,c.T(6))/P.Tmax(6),'LineWidth',1.3);grid on;ylim([-.1 1.1]);ylabel('FR push fraction');xlabel('Time s');legend('Valve command','Delivered thrust after lag');
for ax=findall(fig,'Type','axes').'
    ax.Color=[.08 .10 .12];ax.XColor=[.85 .88 .92];ax.YColor=[.85 .88 .92];
end
exportgraphics(fig,fullfile(outDir,'aero_pulse_diagnostics.png'),'Resolution',120);close(fig);
fid=fopen(fullfile(outDir,'pulse_aero_metrics.json'),'w');fprintf(fid,'%s',jsonencode(stats,'PrettyPrint',true));fclose(fid);
end
