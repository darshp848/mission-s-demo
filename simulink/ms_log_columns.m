function c=ms_log_columns(P)
%MS_LOG_COLUMNS Shared telemetry schema (no fixed thruster-count offsets).
c.t=1;c.pos=2:4;c.vel=5:7;c.q=8:11;c.rate=12:14;c.mprop=15:16;c.wheel=17;
c.T=18:17+P.n_thr;c.aux=c.T(end)+(1:14);c.mode=c.aux(end)+1;
c.F_drive=c.mode+1;c.brake=c.mode+2;c.command=c.brake+(1:P.n_thr);c.width=c.command(end);
end
