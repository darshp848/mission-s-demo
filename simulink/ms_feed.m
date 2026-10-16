function T_eff = ms_feed(T_cmd, P_tank, mprop, PP)
%MS_FEED  Feed-pressure limit and propellant cut-off ahead of the engine lag.
s = min(1, max(0, P_tank/PP.P_reg));
if mprop(1) <= 0.01 || mprop(2) <= 0.01, s = 0; end
T_eff = min(max(T_cmd, 0), PP.Tmax).*PP.thr_scale*s;
end
