function [F, rpm_out, gear_out] = ms_tractive(P, v)
%MS_TRACTIVE  Max powertrain force at the wheels [N], engine rpm and gear.
w = max(v, 0)/P.wheel_r;
best = 0; rpm_out = P.launch_rpm; gear_out = 1;
for i = 1:numel(P.gears)
    gr = P.gears(i);
    rpm = w*gr*P.final_drive*60/(2*pi);
    if rpm > P.redline, continue; end
    if i > 1 && rpm < 2500, continue; end
    if i == 1, rpm_eff = max(rpm, P.launch_rpm); else, rpm_eff = rpm; end
    rc = min(max(rpm_eff, P.eng_rpm(1)), P.eng_rpm(end));
    Fi = interp1(P.eng_rpm, P.eng_Nm, rc)*gr*P.final_drive*P.eta/P.wheel_r;
    if Fi > best, best = Fi; rpm_out = rpm_eff; gear_out = i; end
end
vv = max(v, 0.5);
F = best + min(P.em_front_Fmax, P.em_front_kW*1e3/vv) + min(P.em_rear_Fmax, P.em_rear_kW*1e3/vv);
end
