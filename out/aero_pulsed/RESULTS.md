# Aerodynamics + pulsed attitude thrusters

[Flight video](mission_s_telemetry.mp4) · [8× slow-motion valve close-up](pulse_slow_motion.mp4) · [Diagnostics](aero_pulse_diagnostics.png)

This revision replaces continuously throttled small jets with fixed-thrust valve
commands. An impulse-error accumulator converts requested mean force into variable
pulse duration/frequency. The assumed minimum on-time is20ms and minimum off-time
10ms. Existing5ms command latency and30ms thrust rise/fall remain. Main SuperDraco
commands retain throttle control. Propellant is still the original MMH/NTO concept,
not cold gas; a cold-gas conversion requires a different tank/feed/performance model.

Aerodynamics now use local air-relative velocity at six differently oriented
surfaces, including rotational point velocity omega×r. Windward normal forces scale
with squared normal airspeed; their offsets from the instantaneous CG produce roll,
pitch and yaw moments and rotational damping. A separate perpendicular downforce
term acts at an offset center. The simulation includes1.5m/s mean crosswind plus
spatially varying gusts. Feedback uses vehicle attitude/rates; feedforward estimates
nominal aerodynamic loads and mean wind, without knowledge of actual gusts.

These effective drag areas, downforce coefficient and surface centers are explicitly
assumed. This is an all-attitude surrogate, not measured MissionS CFD/wind-tunnel
coefficients; no claim of hardware readiness follows from passing simulations.
The rate-lag observer still uses averaged allocation. Sensors are ideal and valve
latency/lag are simplified. The pulse close-up uses actual1kHz model logging; the
normal24fps flight video cannot resolve every20ms valve event.

| Evidence | Result |
|---|---|
| Nominal full stunt / landing | PASS / PASS |
| Seed7 MC incl.aero coefficients, CP centers and wind |50/50 full stunt and safe landing|
| Nominal roll / flip |-359.37° /360.53°|
| Nominal touchdown sink / tilt |1.82m/s /0.40°|
| Minimum lineup clearance |5.13m|
| RCS pulses counted |209|
| Minimum measured on / off |20ms /25ms|
| Peak aerodynamic force / torque |626N /73N·m|
| Total propellant used |82.76kg (previous model79.14kg)|
| RCS propellant used |20.17kg (previous16.81kg)|
| Maximum drift |2.030m; touchdowny1.9994m|

Compared with the old video, the flight path is close because speeds are modest
and thrust authority is large. The differences are more visible in valve switching,
small attitude corrections and propellant use. This does not establish optimality.

Numerical sensitivity: changing fixed integration step from1ms to0.5ms and0.25ms
produced descent speeds1.816,1.657 and1.763m/s and touchdown times10.415,10.490 and
10.460s respectively. All three runs passed the mission/landing gates. Discontinuous
landing/valve switching makes the exact touchdown prediction step-sensitive;
no asymptotic convergence or exact landing speed is claimed.

Tests:80 randomized panel energy-dissipation checks; zero-flow zero-wrench and
forward-flow/downforce signs;10s pulse impulse tracking within0.1percentage points;
binary output and minimum dwell checks; quaternion/inertia invariants;40 allocator
comparisons against lsqlin; compile,5s diagnostic, nominal and50 dispersed flights.

Files: `pulse_telemetry.csv` stores1kHz valve commands/delivered fractions;
`aero_telemetry.csv` stores1kHz force/moment/wind/airspeed; `scene.json` and
`monte_carlo.csv` preserve actual outcomes. New source: `ms_aero.m`, `ms_wind.m`,
`ms_pulse_schedule.m`, `ms_pulse_report.m`, `ms_validate_aero_pulses.m`.

Research references: [NASA drag equation](https://www1.grc.nasa.gov/beginners-guide-to-aeronautics/drag-equation/),
[NASA RCS minimum firing-time tests](https://www.nasa.gov/blogs/missions/2022/11/27/artemis-i-flight-day-12-orion-star-trackers-reaction-control-thrusters-tested/),
[NASA pulse/impulse characterization](https://ntrs.nasa.gov/citations/19870004800).
The cited timing examples are not hardware specifications for the assumed jets.
