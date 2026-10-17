# Revised Simulink flight control

The authoritative model is `mission_s.slx`. See the project README for the full
layout, assumptions, run commands and measured results.

```matlab
build_mission_s_model % safe opener + zero-time compile, preserves .slx
ms_validate_plant
ms_validate_allocator
S=run_mission_s();
R=ms_robustness(50);
```

Outputs go to `../out/revised`, with selected reference results already included.
`ms_params` reads `../thruster_layout.json`; all compiled parameters are numeric.
P is the controller's nominal model; PP is the plant truth. SimulationInput
objects apply each run's P/PP without permanently changing the model workspace.

The plant receives scalar-first body-to-world quaternion q4 and right-handed
body rates3. World/body axes at rest are x forward, y left, z up. Positive
nose-up display pitch is opposite right-handed body-y rate. Forces/moments
use unit force directions and cross(position-CG, force). Body properties
include full inertia and fixed-point tank depletion Idot. Ground contact is
upright suspension/tyres; minimum box-corner heights detect ground penetration.

Blocks: Vehicle Dynamics integrates position, velocity, quaternion, body rate,
propellant and wheel angle; Propulsion applies feed, cut-off, lag and nonnegative
thrust; Flight Computer runs 200 Hz with one-frame command latency. Physical
plant integration uses fixed-step ode4 at 1 ms.

The 22-element controller memory is:
1 mode; 2 mode time; 3 roll end; 4 pitch reference; 5:7 attitude integrals;
8:10 lag acceleration observer; 11 stop time; 12 touchdown mode;
13 roll profile time; 14 estimated total main thrust; 15 roll progress;
16 pitch progress; 17:20 previous quaternion (reserved); 21:22 reserved.

`ms_log_columns` defines the telemetry indices. `ms_log_matrix` normalizes
Simulink's widthÃ—1Ã—samples Array representation to samplesÃ—62. Aux14 preserves
legacy indices1:13, then appends Izz. Exported CSV has quaternion columns,
display Euler angles, all28 per-channel throttle percentages and physical
telemetry. `ms_summary` checks completion from actual rotations and terminal
orientation, not just sequencer mode entry.

`ms_validate_allocator` compares40 targets to Optimization Toolbox lsqlin.
`ms_validate_plant` checks torque handedness, wrench rank, equilibrium,
quaternion integration and inertia derivative. `ms_robustness` preserves seed7,
plant draws and explicit failure messages; local loop margins are separately
exported from full nonlinear outcomes.

## Current aerodynamic and pulsed-RCS version

Current output is ../out/aero_pulsed; ../out/revised holds the baseline. Small jets
are binary valves after impulse-error scheduling, with assumed 20 ms minimum on
and 10 ms minimum off. Main engines retain throttle control. Run
ms_validate_aero_pulses alongside the existing checks. ms_aero models assumed
all-attitude surface loads and ms_wind a spatial gust field.
Memory is now 100 entries: the original 22, then 26 impulse credits, 26 valve
states and 26 dwell timers. Telemetry is now 90 columns: the original 62 plus
28 delayed commands. Environment/actuator assumptions are exported in scene.json.
See ../out/aero_pulsed/RESULTS.md for actual outcomes and numerical sensitivity.
