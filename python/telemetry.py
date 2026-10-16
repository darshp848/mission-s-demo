#!/usr/bin/env python3
"""Live telemetry overlay for the flying Mission S.

Reads out/trajectory.csv (one row per video frame) and draws a 1920x1080 frame per row:
the Blender render top-left, live layout-derived thruster throttle bars on the right,
and G-load, propellant, pressure and centre-of-mass traces along the bottom.

  python3 telemetry.py [--csv out/trajectory.csv] [--render_dir out/frames]
                       [--out out/telemetry] [--mp4 out/mission_s_telemetry.mp4]

--render_dir holds the PNG sequence from `build_scene.py -- --frames_dir DIR`
(frame_0001.png ...). Without it the video area is left as a dark placeholder, which is
handy as a standalone telemetry check. --mp4 needs ffmpeg on PATH.
Needs numpy and matplotlib. matlab/telemetry_video.m is the MATLAB equivalent.
"""
import argparse
import csv
import json
import os
import shutil
import subprocess

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

MODES = {"hold": "HOLD", "launch": "LAUNCH (V8 + e-motors)", "boost": "SUPERDRACO BOOST",
         "coast": "COAST", "roll": "BARREL ROLL", "roll_settle": "ROLL SETTLE",
         "pitch_kick": "BACKFLIP KICK", "pitch_coast": "BACKFLIP", "pitch_catch": "BACKFLIP CATCH",
         "descent": "DESCENT", "landing_burn": "LANDING BURN", "settle": "SETTLE", "rollout": "ROLLOUT"}
BG, PANEL, FG, DIM = "#0b0d10", "#14181d", "#e8eaed", "#8a929c"
C = {"x": "#4fc3f7", "y": "#ffb74d", "z": "#81c784", "tot": "#ffffff", "fuel": "#ffb74d",
     "ox": "#4fc3f7", "copv": "#ce93d8", "tank": "#81c784", "sd": "#ff8a50", "dr": "#9fb8ff"}


def load(path):
    with open(path, newline="") as f:
        rows = list(csv.DictReader(f))
    if not rows:
        raise ValueError("Telemetry CSV contains no data rows")
    d = {k: np.array([float(r[k]) for r in rows]) for k in rows[0] if k != "phase"}
    d["phase"] = [r["phase"] for r in rows]
    return d


def telemetry_groups(data, scene_path, layout_path):
    """Use the exported scene layout; infer columns only for legacy CSVs."""
    layout = None
    if scene_path and os.path.isfile(scene_path):
        with open(scene_path, encoding="utf-8") as f:
            layout = json.load(f).get("thruster_layout")
    strict = layout is not None
    if layout is None and os.path.isfile(layout_path):
        with open(layout_path, encoding="utf-8") as f:
            candidate = json.load(f)
        if all("thr_" + c["name"] in data for c in candidate["channels"]):
            layout = candidate
    channels = layout["channels"] if layout else [
        {"name": k[4:], "kind": "superdraco" if k.startswith("thr_SD") else "legacy"}
        for k in data if k.startswith("thr_")]
    missing = [c["name"] for c in channels if "thr_" + c["name"] not in data]
    if missing or not channels:
        raise ValueError("Scene/CSV thruster channels disagree: " + ", ".join(missing))
    groups = []
    for c in channels:
        name = c["name"]
        if c.get("kind") == "side" and name.startswith(("S_L", "S_R")):
            side = name[:3]
            if any(g["key"] == side for g in groups):
                continue
            members = [v["name"] for v in channels
                       if v.get("kind") == "side" and v["name"].startswith(side)]
            label = "Sill " + ("left" if side == "S_L" else "right") + " (mean)"
        else:
            side, members, label = name, [name], name.replace("_", " ")
        groups.append({"key": side, "label": label, "members": members,
                       "kind": c.get("kind", ""),
                       "values": np.mean([data["thr_" + v] for v in members], axis=0)})
    notes = ["Delivered thrust, after actuator lag"]
    for kind in dict.fromkeys(c.get("kind", "") for c in channels):
        thrusts = sorted({c["max_thrust_N"] for c in channels
                          if c.get("kind", "") == kind and "max_thrust_N" in c})
        if thrusts:
            notes.append(f"{kind}: " + "/".join(f"{v / 1000:g}" for v in thrusts) + " kN/channel")
    aggregated = [g for g in groups if len(g["members"]) > 1]
    if aggregated:
        counts = "/".join(str(len(g["members"])) for g in aggregated)
        notes.append(f"Sill bars: mean channel % ({counts} per side);\n"
                     f"all {len(channels)} channels retained in CSV")
    if not layout:
        notes.append("Legacy CSV channels; maximum forces unspecified")
    return groups, "\n".join(notes), strict


def style(ax, title, ylabel=None):
    ax.set_facecolor(PANEL)
    for s in ax.spines.values():
        s.set_color("#2a3038")
    ax.tick_params(colors=DIM, labelsize=8)
    ax.set_title(title, color=FG, fontsize=10, loc="left", pad=4)
    if ylabel:
        ax.set_ylabel(ylabel, color=DIM, fontsize=8)
    ax.grid(True, color="#222831", lw=0.6)


def validate_render_frames(render_dir, first, last):
    """Check only the requested inclusive range, including non-first subsets."""
    if not render_dir:
        return
    missing = [f"frame_{i:04d}.png" for i in range(first, last + 1)
               if not os.path.isfile(os.path.join(render_dir, f"frame_{i:04d}.png"))]
    if missing:
        raise FileNotFoundError(
            f"Missing rendered frames in {render_dir} for range {first}:{last}: "
            + ", ".join(missing[:8]) + (f" (+{len(missing)-8} more)" if len(missing) > 8 else ""))


def main():
    here = os.path.dirname(os.path.abspath(__file__))
    root = os.path.normpath(os.path.join(here, ".."))
    ap = argparse.ArgumentParser()
    ap.add_argument("--csv", default=os.path.join(root, "out", "trajectory.csv"))
    ap.add_argument("--render_dir", default=None)
    ap.add_argument("--out", default=os.path.join(root, "out", "telemetry"))
    ap.add_argument("--mp4", default=None)
    ap.add_argument("--fps", type=int, default=24)
    ap.add_argument("--frames", default=None, help="A:B (1-based, inclusive)")
    ap.add_argument("--scene", default=None, help="scene.json (default: beside CSV)")
    ap.add_argument("--layout", default=os.path.join(root, "thruster_layout.json"))
    a = ap.parse_args()

    ff = shutil.which("ffmpeg") if a.mp4 else None
    if a.mp4 and not ff:
        raise RuntimeError("--mp4 requires ffmpeg on PATH; no video was written")
    if a.fps <= 0:
        ap.error("--fps must be positive")

    d = load(a.csv)
    groups, layout_note, _ = telemetry_groups(
        d, a.scene or os.path.join(os.path.dirname(a.csv), "scene.json"), a.layout)
    t = d["t"]
    n = len(t)
    lo_f, hi_f = 1, n
    if a.frames:
        lo_f, hi_f = (int(v) for v in a.frames.split(":"))
    if not 1 <= lo_f <= hi_f <= n:
        raise ValueError(f"Frame range must be inside 1:{n}")
    validate_render_frames(a.render_dir, lo_f, hi_f)
    os.makedirs(a.out, exist_ok=True)
    cg0 = np.array([d["cg_x"][0], d["cg_y"][0], d["cg_z"][0]])

    fig = plt.figure(figsize=(19.2, 10.8), dpi=100, facecolor=BG)
    # video area, 1344x756 top-left
    axv = fig.add_axes([0, 0.30, 0.70, 0.70])
    axv.set_axis_off()
    vid = axv.imshow(np.zeros((756, 1344, 3)), aspect="auto", extent=(0, 1, 0, 1))
    axv.set_xlim(0, 1)
    axv.set_ylim(0, 1)
    ph_txt = None
    if not a.render_dir:
        ph_txt = axv.text(0.5, 0.5, "3D render goes here\n(--render_dir)", color="#333a44", ha="center",
                          va="center", fontsize=22)
    hud = axv.text(0.015, 0.965, "", color=FG, fontsize=15, family="monospace", va="top",
                   bbox=dict(facecolor="#000000", alpha=0.45, edgecolor="none", pad=6))
    mode = axv.text(0.985, 0.965, "", color="#ffcc80", fontsize=17, weight="bold", ha="right", va="top",
                    bbox=dict(facecolor="#000000", alpha=0.45, edgecolor="none", pad=6))

    # thruster throttle bars
    axb = fig.add_axes([0.815, 0.39, 0.17, 0.44])
    style(axb, "Delivered thrust (% of channel max)")
    labels = [g["label"] for g in groups]
    ypos = np.arange(len(groups))[::-1]
    cols = [C["sd"] if g["kind"] == "superdraco" else C["dr"] for g in groups]
    bars = axb.barh(ypos, np.zeros(len(groups)), color=cols, height=0.62)
    axb.set_yticks(ypos)
    axb.set_yticklabels(labels, color=FG, fontsize=8, family="monospace")
    axb.set_xlim(0, 118)
    axb.set_xticks([0, 20, 40, 60, 80, 100])
    axb.set_axisbelow(True)
    vals = [axb.text(2, y, "", color=FG, fontsize=8, va="center") for y in ypos]
    fig.text(0.71, 0.35, layout_note, color=DIM, fontsize=7, va="top")

    # readout panel
    axr = fig.add_axes([0.71, 0.86, 0.28, 0.13])
    axr.set_axis_off()
    readout = axr.text(0.0, 1.0, "", color=FG, fontsize=11.5, family="monospace", va="top")

    # bottom traces
    W, gap, x0 = 0.215, 0.033, 0.035
    axg = fig.add_axes([x0, 0.05, W, 0.20])
    axp = fig.add_axes([x0 + (W + gap), 0.05, W, 0.20])
    axq = fig.add_axes([x0 + 2 * (W + gap), 0.05, W, 0.20])
    axc = fig.add_axes([x0 + 3 * (W + gap), 0.05, W, 0.20])
    style(axg, "Accelerometer at CG (g, body axes)")
    style(axp, "Propellant remaining (%, MMH & NTO drain at O/F 1.65)")
    style(axq, "Helium COPVs & tank pressure (bar)")
    style(axc, "CG shift from launch (mm, body axes)")
    traces = []

    def trace(ax, y, color, label, lw=1.4):
        ln, = ax.plot([], [], color=color, lw=lw, label=label)
        traces.append((ln, y))
        return ln

    trace(axg, d["g_x"], C["x"], "x fwd")
    trace(axg, d["g_y"], C["y"], "y left")
    trace(axg, d["g_z"], C["z"], "z up")
    trace(axg, d["g_total"], C["tot"], "|g|", 1.0)
    lo = min(d["g_x"].min(), d["g_y"].min(), d["g_z"].min())
    hi = max(d["g_total"].max(), d["g_z"].max())
    axg.set_ylim(lo - 0.3, hi + 0.3)
    trace(axp, d["fuel_pct"], C["fuel"], "MMH")
    trace(axp, d["ox_pct"], C["ox"], "NTO", 1.0).set_linestyle("--")
    axp.set_ylim(0, 105)
    trace(axq, d["copv_bar"], C["copv"], "COPV")
    axq2 = axq.twinx()
    axq2.tick_params(colors=C["tank"], labelsize=8)
    ln_tank, = axq2.plot([], [], color=C["tank"], lw=1.4, label="tank (reg.)")
    traces.append((ln_tank, d["tank_bar"]))
    axq.set_ylim(min(d["copv_bar"].min(), 250) - 10, d["copv_bar"].max() + 10)
    axq2.set_ylim(0, max(d["tank_bar"].max() * 1.3, 30))
    dcg = [(d[k] - c) * 1000 for k, c in zip(("cg_x", "cg_y", "cg_z"), cg0)]
    trace(axc, dcg[0], C["x"], "x")
    trace(axc, dcg[1], C["y"], "y")
    trace(axc, dcg[2], C["z"], "z")
    span = max(5.0, max(np.abs(v).max() for v in dcg) * 1.15)
    axc.set_ylim(-span, span)
    cursors = []
    for ax in (axg, axp, axq, axc):
        ax.set_xlim(t[0], t[-1])
        ax.set_xlabel("t (s)", color=DIM, fontsize=8)
        cursors.append(ax.axvline(t[0], color="#ffcc80", lw=0.8, alpha=0.8))
        h, l = ax.get_legend_handles_labels()
        if ax is axq:
            h2, l2 = axq2.get_legend_handles_labels()
            h, l = h + h2, l + l2
        ax.legend(h, l, loc="lower left", fontsize=7, ncol=4, facecolor=PANEL, edgecolor="none",
                  labelcolor=FG)
    fig.text(0.035, 0.275, "Porsche Mission S  |  SuperDraco / Draco thruster flight  |  simulated",
             color=DIM, fontsize=9)

    for f in range(lo_f, hi_f + 1):
        i = f - 1
        if a.render_dir:
            p = os.path.join(a.render_dir, f"frame_{f:04d}.png")
            vid.set_data(plt.imread(p)[..., :3])
        for ln, y in traces:
            ln.set_data(t[:i + 1], y[:i + 1])
        for c in cursors:
            c.set_xdata([t[i], t[i]])
        for k, group in enumerate(groups):
            v = group["values"][i]
            bars[k].set_width(v)
            vals[k].set_text(f"{v:.0f}%" if v >= 0.5 else "")
            vals[k].set_x(v + 1.5)
        ph = d["phase"][i]
        mode.set_text(MODES.get(ph, ph.upper()))
        hud.set_text(f"T+{t[i]:5.2f} s   {d['speed'][i] * 3.6:5.1f} km/h   CG {d['z'][i]:5.2f} m\n"
                     f"roll {np.degrees(d['roll'][i]):6.1f}°   pitch {np.degrees(d['pitch'][i]):6.1f}°   "
                     f"{d['g_total'][i]:4.2f} g")
        gear = int(d["gear"][i])
        readout.set_text(f"mass     {d['mass_kg'][i]:7.1f} kg\n"
                         f"engine   {d['rpm'][i]:7.0f} rpm  gear {gear if gear else '-'}\n"
                         f"MMH/NTO  {d['fuel_pct'][i]:5.1f} / {d['ox_pct'][i]:5.1f} %\n"
                         f"COPV     {d['copv_bar'][i]:6.1f} bar   tank {d['tank_bar'][i]:4.1f} bar")
        fig.savefig(os.path.join(a.out, f"tele_{f:04d}.png"), dpi=100, facecolor=BG)
        if f % 50 == 0:
            print(f"frame {f}/{n}")
    plt.close(fig)
    print("wrote", a.out)
    if a.mp4:
        subprocess.run([ff, "-y", "-loglevel", "error", "-framerate", str(a.fps), "-start_number", str(lo_f),
                        "-i", os.path.join(a.out, "tele_%04d.png"), "-frames:v", str(hi_f-lo_f+1),
                        "-c:v", "libx264", "-pix_fmt", "yuv420p",
                        "-crf", "18", a.mp4], check=True)
        print("wrote", a.mp4)


if __name__ == "__main__":
    main()
