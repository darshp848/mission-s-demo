"""Resume the saved canonical PNG animation with a selected laptop GPU.

blender -b out/revised/mission_s.blend -t 8 -P blender/render_saved.py
Cycles device preferences are process-local; reselect them after opening a blend.
"""
from pathlib import Path

import bpy

root = Path(__file__).resolve().parents[1]
sc = bpy.context.scene
prefs = bpy.context.preferences.addons["cycles"].preferences
prefs.compute_device_type = "OPTIX"
prefs.get_devices()
devices = [d for d in prefs.devices if d.type == "OPTIX"]
if not devices:
    raise RuntimeError("Expected OPTIX laptop GPU was not available")
for device in prefs.devices:
    device.use = device.type == "OPTIX"
sc.cycles.device = "GPU"
sc.render.image_settings.file_format = "PNG"
sc.render.filepath = str(Path(bpy.data.filepath).resolve().parent / "frames/frame_")
sc.render.use_overwrite = False
print("RENDER_GPU", [(d.name, d.type) for d in devices], flush=True)
print("FRAME_RANGE", sc.frame_start, sc.frame_end, flush=True)
bpy.ops.render.render(animation=True)
print("FULL_RENDER_COMPLETE", sc.frame_start, sc.frame_end, flush=True)
