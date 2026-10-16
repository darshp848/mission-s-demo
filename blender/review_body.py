"""Bounded Blender studio review, independent of flight/animation state."""
import importlib.util
import json
import sys
from pathlib import Path

import bpy
from mathutils import Vector

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location("mission_scene", ROOT / "blender/build_scene.py")
builder = importlib.util.module_from_spec(spec)
spec.loader.exec_module(builder)
args = sys.argv[sys.argv.index("--") + 1:] if "--" in sys.argv else []
scene_path = Path(args[0]).resolve() if args else ROOT / "out/revised/scene.json"
out = Path(args[1]).resolve() if len(args) > 1 else ROOT / "out/revised/body_review"
out.mkdir(parents=True, exist_ok=True)
scene = json.loads(scene_path.read_text())
bpy.ops.wm.read_factory_settings(use_empty=True)
sc = bpy.context.scene
sc.unit_settings.system = "METRIC"
coll = bpy.data.collections.new("MissionS")
sc.collection.children.link(coll)
car = dict(scene["car"])
car.setdefault("h_ref", car.get("h_cg", 0.55))
rig, plumes = builder.build_car(coll, car, scene)
rig.location.z = car["h_ref"]
for pl in plumes.values():
    pl.hide_render = True
builder.configure_render("preview", out)
sc.render.resolution_x, sc.render.resolution_y = 960, 540
if hasattr(sc, "eevee"):
    sc.eevee.taa_render_samples = 32
sc.world = bpy.data.worlds.new("Studio")
sc.world.use_nodes = True
sc.world.node_tree.nodes["Background"].inputs["Color"].default_value = (0.12, 0.14, 0.18, 1)
sc.world.node_tree.nodes["Background"].inputs["Strength"].default_value = 0.45
ground = builder.prim("plane", "StudioFloor", coll,
                      builder.material("StudioFloorMat", (0.075, 0.085, 0.10), rough=0.55), size=200)
for name, loc, energy, size in [("Key", (3, -4, 6), 1500, 5),
                                ("Fill", (0, 4, 4), 1100, 4),
                                ("Rim", (-4, -1, 4), 1300, 3)]:
    lamp = bpy.data.objects.new(name, bpy.data.lights.new(name, "AREA"))
    coll.objects.link(lamp)
    lamp.location = loc
    lamp.data.energy, lamp.data.shape, lamp.data.size = energy, "DISK", size
    lamp.rotation_euler = (Vector((0, 0, 0.5)) - lamp.location).to_track_quat("-Z", "Y").to_euler()
target = builder.empty("StudioTarget", coll, (0, 0, 0.55))
cam = builder.new_cam(coll, "StudioCamera", 52, target)
sc.camera = cam
sc.render.image_settings.file_format = "PNG"
bpy.context.view_layer.update()
invalid, bounds = [], {}
for obj in coll.objects:
    if obj.type == "MESH" and obj.name.startswith("MS_"):
        if obj.data.validate():
            invalid.append(obj.name)
        coords = [obj.matrix_world @ Vector(c) for c in obj.bound_box]
        bounds[obj.name] = {"min": [min(p[i] for p in coords) for i in range(3)],
                            "max": [max(p[i] for p in coords) for i in range(3)]}
layout = scene.get("thruster_layout") or json.loads((ROOT / "thruster_layout.json").read_text())
duct_obstructions = []
vehicle_obstructions = []
body = bpy.data.objects.get("MS_Body")
for outlet in layout["outlets"]:
    pos, force = Vector(outlet["position_m"]), Vector(outlet["force_direction"])
    ok, loc, normal, index = body.ray_cast(pos-force*0.004, -force, distance=1.0)
    if ok:
        duct_obstructions.append({"outlet": outlet["name"], "distance_m": (loc-pos).length})
    origin_world = rig.matrix_world @ (pos-force*0.004)
    direction_world = rig.matrix_world.to_3x3() @ -force
    for obj in coll.objects:
        if obj.type != "MESH" or not obj.name.startswith("MS_") or obj.name.startswith((
                "MS_Plume_", "MS_RecessedNozzle_", "MS_NozzleThroat_")):
            continue
        inv = obj.matrix_world.inverted()
        hit, loc, normal, index = obj.ray_cast(inv @ origin_world,
            (inv.to_3x3() @ direction_world).normalized(), distance=0.8)
        if hit:
            vehicle_obstructions.append({"outlet": outlet["name"], "object": obj.name,
                "distance_m": ((obj.matrix_world @ loc)-origin_world).length})
hardware = [v["min"][2] for k,v in bounds.items() if k.startswith("MS_RecessedNozzle_")]
(out / "geometry_review.json").write_text(json.dumps({"blender": bpy.app.version_string,
    "invalid_meshes": invalid, "object_bounds": bounds,
    "external_pods": [o.name for o in coll.objects if o.name.startswith("MS_Pod_")],
    "canonical_outlet_count": len(layout["outlets"]),
    "rendered_outlet_count": sum(o.name.startswith("MS_RecessedNozzle_") for o in coll.objects),
    "minimum_nozzle_ground_clearance_m": min(hardware),
    "duct_obstructions": duct_obstructions,
    "vehicle_exhaust_axis_obstructions": vehicle_obstructions,
    "scene_input": str(scene_path)}, indent=2))
views = {"front": (6, -7, 3.1), "rear": (-6, -7, 2.8), "side": (0.3, -8.5, 1.6),
         "underside": (3, -5, -3.5)}
if len(args) > 2:
    views = {key: views[key] for key in args[2].split(",")}
for name, loc in views.items():
    ground.hide_render = name == "underside"
    cam.location = loc
    sc.render.filepath = str(out / (name + ".png"))
    bpy.ops.render.render(write_still=True)
ground.hide_render = False
bpy.ops.wm.save_as_mainfile(filepath=str(out / "body_review.blend"))
print("BODY_REVIEW_OK", out)
