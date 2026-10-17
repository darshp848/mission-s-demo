"""Build and animate the Mission S concept in Blender 5.2 LTS.

Reads ../out/trajectory.csv and ../out/scene.json (from matlab/mission_s_sim.m or
python/mission_s_sim.py) and builds:
  * a Mission S body lofted from profiles digitised off Porsche's official photos
    (side, top, front, rear; see the SIDE_*/TOP_* tables), with wheel arches, black
    canopy, GT1-style fender headlights, light bar, ducktail, diffuser,
    engine-cover fan, camera mirrors and five-hole wheels with red calipers,
  * canonical recessed propulsion hardware from thruster_layout.json, embedded
    in scene.json: twin short outlets per SD chamber, integrated corner ports,
    a single eight-aperture row per sill and rear pair, with channel-driven plumes,
  * five lineup stand-in vehicles, start and liftoff lines, ground and sky,
  * four cameras cut together with markers, and quaternion pose keyframes.

Headless:
  blender -b -P build_scene.py -- [--render preview|final|none] [--frames A:B]
        [--still FRAME] [--out DIR] [--save FILE.blend] [--engine cycles|eevee]
        [--samples N] [--res PERCENT] [--frames_dir DIR]
  --frames_dir renders a PNG sequence (used by python/telemetry.py --render_dir DIR) instead of an MP4.
In the GUI: Scripting workspace, open this file, Run Script.
"""
import csv
import json
import math
import sys
from pathlib import Path

import bpy
from mathutils import Quaternion, Vector

PROJECT_DIR = None                 # optional project-root override
PAINT = (0.80, 0.80, 0.80)         # Mission S show-car white (linear RGB)

# ----------------------------------------------------------------------------- profiles
# Digitised from Porsche's official Mission S press photos (768 px wide):
#   side view (ms_0001): x_img, y_img; ground at y=383, scale 0.0075 m/px,
#   car centre x=375.5 (nose at 68, tail at 683).
SIDE_TOP = [(68, 330), (75, 320), (90, 309), (110, 300), (130, 294), (155, 289), (180, 288),
            (205, 288), (230, 285), (255, 282), (272, 280), (290, 264), (310, 250), (330, 240),
            (350, 233), (375, 229), (400, 228), (430, 229), (460, 232), (490, 237), (520, 244),
            (550, 252), (580, 261), (600, 267), (620, 272), (640, 275), (660, 279), (672, 290),
            (680, 300), (683, 320)]
SIDE_BELT = [(272, 280), (320, 283), (380, 281), (440, 275), (480, 268), (520, 262), (560, 262),
             (590, 265), (600, 267)]          # body shoulder under the greenhouse
SIDE_BOT = [(68, 356), (90, 361), (130, 362), (260, 363), (500, 363), (600, 355), (640, 348),
            (683, 338)]
SIDE_AXLES = (208, 565)                       # wheel centres, x_img
#   top view (ms_0003): nose x=102, tail x=702, centreline y=278; half-widths in px,
#   rescaled so the widest point (rear hips) is half the body width.
TOP_HW = [(102, 50), (106, 62), (112, 76), (120, 88), (140, 100), (170, 108), (200, 108),
          (250, 106), (300, 96), (350, 89), (400, 88), (430, 92), (460, 103), (500, 113),
          (530, 116), (560, 114), (600, 103), (640, 88), (670, 63), (690, 42), (702, 30)]
TOP_GLASS_HW = [(255, 40), (270, 62), (300, 80), (350, 86), (400, 86), (450, 84), (500, 78),
                (550, 66), (590, 50), (602, 36)]
S_SIDE, X_MID_SIDE, Y_GROUND = 0.0075, 375.5, 383
L_TOP, X_NOSE_TOP, X_TAIL_TOP, HW_MAX_PX = 4.61, 102, 702, 116


def side_x(xi):
    return (X_MID_SIDE - xi) * S_SIDE                      # +x is the nose


def side_z(yi):
    return (Y_GROUND - yi) * S_SIDE                        # above ground


def top_x(xi):
    return L_TOP / 2 - (xi - X_NOSE_TOP) * L_TOP / (X_TAIL_TOP - X_NOSE_TOP)


def table(pts, fx, fy):
    out = sorted((fx(a), fy(b)) for a, b in pts)
    return [p[0] for p in out], [p[1] for p in out]


def lerp_table(xs, ys, x):
    if x <= xs[0]:
        return ys[0]
    for i in range(1, len(xs)):
        if x <= xs[i]:
            f = (x - xs[i - 1]) / (xs[i] - xs[i - 1])
            return ys[i - 1] + f * (ys[i] - ys[i - 1])
    return ys[-1]


# ----------------------------------------------------------------------------- utils
def project_dir():
    if PROJECT_DIR:
        return Path(PROJECT_DIR)
    try:
        return Path(__file__).resolve().parent.parent
    except NameError:
        return Path(bpy.path.abspath("//")).parent


def parse_args():
    argv = sys.argv[sys.argv.index("--") + 1:] if "--" in sys.argv else []
    a = {"render": "none", "frames": None, "still": None, "out": None, "save": None,
         "csv": None, "scene": None, "engine": None, "samples": None, "res": None,
         "frames_dir": None}
    i = 0
    while i < len(argv):
        k = argv[i].lstrip("-")
        if k in a and i + 1 < len(argv):
            a[k] = argv[i + 1]
            i += 2
        else:
            i += 1
    return a


def set_input(node, names, value):
    for n in names if isinstance(names, (list, tuple)) else [names]:
        if n in node.inputs:
            node.inputs[n].default_value = value
            return


def material(name, color, metallic=0.0, rough=0.4, coat=0.0, emit=0.0, alpha=1.0, transmission=0.0):
    if name in bpy.data.materials:
        return bpy.data.materials[name]
    m = bpy.data.materials.new(name)
    m.use_nodes = True
    b = m.node_tree.nodes.get("Principled BSDF")
    set_input(b, "Base Color", (*color, 1))
    set_input(b, "Metallic", metallic)
    set_input(b, "Roughness", rough)
    set_input(b, ["Coat Weight", "Clearcoat"], coat)
    set_input(b, ["Coat Roughness", "Clearcoat Roughness"], 0.03)
    set_input(b, ["Transmission Weight", "Transmission"], transmission)
    if emit:
        set_input(b, ["Emission Color", "Emission"], (*color, 1))
        set_input(b, "Emission Strength", emit)
    if alpha < 1:
        set_input(b, "Alpha", alpha)
        if hasattr(m, "surface_render_method"):
            m.surface_render_method = "BLENDED"
        elif hasattr(m, "blend_method"):
            m.blend_method = "BLEND"
    m.diffuse_color = (*color, alpha)
    return m


def plume_material(name, color, strength):
    """Emissive exhaust that fades toward its edges and its far end (no volumes, renders fast)."""
    if name in bpy.data.materials:
        return bpy.data.materials[name]
    m = bpy.data.materials.new(name)
    m.use_nodes = True
    nt = m.node_tree
    nt.nodes.clear()
    out = nt.nodes.new("ShaderNodeOutputMaterial")
    em = nt.nodes.new("ShaderNodeEmission")
    em.inputs["Color"].default_value = (*color, 1)
    em.inputs["Strength"].default_value = strength
    tr = nt.nodes.new("ShaderNodeBsdfTransparent")
    mix = nt.nodes.new("ShaderNodeMixShader")
    lw = nt.nodes.new("ShaderNodeLayerWeight")
    lw.inputs["Blend"].default_value = 0.35
    tc = nt.nodes.new("ShaderNodeTexCoord")
    sep = nt.nodes.new("ShaderNodeSeparateXYZ")
    inv = nt.nodes.new("ShaderNodeMath")
    inv.operation = "SUBTRACT"
    inv.inputs[0].default_value = 1.0
    mul = nt.nodes.new("ShaderNodeMath")
    mul.operation = "MULTIPLY"
    pw = nt.nodes.new("ShaderNodeMath")
    pw.operation = "POWER"
    pw.inputs[1].default_value = 1.6
    opa = nt.nodes.new("ShaderNodeMath")
    opa.operation = "MULTIPLY"
    opa.inputs[1].default_value = 0.85
    l = nt.links.new
    l(tc.outputs["Generated"], sep.inputs[0])
    l(lw.outputs["Facing"], inv.inputs[1])                  # 1 at the core, 0 at the rim
    l(sep.outputs["Z"], pw.inputs[0])                       # 1 at the nozzle, 0 at the tip
    l(inv.outputs[0], mul.inputs[0])
    l(pw.outputs[0], mul.inputs[1])
    l(mul.outputs[0], opa.inputs[0])
    l(opa.outputs[0], mix.inputs["Fac"])
    l(tr.outputs[0], mix.inputs[1])
    l(em.outputs[0], mix.inputs[2])
    l(mix.outputs[0], out.inputs["Surface"])
    if hasattr(m, "surface_render_method"):
        m.surface_render_method = "BLENDED"
    elif hasattr(m, "blend_method"):
        m.blend_method = "BLEND"
    m.diffuse_color = (*color, 0.3)
    return m


def link(obj, coll):
    for c in obj.users_collection:
        c.objects.unlink(obj)
    coll.objects.link(obj)
    return obj


def mesh_obj(name, verts, faces, coll, mats=(), smooth=True, subsurf=0):
    me = bpy.data.meshes.new(name)
    me.from_pydata(verts, [], faces)
    me.update()
    ob = bpy.data.objects.new(name, me)
    coll.objects.link(ob)
    for m in mats:
        me.materials.append(m)
    if smooth:
        for p in me.polygons:
            p.use_smooth = True
    if subsurf:
        mod = ob.modifiers.new("Subsurf", "SUBSURF")
        mod.levels = mod.render_levels = subsurf
    return ob


def prim(kind, name, coll, mat=None, **kw):
    getattr(bpy.ops.mesh, f"primitive_{kind}_add")(**kw)
    ob = bpy.context.active_object
    ob.name = name
    link(ob, coll)
    if mat:
        ob.data.materials.append(mat)
    return ob


def beveled_box(name, size, loc, coll, mat, bevel=0.06, rot=(0, 0, 0)):
    ob = prim("cube", name, coll, mat, size=1, location=loc, rotation=rot)
    ob.scale = size
    bpy.ops.object.transform_apply(location=False, rotation=False, scale=True)
    if bevel:
        m = ob.modifiers.new("Bevel", "BEVEL")
        m.width = bevel
        m.segments = 3
    for p in ob.data.polygons:
        p.use_smooth = True
    return ob


def empty(name, coll, loc=(0, 0, 0), parent=None):
    e = bpy.data.objects.new(name, None)
    e.empty_display_size = 0.3
    e.location = loc
    coll.objects.link(e)
    if parent:
        e.parent = parent
    return e


def trim_curve(name, points, coll, mat, radius=0.004, parent=None, closed=False):
    cu = bpy.data.curves.new(name, "CURVE")
    cu.dimensions = "3D"
    cu.bevel_depth, cu.bevel_resolution = radius, 3
    spline = cu.splines.new("POLY")
    spline.points.add(len(points) - 1)
    for p, co in zip(spline.points, points):
        p.co = (*co, 1)
    spline.use_cyclic_u = closed
    ob = bpy.data.objects.new(name, cu)
    coll.objects.link(ob)
    cu.materials.append(mat)
    ob.parent = parent
    return ob


def apply_modifiers(ob):
    bpy.ops.object.select_all(action="DESELECT")
    bpy.context.view_layer.objects.active = ob
    ob.select_set(True)
    for m in list(ob.modifiers):
        bpy.ops.object.modifier_apply(modifier=m.name)


def boolean_cut(ob, cutter):
    mod = ob.modifiers.new("Cut", "BOOLEAN")
    mod.operation = "DIFFERENCE"
    mod.object = cutter
    if hasattr(mod, "solver"):
        mod.solver = "EXACT"
    apply_modifiers(ob)
    bpy.data.objects.remove(cutter, do_unlink=True)


# ----------------------------------------------------------------------------- body
def section_points(hw, zb, zs, zc, bump=0.0):
    """Half cross-section from bottom centre round to top centre (y >= 0).
    zs is the fender/shoulder crown (outboard), zc the centreline; bump raises the crown over a wheel."""
    h = zs - zb
    return [(0.0, zb), (0.88 * hw, zb), (0.98 * hw, zb + 0.08 * h), (hw, zb + 0.33 * h),
            (0.985 * hw, zb + 0.64 * h + 0.5 * bump), (0.93 * hw, zs - 0.03 + bump), (0.80 * hw, zs + bump),
            (0.60 * hw, zs - 0.35 * (zs - zc)), (0.30 * hw, zc + 0.05 * (zs - zc)), (0.0, zc)]


def loft_sections(name, coll, mats, stations, sec_fn, subsurf=2):
    """stations: x list; sec_fn(x) -> half-section points. Mirrored, capped, subdivided."""
    verts, faces = [], []
    ring = None
    for x in stations:
        half = sec_fn(x)
        full = half + [(-y, z) for y, z in reversed(half[1:-1])]
        if ring is None:
            ring = len(full)
        for y, z in full:
            verts.append((x, y, z))
    n = len(stations)
    for i in range(n - 1):
        for j in range(ring):
            a, b = i * ring + j, i * ring + (j + 1) % ring
            faces.append((a, b, b + ring, a + ring))
    faces.append(tuple(reversed(range(ring))))
    faces.append(tuple((n - 1) * ring + j for j in range(ring)))
    return mesh_obj(name, verts, faces, coll, mats, subsurf=subsurf)


def build_car(coll, car, scene):
    """Mission S centred on the body reference point (base-car CG). +x is the nose."""
    h_ref, wr = car["h_ref"], car["wheel_r"]
    paint = material("MS_Paint", PAINT, metallic=0.15, rough=0.18, coat=1.0)
    black = material("MS_GlossBlack", (0.01, 0.01, 0.012), metallic=0.2, rough=0.08, coat=1.0)
    satin = material("MS_SatinBlack", (0.02, 0.02, 0.022), metallic=0.1, rough=0.45)
    glass = material("MS_Glass", (0.005, 0.006, 0.008), rough=0.14, coat=0.35)
    chrome = material("MS_Chrome", (0.9, 0.9, 0.92), metallic=1.0, rough=0.08)
    tire = material("MS_Tire", (0.012, 0.012, 0.012), rough=0.85)
    rimw = material("MS_RimWhite", (0.85, 0.85, 0.85), metallic=0.3, rough=0.25, coat=0.6)
    head = material("MS_Headlight", (1.0, 0.97, 0.92), emit=12.0)
    drl = material("MS_DRL", (1.0, 1.0, 1.0), emit=6.0)
    tail = material("MS_Taillight", (1.0, 0.02, 0.01), emit=30.0)
    caliper = material("MS_Caliper", (0.75, 0.02, 0.02), rough=0.3, coat=0.5)

    rig = empty("MissionS_Rig", coll)
    zr = -h_ref                                            # ground, relative to the rig

    tx, tt = table(SIDE_TOP, side_x, side_z)
    bx, bb = table(SIDE_BOT, side_x, side_z)
    ex, ee = table(SIDE_BELT, side_x, side_z)
    hx, hh = table(TOP_HW, top_x, lambda v: v / HW_MAX_PX * car["W"] / 2)
    gx, gg = table(TOP_GLASS_HW, top_x, lambda v: v / HW_MAX_PX * car["W"] / 2)
    x_ws, x_rg = max(ex), min(ex)                           # greenhouse extent
    xf_ax, xr_ax = side_x(SIDE_AXLES[0]), side_x(SIDE_AXLES[1])

    def shoulder(x):
        if x_rg < x < x_ws:
            return lerp_table(ex, ee, x)
        return lerp_table(tx, tt, x)

    def centre_dip(x):
        # hood valley between the GT1-style fenders, shallow valley on the engine deck
        return 0.07 * math.exp(-((x - 1.35) / 0.55) ** 2) + 0.04 * math.exp(-((x + 1.55) / 0.5) ** 2)

    def body_sec(x):
        hw = lerp_table(hx, hh, x)
        zs = shoulder(x)
        bump = 0.05 * math.exp(-((x - xf_ax) / 0.45) ** 2) + 0.04 * math.exp(-((x - xr_ax) / 0.5) ** 2)
        return [(y, z + zr) for y, z in section_points(hw, lerp_table(bx, bb, x), zs, zs - centre_dip(x), bump)]

    L = car["L"]
    stations = [-L / 2 + 0.01 + i * (L - 0.02) / 89 for i in range(90)]
    body = loft_sections("MS_Body", coll, [paint, satin], stations, body_sec, subsurf=2)
    for poly in body.data.polygons:                         # satin black: underbody, rockers, intake, valance
        c = poly.center                                     # (on the cage, so subsurf keeps clean edges)
        if c.z < zr + 0.20 or (c.z < zr + 0.23 and xr_ax + 0.45 < c.x < xf_ax - 0.45) or \
           (c.x < -L / 2 + 0.40 and c.z < zr + 0.46 and abs(c.y) < 0.80):
            poly.material_index = 1
    apply_modifiers(body)
    for x in (xf_ax, xr_ax):                                # wheel arches, outboard of the tub only
        for s in (1, -1):
            cut = prim("cylinder", "cutter", coll, None, vertices=48, radius=wr + 0.03, depth=1.5,
                       location=(x, s * 1.30, zr + wr), rotation=(math.pi / 2, 0, 0))
            boolean_cut(body, cut)
    body.parent = rig

    def gh_sec(x):                                          # black glass canopy
        hw = lerp_table(gx, gg, x) * 0.98
        zb = shoulder(x) - 0.04
        zt = max(lerp_table(tx, tt, x), zb + 0.02)
        return [(y, z + zr) for y, z in [(0, zb), (0.9 * hw, zb), (hw, zb + 0.05), (0.86 * hw, zb + 0.40 * (zt - zb)),
                                         (0.62 * hw, zt - 0.03), (0.35 * hw, zt), (0, zt)]]

    gst = [x_rg + 0.02 + i * (x_ws - x_rg - 0.04) / 39 for i in range(40)]
    gh = loft_sections("MS_Canopy", coll, [glass], gst, gh_sec, subsurf=2)
    apply_modifiers(gh)
    gh.parent = rig

    def hit(origin, direction, ob=body):
        ok, loc, nrm, _ = ob.ray_cast(Vector(origin), Vector(direction).normalized(), distance=20.0)
        return (loc, nrm) if ok else (None, None)

    def orient(ob, normal):                                # local +z along the normal
        ob.rotation_euler = Vector((0, 0, 1)).rotation_difference(normal).to_euler()

    for s in (1, -1):                                       # round headlights recessed in the fender noses
        loc, nrm = hit((1.88, s * 0.68, 3.0), (0, 0, -1))
        if loc is not None:
            # smoked-glass bowl with a light ring, set into the fender nose (as in the press photos)
            ring = prim("cylinder", f"MS_HeadPod_{s}", coll, black, vertices=48, radius=0.14, depth=0.04)
            orient(ring, nrm)
            ring.location = loc - nrm * 0.014
            ring.scale = (1.28, 1, 1)
            ring.parent = rig
            lens = prim("uv_sphere", f"MS_HeadLens_{s}", coll, glass, segments=32, ring_count=16, radius=0.12)
            orient(lens, nrm)
            lens.scale = (1.25, 1, 0.22)
            lens.location = loc + nrm * 0.005
            lens.parent = rig
            led = prim("torus", f"MS_HeadRing_{s}", coll, drl, major_radius=0.125, minor_radius=0.003)
            orient(led, nrm)
            led.location = loc + nrm * 0.012
            led.scale.x = 1.25
            led.parent = rig
        top, _ = hit((side_x(330), s * 0.70, 3.0), (0, 0, -1))  # camera mirrors on short stalks
        if top is not None:
            stalk = beveled_box(f"MS_MirrorStalk_{s}", (0.05, 0.16, 0.02), top + Vector((0, s * 0.10, 0.04)),
                                coll, black, 0.008)
            stalk.parent = rig
            cam_ = prim("uv_sphere", f"MS_MirrorCam_{s}", coll, paint, radius=0.06,
                        location=top + Vector((0.0, s * 0.20, 0.07)))
            cam_.scale = (1.7, 0.7, 0.65)
            cam_.parent = rig

    nose, _ = hit((4.0, 0, zr + 0.14), (-1, 0, 0))
    if nose is not None:                                    # splitter just behind the nose tip, big lower intake
        q, _ = hit((4.0, 0, zr + 0.25), (-1, 0, 0))
        if q is not None:
            beveled_box("MS_Intake", (0.16, 1.25, 0.20), (q.x - 0.07, 0, zr + 0.25), coll, satin, 0.05).parent = rig
        beveled_box("MS_Splitter", (0.30, 1.55, 0.025), (nose.x - 0.10, 0, zr + 0.085), coll, satin, 0.008).parent = rig

    for s in (1, -1):                                       # hood ducts between the fenders
        x0 = 1.25
        top, nrm = hit((x0, s * 0.30, 3.0), (0, 0, -1))
        if top is not None:
            d = beveled_box(f"MS_HoodDuct_{s}", (0.38, 0.16, 0.03), top - Vector((0, 0, 0.008)), coll, satin, 0.012)
            orient(d, nrm)
            d.parent = rig

    # Surface-following details keep the low show-car silhouette legible in motion.
    for s in (-1, 1):
        for label, path in (("Door", [(0.72, 0.69), (0.68, 0.60), (0.62, 0.45),
                                      (0.48, 0.25), (0.40, 0.21), (-0.65, 0.21)]),
                            ("Shoulder", [(-1.9 + k * 3.5 / 60, 0.53 + 0.025 * k / 60)
                                          for k in range(61)])):
            pts = []
            for x, z in path:
                q, normal = hit((x, s * 2, zr + z), (0, -s, 0))
                if q is not None:
                    pts.append(q + normal * 0.003)
            if len(pts) > 1:
                trim_curve(f"MS_{label}Seam_{s}", pts, coll, satin, 0.002, rig)
        pts = []
        for k in range(24):
            x = -0.86 + k * 1.65 / 23
            q, normal = hit((x, s * 2, zr + 0.195), (0, -s, 0))
            if q is not None:
                pts.append(q + normal * 0.004)
        if len(pts) > 1:
            trim_curve(f"MS_SillBlade_{s}", pts, coll, satin, 0.018, rig)
        pts = []
        for k in range(15):
            y = s * (0.49 + k * 0.34 / 14)
            q, normal = hit((4, y, zr + 0.39), (-1, 0, 0))
            if q is not None:
                pts.append(q + normal * 0.006)
        if len(pts) > 1:
            trim_curve(f"MS_FrontDRL_{s}", pts, coll, drl, 0.006, rig)

    # A conformal lower fascia patch makes the deep intake visible rather than buried.
    verts, faces = [], []
    for iz in range(5):
        z = zr + 0.19 + iz * 0.035
        for iy in range(25):
            y = -0.59 + iy * 1.18 / 24
            q, normal = hit((4, y, z), (-1, 0, 0))
            verts.append(tuple(q + normal * 0.004) if q is not None else (2.20, y, z))
    for iz in range(4):
        for iy in range(24):
            a = iz * 25 + iy
            faces.append((a, a+1, a+26, a+25))
    fascia = mesh_obj("MS_FrontIntakeSkin", verts, faces, coll, [satin])
    fascia.parent = rig

    gtop, _ = hit((side_x(372), 0, 3.0), (0, 0, -1), gh)
    if gtop is not None:                                    # roof scoop on the canopy
        beveled_box("MS_RoofScoop", (0.32, 0.30, 0.05), gtop + Vector((0, 0, 0.015)), coll, paint, 0.02).parent = rig

    # tail: light bar and ducktail lip follow the actual tail surface
    zl, zd = zr + side_z(298), zr + side_z(285)
    pts_l, pts_d = [], []
    for k in range(25):
        yy = -0.86 + k * 1.72 / 24
        l, _ = hit((-4.0, yy, zl), (1, 0, 0))
        d, _ = hit((-4.0, yy, zd), (1, 0, 0))
        if l is not None:
            pts_l.append(l + Vector((-0.008, 0, 0)))
        if d is not None:
            pts_d.append(d)
    if len(pts_l) > 3:
        cu = bpy.data.curves.new("MS_LightBar", "CURVE")
        cu.dimensions = "3D"
        cu.bevel_depth, cu.bevel_resolution = 0.011, 2
        sp = cu.splines.new("POLY")
        sp.points.add(len(pts_l) - 1)
        for i_, q in enumerate(pts_l):
            sp.points[i_].co = (q.x, q.y, q.z, 1)
        lb = bpy.data.objects.new("MS_LightBar", cu)
        lb.data.materials.append(tail)
        coll.objects.link(lb)
        lb.parent = rig
    if len(pts_d) > 3:                                      # thin satin ducktail blade
        verts, faces = [], []
        for q in pts_d:
            verts += [(q.x + 0.06, q.y, q.z - 0.01), (q.x - 0.16, q.y, q.z + 0.035)]
        for i_ in range(len(pts_d) - 1):
            faces.append((2 * i_, 2 * i_ + 1, 2 * i_ + 3, 2 * i_ + 2))
        duck = mesh_obj("MS_Ducktail", verts, faces, coll, [satin])
        duck.modifiers.new("Thick", "SOLIDIFY").thickness = 0.02
        duck.parent = rig
        for q in (pts_d[1], pts_d[-2]):
            fin = beveled_box("MS_DuckFin", (0.20, 0.02, 0.09), q + Vector((-0.08, 0, -0.03)), coll, satin, 0.008)
            fin.parent = rig
    # The canonical rear jet pair replaces the reference car's quad exhausts.
    q, _ = hit((-4.0, 0, zr + 0.34), (1, 0, 0))
    if q is not None:
        beveled_box("MS_Plate", (0.02, 0.50, 0.12), q + Vector((-0.01, 0, 0)), coll, black, 0.005).parent = rig
    x_fan = top_x(565)
    fan_top, _ = hit((x_fan, 0, 3.0), (0, 0, -1), gh)
    fan_loc = (fan_top or Vector((x_fan, 0, zr + lerp_table(tx, tt, x_fan)))) + Vector((0, 0, 0.004))
    fan = prim("cylinder", "MS_Fan", coll, satin, vertices=40, radius=0.13, depth=0.02, location=fan_loc)
    fan.parent = rig
    for k in range(9):
        bl = beveled_box(f"MS_FanBlade_{k}", (0.11, 0.02, 0.006), fan_loc + Vector((0, 0, 0.012)), coll, chrome,
                         0.002, rot=(0.35, 0, k * 2 * math.pi / 9))
        bl.parent = rig

    for tag, x in (("F", xf_ax), ("R", xr_ax)):              # wheels: five oval windows, red calipers
        for side, y in (("L", 0.79 if tag == "F" else 0.81), ("R", -0.79 if tag == "F" else -0.81)):
            sgn = 1 if side == "L" else -1
            hub = empty(f"MS_Wheel_{tag}{side}", coll, (x, y, zr + wr), rig)
            w = 0.26 if tag == "F" else 0.32
            t = prim("cylinder", f"MS_Tire_{tag}{side}", coll, tire, vertices=64, radius=wr, depth=w,
                     rotation=(math.pi / 2, 0, 0))
            t.parent, t.location = hub, (0, 0, 0)
            bev = t.modifiers.new("Bevel", "BEVEL")
            bev.width, bev.segments = 0.06, 4
            face = prim("cylinder", f"MS_RimFace_{tag}{side}", coll, rimw, vertices=64, radius=wr * 0.80,
                        depth=0.02, rotation=(math.pi / 2, 0, 0))
            face.parent, face.location = hub, (0, sgn * (w / 2 + 0.005), 0)
            for k in range(5):
                # Rounded triangular windows copied from the reference five-hole wheel.
                a = k * 2 * math.pi / 5
                contour = []
                for j in range(48):
                    u = 2 * math.pi * j / 48
                    rr = 1.0 + 0.16 * math.cos(3 * u)
                    radial = 0.175 + 0.093 * rr * math.cos(u)
                    tangential = 0.082 * rr * math.sin(u)
                    contour.append((radial * math.cos(a) - tangential * math.sin(a),
                                    sgn * (w / 2 + 0.017),
                                    radial * math.sin(a) + tangential * math.cos(a)))
                hole = mesh_obj(f"MS_RimWindow_{tag}{side}{k}", contour,
                                [tuple(range(len(contour)))], coll, [black])
                hole.parent = hub
                trim_curve(f"MS_RimWindowTrim_{tag}{side}{k}", contour, coll, chrome, 0.003, hub, True)
            cap = prim("cylinder", f"MS_RimCap_{tag}{side}", coll, black, vertices=32, radius=0.05, depth=0.03,
                       rotation=(math.pi / 2, 0, 0))
            cap.parent, cap.location = hub, (0, sgn * (w / 2 + 0.012), 0)
            cal = beveled_box(f"MS_Caliper_{tag}{side}", (0.20, 0.07, 0.12), (0, 0, 0), coll, caliper, 0.02)
            cal.parent = rig
            cal.location = (x - 0.10, y - sgn * (w / 2 - 0.02), zr + wr + 0.13)

    return rig, build_propulsion(coll, rig, scene, body)


def build_propulsion(coll, rig, scene, body=None):
    """Canonical exit faces: structure extends inward, exhaust extends outward."""
    layout = scene.get("thruster_layout")
    if layout is None:
        layout = json.loads((project_dir() / "thruster_layout.json").read_text())
    nozzle = material("MS_Nozzle", (0.24, 0.21, 0.18), metallic=0.85, rough=0.32)
    dark = material("MS_ThrusterRecess", (0.008, 0.01, 0.012), metallic=0.3, rough=0.5)
    gas_sd = plume_material("MS_PlumeSD", (1.0, 0.60, 0.28), 20.0)
    gas_dr = plume_material("MS_PlumeDraco", (0.60, 0.76, 1.0), 10.0)
    h_ref = scene["car"].get("h_ref", 0.55)
    # A continuous sculpted black rocker carries each horizontal row. It joins
    # the undertray to the door skin, without separate pod housings.
    for side in (-1, 1):
        verts, faces = [], []
        for x in (-0.99, -0.90, 0.90, 0.99):
            taper = 0.06 if abs(x) > 0.95 else 0
            for y, z in ((0.70, -0.37), (0.87-taper, -0.37),
                         (0.893-taper, -0.27), (0.87-taper, -0.17), (0.70, -0.13)):
                verts.append((x, side*y, z))
        for i in range(3):
            for j in range(5):
                faces.append((i*5+j, i*5+(j+1)%5, (i+1)*5+(j+1)%5, (i+1)*5+j))
        faces.extend((tuple(reversed(range(5))), tuple(15+j for j in range(5))))
        sill = mesh_obj(f"MS_IntegratedSill_{side}", verts, faces, coll, [dark])
        sill.parent = rig
    plumes, cutters = {}, []
    for outlet in layout["outlets"]:
        channel = layout["channels"][outlet["channel"]]
        name = outlet["name"]
        pos = Vector(outlet["position_m"])
        force = Vector(outlet["force_direction"]).normalized()
        radius, depth = outlet["radius_m"], outlet["depth_m"]
        is_sd = channel["kind"] == "superdraco"
        root = empty(f"MS_Thr_{name}", coll, pos, rig)
        root.rotation_euler = Vector((0, 0, 1)).rotation_difference(force).to_euler()
        root["channel_name"] = channel["name"]
        root["canonical_exit_position_m"] = list(pos)
        root["force_direction"] = list(force)
        # Open a clear duct through the surrounding skin, including recessed
        # counterjet faces. This changes the body, never canonical outlet positions.
        cutter = prim("cylinder", f"duct_cutter_{name}", coll, None, vertices=32,
                      radius=radius*1.10, depth=0.70+depth,
                      location=pos+force*((depth-0.70)/2))
        cutter.rotation_euler = root.rotation_euler
        cutters.append(cutter)
        # Short annular nozzle, with a physically open mouth and interior throat.
        verts, faces = [], []
        for rr, zz in ((radius*1.06, 0.002), (radius, 0.002),
                       (radius*0.54, depth), (radius*0.64, depth)):
            verts.extend((rr*math.cos(2*math.pi*k/48), rr*math.sin(2*math.pi*k/48), zz)
                         for k in range(48))
        for ring in range(4):
            for k in range(48):
                faces.append((ring*48+k, ring*48+(k+1)%48,
                              ((ring+1)%4)*48+(k+1)%48, ((ring+1)%4)*48+k))
        bell = mesh_obj(f"MS_RecessedNozzle_{name}", verts, faces, coll, [nozzle])
        bell.parent = root
        throat = prim("cylinder", f"MS_NozzleThroat_{name}", coll, dark, vertices=32,
                      radius=radius*0.54, depth=0.004)
        throat.parent, throat.location = root, (0, 0, depth+0.002)
        pl = prim("cone", f"MS_Plume_{name}", coll, gas_sd if is_sd else gas_dr,
                  vertices=24, radius1=radius*1.8, radius2=radius*0.75, depth=1)
        pl.parent = root
        pl.location = (0, 0, -0.0005)
        pl.scale = (1, 1, 0.001)
        pl["full_len"] = 1.8 if is_sd else (0.65 if channel["kind"] == "corner" else 0.35)
        pl["channel_name"] = channel["name"]
        pl["exit_offset"] = 0.0
        plumes[name] = pl
    # One Boolean operation for the entire duct tool set bounds mesh processing.
    bpy.ops.object.select_all(action="DESELECT")
    for cutter in cutters:
        cutter.select_set(True)
    bpy.context.view_layer.objects.active = cutters[0]
    bpy.ops.object.join()
    tool = bpy.context.active_object
    for target in [body] + [o for o in coll.objects if o.name.startswith("MS_IntegratedSill_") or o.name == "MS_Ducktail"]:
        if target is not None:
            apply_modifiers(target)
            mod = target.modifiers.new("Canonical exit ducts", "BOOLEAN")
            mod.operation, mod.solver, mod.object = "DIFFERENCE", "EXACT", tool
            apply_modifiers(target)
    bpy.data.objects.remove(tool, do_unlink=True)
    if body is not None:
        # Analytic local-space trim keeps curved vent boundaries smooth despite
        # the underlying longitudinal loft topology and animated world pose.
        paint = bpy.data.materials["MS_Paint"]
        nt = paint.node_tree
        tc = nt.nodes.new("ShaderNodeTexCoord")
        sep = nt.nodes.new("ShaderNodeSeparateXYZ")
        nt.links.new(tc.outputs["Object"], sep.inputs[0])

        def math_node(operation, a, b):
            node = nt.nodes.new("ShaderNodeMath")
            node.operation = operation
            for index, value in enumerate((a,b)):
                if isinstance(value, (int,float)):
                    node.inputs[index].default_value = value
                else:
                    nt.links.new(value, node.inputs[index])
            return node.outputs[0]

        zx = math_node("MULTIPLY", math_node("ADD", sep.outputs["Z"], .31), 2.5)
        zz = math_node("MULTIPLY", zx, zx)
        mask = 0.0
        for x in sorted({c["position_m"][0] for c in layout["channels"] if c["kind"] == "corner"}):
            xx = math_node("MULTIPLY", math_node("SUBTRACT", sep.outputs["X"], x), 7.7)
            ellipse = math_node("LESS_THAN", math_node("ADD", math_node("MULTIPLY", xx, xx), zz), 1.0)
            mask = math_node("MAXIMUM", mask, ellipse)
        mask = math_node("MULTIPLY", mask, math_node("GREATER_THAN", math_node("ABSOLUTE", sep.outputs["Y"],0), .60))
        mix = nt.nodes.new("ShaderNodeMixRGB")
        mix.inputs[1].default_value = (*PAINT,1)
        mix.inputs[2].default_value = (.008,.01,.012,1)
        nt.links.new(mask, mix.inputs[0])
        nt.links.new(mix.outputs[0], nt.nodes["Principled BSDF"].inputs["Base Color"])
    return plumes


# ----------------------------------------------------------------------------- set
def build_set(coll, scene):
    tarmac = material("Set_Tarmac", (0.075, 0.075, 0.08), rough=0.8)
    steel = material("Set_Steel", (0.18, 0.18, 0.19), metallic=1.0, rough=0.45)
    stripe = material("Set_Stripe", (0.9, 0.9, 0.85), rough=0.5)
    bpy.ops.mesh.primitive_plane_add(size=600, location=(60, 0, 0))
    g = bpy.context.active_object
    g.name = "Set_Ground"
    link(g, coll)
    g.data.materials.append(tarmac)
    for s in (1, -1):
        beveled_box(f"Set_Lane_{s}", (200, 0.15, 0.005), (60, s * 2.6, 0.003), coll, stripe, 0)
    beveled_box("Set_StartLine", (0.4, 5.2, 0.006), (0, 0, 0.004), coll, stripe, 0)
    if "x_liftoff_m" in scene:
        beveled_box("Set_LiftoffMark", (0.25, 5.2, 0.006), (scene["x_liftoff_m"], 0, 0.004), coll,
                    material("Set_Orange", (0.9, 0.3, 0.02), rough=0.5), 0)
    tones = [(0.75, 0.75, 0.76), (0.55, 0.52, 0.45), (0.85, 0.85, 0.86), (0.42, 0.43, 0.45), (0.07, 0.07, 0.08)]
    tire = material("Lineup_Tire", (0.02, 0.02, 0.02), rough=0.8)
    for i, v in enumerate(scene["lineup"]):
        xc, wdt, ln, h = (v["x0"] + v["x1"]) / 2, v["x1"] - v["x0"], v["len"], v["h"]
        m = material(f"Lineup_{v['name']}", tones[i % len(tones)], metallic=0.6, rough=0.3, coat=0.8)
        tag = v["name"].replace(" ", "")
        if v["name"] == "Semi":
            beveled_box(f"Lineup_{tag}_Cab", (wdt, 2.6, h - 1.0), (xc, 1.6, 0.5 + (h - 1.0) / 2), coll, m, 0.25)
            beveled_box(f"Lineup_{tag}_Chassis", (wdt * 0.9, ln, 0.8), (xc, 0, 0.75), coll, steel, 0.05)
        else:
            beveled_box(f"Lineup_{tag}_Body", (wdt, ln, h * 0.55), (xc, 0, 0.18 + h * 0.275), coll, m, 0.18)
            beveled_box(f"Lineup_{tag}_Cabin", (wdt * 0.82, ln * 0.5, h * 0.42),
                        (xc, -ln * 0.05, h * 0.55 + h * 0.21), coll, m, 0.15)
        for sy in (-ln * 0.33, ln * 0.33):
            for sx in (-wdt / 2 + 0.12, wdt / 2 - 0.12):
                prim("cylinder", f"Lineup_{tag}_Wheel", coll, tire, radius=0.36, depth=0.25,
                     location=(xc + sx, sy, 0.36), rotation=(0, math.pi / 2, 0))


def build_world(scene_bpy):
    w = bpy.data.worlds.new("Sky")
    scene_bpy.world = w
    w.use_nodes = True
    nt = w.node_tree
    bg = nt.nodes.get("Background")
    try:
        sky = nt.nodes.new("ShaderNodeTexSky")
        for t in ("MULTIPLE_SCATTERING", "SINGLE_SCATTERING", "NISHITA"):
            try:
                sky.sky_type = t
                break
            except TypeError:
                continue
        if hasattr(sky, "sun_elevation"):
            sky.sun_elevation = math.radians(18)
            sky.sun_rotation = math.radians(210)
        nt.links.new(sky.outputs[0], bg.inputs[0])
        bg.inputs[1].default_value = 0.18
    except Exception:
        bg.inputs[0].default_value = (0.45, 0.6, 0.85, 1)
    bpy.ops.object.light_add(type="SUN", rotation=(math.radians(60), 0, math.radians(210)))
    sun = bpy.context.active_object
    sun.data.energy = 3.0
    sun.data.angle = math.radians(1.5)


def new_cam(coll, name, lens, target, loc=(0, 0, 0)):
    c = bpy.data.objects.new(name, bpy.data.cameras.new(name))
    coll.objects.link(c)
    c.location = loc
    c.data.lens = lens
    c.data.clip_end = 1000
    tr = c.constraints.new("TRACK_TO")
    tr.target = target
    tr.track_axis, tr.up_axis = "TRACK_NEGATIVE_Z", "UP_Y"
    return c


def build_cameras(coll, rig, scene, rows):
    target = empty("CamTarget", coll)
    target.parent = rig
    xs = [r["x"] for r in rows]
    x_mid = (min(xs) + max(xs)) / 2
    lx = [(v["x0"] + v["x1"]) / 2 for v in scene["lineup"]]
    c1 = new_cam(coll, "Cam_Chase", 32, target)
    c2 = new_cam(coll, "Cam_Wide", 35, target, (x_mid, -38, 4))
    c3 = new_cam(coll, "Cam_Hero", 38, target)
    c4 = new_cam(coll, "Cam_Side", 40, target)              # follows alongside for the backflip
    return c1, c2, c3, c4


# ----------------------------------------------------------------------------- animation
def read_traj(path):
    rows = []
    with open(path, newline="") as f:
        for r in csv.DictReader(f):
            rows.append({k: (v if k == "phase" else float(v)) for k, v in r.items()})
    return rows


def animate(rig, plumes, rows, cams, scene):
    sc = bpy.context.scene
    bpy.context.preferences.edit.keyframe_new_interpolation_type = "LINEAR"
    wheels = [o for o in bpy.data.objects if o.name.startswith("MS_Wheel_")]
    chase = cams[0]
    first = {}
    for f, r in enumerate(rows, start=1):
        first.setdefault(r["phase"], f)
        position = Vector((r["x"], r["y"], r["z"]))
        if all(k in r for k in ("q_w", "q_x", "q_y", "q_z")):
            rig.rotation_mode = "QUATERNION"
            attitude = Quaternion((r["q_w"], r["q_x"], r["q_y"], r["q_z"]))
            attitude.normalize()
            rig.rotation_quaternion = attitude
            cg_body = Vector((r.get("cg_x",0), r.get("cg_y",0), r.get("cg_z",0)))
            rig.location = position - attitude @ cg_body
        else:
            rig.rotation_euler = (r["roll"], -r["pitch"], r["yaw"])
            rig.location = position
        rig.keyframe_insert("location", frame=f)
        rig.keyframe_insert("rotation_quaternion" if rig.rotation_mode == "QUATERNION" else "rotation_euler", frame=f)
        for w in wheels:
            w.rotation_euler = (0, r["wheel_angle"], 0)
            w.keyframe_insert("rotation_euler", frame=f)
        for name, pl in plumes.items():
            k = r.get("thr_" + pl.get("channel_name", name), 0.0) / 100.0
            sz = max(k * pl["full_len"], 0.001)
            pl.scale = (0.5 + 0.5 * k, 0.5 + 0.5 * k, sz)
            pl.location.z = -pl.get("exit_offset", 0.0) - 0.5 * sz
            pl.hide_render = pl.hide_viewport = k < 0.01
            pl.keyframe_insert("scale", frame=f)
            pl.keyframe_insert("location", frame=f)
            pl.keyframe_insert("hide_render", frame=f)
            pl.keyframe_insert("hide_viewport", frame=f)
        chase.location = (r["x"] - 12, -6.0, max(r["z"] + 2.5, 2.2))
        chase.keyframe_insert("location", frame=f)
        if len(cams) > 3:
            cams[3].location = (r["x"] - 3.0, -16.0, max(r["z"] + 1.0, 1.5))
            cams[3].keyframe_insert("location", frame=f)
        cams[2].location = (r["x"] - 5.0, r["y"] - 10.0, r["z"] + 6.0)
        cams[2].keyframe_insert("location", frame=f)
    sc.frame_start, sc.frame_end = 1, len(rows)
    sc.render.fps = int(scene.get("fps", 24))
    sc.timeline_markers.clear()
    n = len(rows)
    fly = first.get("boost", first.get("flight", 1))
    cuts = {1: cams[0], max(2, fly - 12): cams[1], first.get("roll", fly) + 6: cams[2],
            first.get("pitch_kick", fly + 40) - 4: cams[-1], max(3, first.get("rollout", n) - 30): cams[0]}
    for f in sorted(cuts):
        mk = sc.timeline_markers.new(f"cut_{cuts[f].name}", frame=f)
        mk.camera = cuts[f]
    sc.camera = cams[0]
    return first


# ----------------------------------------------------------------------------- render
def configure_render(mode, out_dir):
    sc = bpy.context.scene
    r = sc.render
    vts = [i.identifier for i in sc.view_settings.bl_rna.properties["view_transform"].enum_items]
    sc.view_settings.view_transform = "AgX" if "AgX" in vts else "Filmic"
    try:
        sc.view_settings.look = "AgX - Medium High Contrast" if "AgX" in vts else "Medium High Contrast"
    except TypeError:
        pass
    if mode == "final":
        r.engine = "CYCLES"
        sc.cycles.samples = 256
        sc.cycles.use_denoising = True
        try:
            prefs = bpy.context.preferences.addons["cycles"].preferences
            for dev in ("OPTIX", "CUDA", "HIP", "METAL", "ONEAPI"):
                try:
                    prefs.compute_device_type = dev
                    prefs.get_devices()
                    if any(d.type == dev for d in prefs.devices):
                        for d in prefs.devices:
                            d.use = d.type == dev
                        sc.cycles.device = "GPU"
                        print("CYCLES_GPU", [(d.name, d.type) for d in prefs.devices if d.use], flush=True)
                        break
                except TypeError:
                    continue
        except Exception:
            pass
        if sc.cycles.device != "GPU":
            print("CYCLES_DEVICE_FALLBACK CPU: no supported GPU backend selected", flush=True)
        r.resolution_x, r.resolution_y, r.resolution_percentage = 1920, 1080, 100
        r.use_motion_blur = True
        r.motion_blur_shutter = 0.15
    else:
        engines = [e.identifier for e in r.bl_rna.properties["engine"].enum_items]
        r.engine = "BLENDER_EEVEE_NEXT" if "BLENDER_EEVEE_NEXT" in engines else "BLENDER_EEVEE"
        r.resolution_x, r.resolution_y, r.resolution_percentage = 1280, 720, 100
    r.filepath = str(Path(out_dir) / ("mission_s_" + mode + "_"))


def set_video_output():
    r = bpy.context.scene.render
    try:
        r.image_settings.media_type = "VIDEO"      # Blender 5.x
    except (AttributeError, TypeError):
        pass
    r.image_settings.file_format = "FFMPEG"
    r.ffmpeg.format = "MPEG4"
    r.ffmpeg.codec = "H264"
    r.ffmpeg.constant_rate_factor = "HIGH"


def set_png_output(frames_dir):
    r = bpy.context.scene.render
    try:
        r.image_settings.media_type = "IMAGE"
    except (AttributeError, TypeError):
        pass
    r.image_settings.file_format = "PNG"
    Path(frames_dir).mkdir(parents=True, exist_ok=True)
    r.filepath = str(Path(frames_dir) / "frame_")


# ----------------------------------------------------------------------------- main
def main():
    args = parse_args()
    root = project_dir()
    csv_path = Path(args["csv"] or root / "out" / "aero_pulsed" / "trajectory.csv").resolve()
    scene_path = Path(args["scene"] or root / "out" / "aero_pulsed" / "scene.json").resolve()
    out_dir = Path(args["out"] or root / "out" / "aero_pulsed").resolve()
    if args["save"]:
        args["save"] = str(Path(args["save"]).resolve())
    if args["frames_dir"]:
        args["frames_dir"] = str(Path(args["frames_dir"]).resolve())
    out_dir.mkdir(parents=True, exist_ok=True)
    scene = json.loads(scene_path.read_text())
    for k in ("lineup", "thruster_groups"):
        if isinstance(scene.get(k), dict):                     # MATLAB jsonencode of a 1-element array
            scene[k] = [scene[k]]
    rows = read_traj(csv_path)

    bpy.ops.wm.read_factory_settings(use_empty=True)
    sc = bpy.context.scene
    sc.unit_settings.system = "METRIC"
    colls = {n: bpy.data.collections.new(n) for n in ("MissionS", "Set", "Cameras")}
    for c in colls.values():
        sc.collection.children.link(c)

    car = dict(scene["car"])
    car.setdefault("h_ref", car.get("h_cg", 0.55))
    rig, plumes = build_car(colls["MissionS"], car, scene)
    build_set(colls["Set"], scene)
    build_world(sc)
    cams = build_cameras(colls["Cameras"], rig, scene, rows)
    first = animate(rig, plumes, rows, cams, scene)
    sc.frame_set(1)
    sc["trajectory_source"] = str(csv_path)
    sc["scene_source"] = str(scene_path)
    sc["canonical_outlets"] = len(plumes)
    print("frames:", len(rows), "phase start frames:", first)

    mode = args["render"]
    configure_render("final" if mode == "final" else "preview", out_dir)
    if args["engine"] == "cycles":
        sc.render.engine = "CYCLES"
    if args["samples"] and sc.render.engine == "CYCLES":
        sc.cycles.samples = int(args["samples"])
        sc.cycles.use_denoising = True
    if args["res"]:
        sc.render.resolution_percentage = int(args["res"])
    if args["save"]:
        bpy.ops.wm.save_as_mainfile(filepath=str(Path(args["save"]).resolve()))
    elif mode == "none":
        bpy.ops.wm.save_as_mainfile(filepath=str(out_dir / "mission_s_stunt.blend"))
    if mode in ("preview", "final"):
        if args["still"]:
            sc.frame_set(int(args["still"]))
            sc.render.filepath = str(out_dir / f"still_{int(args['still']):04d}.png")
            bpy.ops.render.render(write_still=True)
        else:
            if args["frames"]:
                a, b = (int(v) for v in args["frames"].split(":"))
                sc.frame_start, sc.frame_end = a, b
            if args["frames_dir"]:
                set_png_output(args["frames_dir"])
            else:
                set_video_output()
            bpy.ops.render.render(animation=True)
        print("rendered to", out_dir)


if __name__ == "__main__":
    main()
