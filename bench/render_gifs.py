"""Render benchmark queries as GIFs in MuJoCo (offscreen, EGL).

For each problem set: one representative query (VAMP wall time closest to its set's median, trial 0).
- Obstacles: exactly as both planners model them (bench/mbm_world.py semantics: capsules for MBM cylinders,
  cuboids for boxes and for the MBM `box` scene's cylinders).
- Start and goal shown as translucent ghosts; the robot executes the planner's returned path.
- Robots:
  - UR5e and UR5e-on-rail: ~/projects/curobo_berkeley/assets/ur5e/ur5e.xml (arm meshes), rail meshes from
    assets/ur5e/mesh/gantry. The Vention pedestal meshes are removed, because no planner models them.
  - Panda and UR5 (MBM): VAMP's URDFs, converted here. The UR5 uses its collision STLs because its visual
    meshes are .dae, which MuJoCo cannot load.
  Every model's kinematics is checked against VAMP's FK (end-effector pose at random configurations) before
  rendering; the script stops if they disagree.
- Display timing: paths are replayed at constant joint-space speed over a fixed duration. Planning times in
  the titles are the measured wall-clock times from results/raw.
Writes results/media/*.gif.
Run: MUJOCO_GL=egl PYTHONPATH=bench:.vamp_only ~/projects/clean_env/venv/bin/python bench/render_gifs.py
"""
import json, math, os, pickle, sys
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
import mujoco
import vamp
from PIL import Image, ImageDraw, ImageFont

ROOT = Path(__file__).resolve().parents[1]
CB = Path.home() / "projects/curobo_berkeley/assets/ur5e"
OUT = ROOT / "results/media"; OUT.mkdir(parents=True, exist_ok=True)
W, H, FPS = 480, 360, 12
HOLD_S, MOVE_S = 1.0, 6.0
FONT = ImageFont.truetype(str(Path(__import__("matplotlib").__file__).parent / "mpl-data/fonts/ttf/DejaVuSans.ttf"), 13)
FONT_B = ImageFont.truetype(str(Path(__import__("matplotlib").__file__).parent / "mpl-data/fonts/ttf/DejaVuSans-Bold.ttf"), 14)
START_RGBA, GOAL_RGBA, OBST_RGBA = (0.16, 0.47, 0.84, 0.30), (0.11, 0.69, 0.48, 0.30), (0.50, 0.52, 0.58, 0.55)


# ---------------------------------------------------------------- robot models
def rpy_quat(r, p, y):
    cr, sr, cp, sp, cy, sy = math.cos(r / 2), math.sin(r / 2), math.cos(p / 2), math.sin(p / 2), math.cos(y / 2), math.sin(y / 2)
    return [cr * cp * cy + sr * sp * sy, sr * cp * cy - cr * sp * sy, cr * sp * cy + sr * cp * sy, cr * cp * sy - sr * sp * cy]


def urdf_to_spec(urdf):
    """Minimal URDF -> MjSpec: kinematic tree, joints, mesh visuals (OBJ/STL; .dae falls back to the collision mesh)."""
    root = ET.parse(urdf).getroot(); base = Path(urdf).parent
    links = {l.get("name"): l for l in root.findall("link")}
    joints = root.findall("joint")
    child_of = {j.find("child").get("link"): j for j in joints}
    kids = {}
    for j in joints:
        kids.setdefault(j.find("parent").get("link"), []).append(j)
    rootlink = next(n for n in links if n not in child_of)
    spec = mujoco.MjSpec(); spec.compiler.degree = False
    meshes = {}

    def mesh_file(el):
        m = el.find("geometry/mesh")
        if m is None:
            return None, None
        f = m.get("filename").replace("package://", "")
        sc = [float(x) for x in m.get("scale", "1 1 1").split()]
        return base / f, sc

    def add_geoms(body, link):
        el = None
        for tag in ("visual", "collision"):
            for e in link.findall(tag):
                f, sc = mesh_file(e)
                if f is not None and f.suffix.lower() in (".obj", ".stl") and f.exists():
                    el = e; break
            if el is not None:
                break
        if el is None:
            return
        f, sc = mesh_file(el)
        key = (str(f), tuple(sc))
        if key not in meshes:
            name = f"m{len(meshes)}"; meshes[key] = name
            spec.add_mesh(name=name, file=str(f), scale=sc)
        o = el.find("origin")
        g = body.add_geom(type=mujoco.mjtGeom.mjGEOM_MESH, meshname=meshes[key], contype=0, conaffinity=0,
                          rgba=[0.78, 0.78, 0.8, 1])
        if o is not None:
            g.pos = [float(x) for x in o.get("xyz", "0 0 0").split()]
            g.quat = rpy_quat(*[float(x) for x in o.get("rpy", "0 0 0").split()])

    def build(parent_body, link):
        add_geoms(parent_body, links[link])
        for j in kids.get(link, []):
            c = j.find("child").get("link"); o = j.find("origin")
            b = parent_body.add_body(name=c)
            if o is not None:
                b.pos = [float(x) for x in o.get("xyz", "0 0 0").split()]
                b.quat = rpy_quat(*[float(x) for x in o.get("rpy", "0 0 0").split()])
            t = j.get("type")
            if t in ("revolute", "continuous", "prismatic"):
                ax = j.find("axis"); axis = [float(x) for x in (ax.get("xyz") if ax is not None else "1 0 0").split()]
                b.add_joint(name=j.get("name"), axis=axis,
                            type=mujoco.mjtJoint.mjJNT_SLIDE if t == "prismatic" else mujoco.mjtJoint.mjJNT_HINGE)
            build(b, c)

    rb = spec.worldbody.add_body(name=rootlink)
    build(rb, rootlink)
    return spec


def ur5e_spec(rail):
    arm = mujoco.MjSpec.from_file(str(CB / "ur5e.xml"))
    for g in list(arm.geoms):
        if g.meshname in ("vention", "vention2", "vention3"):
            arm.delete(g)
    for lt in list(arm.lights):
        arm.delete(lt)
    for a in list(arm.actuators):
        arm.delete(a)
    if not rail:
        return arm
    spec = mujoco.MjSpec(); spec.compiler.degree = False
    spec.add_mesh(name="bs_link", file=str(CB / "mesh/gantry/bs_link.STL"))
    spec.add_mesh(name="linear_rail", file=str(CB / "mesh/gantry/linear_rail.STL"))
    bs = spec.worldbody.add_body(name="bs_link", pos=[0, 0, -0.12])
    bs.add_geom(type=mujoco.mjtGeom.mjGEOM_MESH, meshname="bs_link", contype=0, conaffinity=0, rgba=[0.7, 0.7, 0.72, 1])
    lr = bs.add_body(name="linear_rail", pos=[0, 0, 0.12])
    lr.add_joint(name="base_x", type=mujoco.mjtJoint.mjJNT_SLIDE, axis=[1, 0, 0], range=[-1, 1])
    lr.add_geom(type=mujoco.mjtGeom.mjGEOM_MESH, meshname="linear_rail", contype=0, conaffinity=0, rgba=[0.6, 0.6, 0.62, 1])
    frame = lr.add_frame()
    frame.attach_body(arm.body("base"), "", "")
    return spec


ROBOTS = {  # vamp module -> (spec builder, MuJoCo EE frame (kind, name), extra base yaw)
    "panda": (lambda: urdf_to_spec(ROOT / "third_party/vamp/resources/panda/panda.urdf"), ("body", "panda_grasptarget")),
    "ur5": (lambda: urdf_to_spec(ROOT / "third_party/vamp/resources/ur5/ur5.urdf"), ("body", "robotiq_85_base_link")),
    "ur5e": (lambda: ur5e_spec(False), ("site", "attachment_site")),
    "ur5erail": (lambda: ur5e_spec(True), ("site", "attachment_site")),
}


# ---------------------------------------------------------------- scene
def euler_quat_xyzw_to_wxyz(q):
    x, y, z, w = q
    return [w, x, y, z]


def add_obstacles(spec, p):
    wb = spec.worldbody
    for o in p["sphere"]:
        wb.add_geom(type=mujoco.mjtGeom.mjGEOM_SPHERE, size=[o["radius"], 0, 0], pos=o["position"], rgba=OBST_RGBA, contype=0, conaffinity=0)
    for o in p["cylinder"]:
        q = euler_quat_xyzw_to_wxyz(o["orientation_quat_xyzw"])
        if p["problem"] == "box":
            wb.add_geom(type=mujoco.mjtGeom.mjGEOM_BOX, size=[o["radius"], o["radius"], o["length"] / 2], pos=o["position"], quat=q, rgba=OBST_RGBA, contype=0, conaffinity=0)
        else:
            wb.add_geom(type=mujoco.mjtGeom.mjGEOM_CAPSULE, size=[o["radius"], o["length"] / 2, 0], pos=o["position"], quat=q, rgba=OBST_RGBA, contype=0, conaffinity=0)
    for o in p["box"]:
        wb.add_geom(type=mujoco.mjtGeom.mjGEOM_BOX, size=o["half_extents"], pos=o["position"], quat=euler_quat_xyzw_to_wxyz(o["orientation_quat_xyzw"]), rgba=OBST_RGBA, contype=0, conaffinity=0)


def finish_scene(spec):
    spec.visual.headlight.ambient = [0.45, 0.45, 0.45]; spec.visual.headlight.diffuse = [0.5, 0.5, 0.5]
    spec.worldbody.add_light(pos=[1.5, -1.5, 3], dir=[-0.4, 0.4, -1], diffuse=[0.5, 0.5, 0.5], type=mujoco.mjtLightType.mjLIGHT_DIRECTIONAL)
    spec.visual.global_.offwidth, spec.visual.global_.offheight = W, H
    tex = spec.add_texture(name="sky", type=mujoco.mjtTexture.mjTEXTURE_SKYBOX, builtin=mujoco.mjtBuiltin.mjBUILTIN_GRADIENT,
                           rgb1=[0.98, 0.98, 0.97], rgb2=[0.88, 0.89, 0.9], width=64, height=64)


def qpos_index(model, names):
    return [model.jnt_qposadr[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, n)] for n in names]


def check_fk(model, vm, ee, rng):
    """MuJoCo EE pose vs VAMP eefk at random in-bounds configurations. Returns (max pos err m, max rot err rad, yaw fix)."""
    vmod = getattr(vamp, vm); names = vmod.joint_names(); idx = qpos_index(model, names)
    lo, hi = np.array(vmod.lower_bounds()), np.array(vmod.upper_bounds())
    d = mujoco.MjData(model); pe, re = 0.0, 0.0
    for _ in range(50):
        q = rng.uniform(np.maximum(lo, -3), np.minimum(hi, 3))
        d.qpos[:] = 0; d.qpos[idx] = q; mujoco.mj_kinematics(model, d)
        if ee[0] == "site":
            i = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, ee[1]); p, R = d.site_xpos[i], d.site_xmat[i].reshape(3, 3)
        else:
            i = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, ee[1]); p, R = d.xpos[i], d.xmat[i].reshape(3, 3)
        T = np.asarray(vmod.eefk(q.astype(np.float32).tolist()))
        pe = max(pe, float(np.linalg.norm(p - T[:3, 3])))
        c = (np.trace(R.T @ T[:3, :3]) - 1) / 2; re = max(re, float(np.arccos(np.clip(c, -1, 1))))
    return pe, re


# ---------------------------------------------------------------- rendering
def resample(path, n):
    q = np.asarray(path, float)
    seg = np.linalg.norm(np.diff(q, axis=0), axis=1); s = np.concatenate([[0], np.cumsum(seg)])
    if s[-1] == 0:
        return np.repeat(q[:1], n, 0)
    t = np.linspace(0, s[-1], n)
    return np.stack([np.interp(t, s, q[:, k]) for k in range(q.shape[1])], 1)


def ghost(renderer, model, data, rgba):
    sc = renderer.scene; n0 = sc.ngeom
    mujoco.mjv_addGeoms(model, data, mujoco.MjvOption(), mujoco.MjvPerturb(), mujoco.mjtCatBit.mjCAT_DYNAMIC, sc)
    for k in range(n0, sc.ngeom):
        sc.geoms[k].rgba[:] = rgba


def render_query(vm, p, path, cam, title_lines, rng):
    build, ee = ROBOTS[vm]
    spec = build(); add_obstacles(spec, p); finish_scene(spec)
    model = spec.compile()
    pe, re = check_fk(model, vm, ee, rng)
    if pe > 5e-3 or re > 5e-3:  # display tolerance; measured values go to the manifest
        sys.exit(f"{vm}: MuJoCo model disagrees with VAMP FK (pos {pe:.4f} m, rot {re:.4f} rad)")
    names = getattr(vamp, vm).joint_names(); idx = qpos_index(model, names)
    d, ds, dg = mujoco.MjData(model), mujoco.MjData(model), mujoco.MjData(model)
    ds.qpos[idx] = p["start"]; dg.qpos[idx] = p["goals"][0]
    mujoco.mj_kinematics(model, ds); mujoco.mj_kinematics(model, dg)
    r = mujoco.Renderer(model, H, W)
    c = mujoco.MjvCamera(); c.lookat[:], c.distance, c.azimuth, c.elevation = auto_camera(vm, p, path, cam)
    opt = mujoco.MjvOption(); opt.geomgroup[:] = [1, 1, 1, 0, 0, 0]  # hide UR5e collision primitives (group 3)
    traj = resample(path, int(MOVE_S * FPS))
    seq = [traj[0]] * int(HOLD_S * FPS) + list(traj) + [traj[-1]] * int(HOLD_S * FPS)
    frames = []
    for q in seq:
        d.qpos[idx] = q; mujoco.mj_kinematics(model, d)
        r.update_scene(d, camera=c, scene_option=opt)
        ghost(r, model, ds, START_RGBA); ghost(r, model, dg, GOAL_RGBA)
        frames.append(overlay(r.render(), title_lines))
    return frames, (pe, re)


def auto_camera(vm, p, path, elev):
    """Frame the end-effector sweep, the robot base and the obstacles near them; look from the side of the
    base->obstacle direction so the robot and the obstacles are side by side."""
    vmod = getattr(vamp, vm)
    ee = np.array([np.asarray(vmod.eefk(q.astype(np.float32).tolist()))[:3, 3] for q in resample(path, 40)])
    base = np.array(vamp.ROBOT_FIRST_JOINT_LOCATIONS.get(vm, [0.0, 0.0, 0.0]))  # first joint (UR5 MBM: z = 0.91)
    obs = [np.array(o["position"]) for k in ("box", "cylinder", "sphere") for o in p[k]
           if max(o.get("half_extents", [0])) < 0.8]  # skip floors/tables for framing
    near = [o for o in obs if np.min(np.linalg.norm(ee - o, axis=1)) < 0.6]
    pts = np.vstack([ee, base[None], *([np.array(near)] if near else [])])
    lo, hi = pts.min(0), pts.max(0); look = (lo + hi) / 2
    ext = float(np.linalg.norm(hi - lo))
    d = (np.mean(near, 0) if near else ee.mean(0))[:2] - base[:2]
    az = math.degrees(math.atan2(d[1], d[0])) + 90 if np.linalg.norm(d) > 1e-3 else 135
    return look, max(1.6, 1.25 * ext + 0.6), az, elev


def overlay(img, lines):
    im = Image.fromarray(img); dr = ImageDraw.Draw(im, "RGBA")
    h = 8 + 18 * len(lines)
    dr.rectangle([0, 0, im.width, h], fill=(252, 252, 251, 215))
    for k, (txt, bold) in enumerate(lines):
        dr.text((8, 5 + 18 * k), txt, font=FONT_B if bold else FONT, fill=(11, 11, 11, 255) if bold else (82, 81, 78, 255))
    dr.rectangle([0, im.height - 22, im.width, im.height], fill=(252, 252, 251, 215))
    dr.text((8, im.height - 18), "translucent ghosts: start (blue) · goal (green)", font=FONT, fill=(82, 81, 78, 255))
    return np.asarray(im)


def save_gif(frames, path):
    ims = [Image.fromarray(f).quantize(colors=128, method=Image.Quantize.MEDIANCUT, dither=Image.Dither.NONE) for f in frames]
    ims[0].save(path, save_all=True, append_images=ims[1:], duration=int(1000 / FPS), loop=0, optimize=True)
    return path.stat().st_size


def fmt_ms(ms):
    return f"{ms / 1000:.2f} s" if ms >= 1000 else (f"{ms:.0f} ms" if ms >= 10 else f"{ms:.2f} ms")


# ---------------------------------------------------------------- main
SETS = [  # name, tag, vamp module, curobo robot, problems file, camera (lookat, dist, azimuth, elevation), file
    ("Panda · MotionBenchMaker", "mbm", "panda", "panda", ROOT / "third_party/vamp/resources/panda/problems.pkl", -25, "panda_mbm"),
    ("UR5 · MotionBenchMaker", "mbm", "ur5", "ur5", ROOT / "third_party/vamp/resources/ur5/problems.pkl", -25, "ur5_mbm"),
    ("UR5e + 7th-axis rail · pillars (paper)", "gantry_pillars", "ur5erail", "ur5e_rail", ROOT / "problems/gantry_pillars.pkl", -28, "ur5e_rail_pillars"),
    ("UR5e + 7th-axis rail · no obstacles (paper)", "gantry_free", "ur5erail", "ur5e_rail", ROOT / "problems/gantry_free.pkl", -28, "ur5e_rail_free"),
    ("UR5e · pillars (paper)", "ur5e_pillars", "ur5e", "ur5e", ROOT / "problems/ur5e_pillars.pkl", -28, "ur5e_pillars"),
]
rng = np.random.default_rng(0)
manifest = {"display": f"{W}x{H}, {FPS} fps, {HOLD_S}s hold + {MOVE_S}s motion + {HOLD_S}s hold; paths replayed at constant joint speed", "gifs": {}}
for name, tag, vm, cr, pk, cam, fn in SETS:
    probs = pickle.load(open(pk, "rb"))["problems"]
    V = json.load(open(ROOT / f"results/raw/vamp_{tag}_{vm}.json"))["records"]
    ok = [r for r in V if r.get("success") and r.get("trial", 0) == 0]
    med = np.median([r["wall_us"] for r in ok]); r = min(ok, key=lambda x: abs(x["wall_us"] - med))
    p = probs[r["scene"]][r["index"]]
    lines = [(name, True), (f"VAMP, 1 CPU core · plan {fmt_ms(r['wall_us'] / 1e3)} · {r['scene']} #{r['index']}", False)]
    frames, fk = render_query(vm, p, r["path"], cam, lines, rng)
    size = save_gif(frames, OUT / f"{fn}.gif")
    manifest["gifs"][f"{fn}.gif"] = {"set": name, "planner": "VAMP", "scene": r["scene"], "index": r["index"], "trial": 0,
                                     "plan_wall_ms": r["wall_us"] / 1e3, "set_median_ms": med / 1e3, "frames": len(frames),
                                     "seconds": len(frames) / FPS, "bytes": size, "fk_check_max_pos_m": fk[0], "fk_check_max_rot_rad": fk[1]}
    print(fn, f"{size / 1e6:.2f} MB", len(frames) / FPS, "s", "fk", fk, flush=True)

# side by side: VAMP vs cuRobo v1, same query, rail + pillars
name, tag, vm, cr, pk, cam, fn = SETS[2]
probs = pickle.load(open(pk, "rb"))["problems"]
V = {(r["scene"], r["index"]): r for r in json.load(open(ROOT / f"results/raw/vamp_{tag}_{vm}.json"))["records"] if r.get("success") and r.get("trial", 0) == 0}
C = {(r["scene"], r["index"]): r for r in json.load(open(ROOT / f"results/raw/curobo_{tag}_{cr}.json"))["records"]
     if r.get("success") and r.get("trial", 0) == 0 and r.get("vamp_validated")}
both = sorted(set(V) & set(C))
cw = {k: C[k]["world_update_us"] + C[k]["plan_wall_us"] for k in both}
med = np.median(list(cw.values())); k = min(both, key=lambda x: abs(cw[x] - med))
p = probs[k[0]][k[1]]
fl, _ = render_query(vm, p, V[k]["path"], cam, [(name.split(" (")[0], True), (f"VAMP, 1 CPU core · plan {fmt_ms(V[k]['wall_us'] / 1e3)}", False)], rng)
fr, _ = render_query(vm, p, C[k]["path"], cam, [(name.split(" (")[0], True), (f"cuRobo v1, GPU · plan {fmt_ms(cw[k] / 1e3)}", False)], rng)
frames = [np.concatenate([a, np.full((H, 4, 3), 252, np.uint8), b], 1) for a, b in zip(fl, fr)]
size = save_gif(frames, OUT / "side_by_side_ur5e_rail_pillars.gif")
manifest["gifs"]["side_by_side_ur5e_rail_pillars.gif"] = {
    "set": name, "scene": k[0], "index": k[1], "trial": 0, "vamp_plan_wall_ms": V[k]["wall_us"] / 1e3,
    "curobo_v1_plan_wall_ms": cw[k] / 1e3, "curobo_v1_query_is_median_time_of_queries_both_solved": True,
    "note": "both motions replayed over the same display duration; cuRobo's own trajectory duration is "
            f"{C[k].get('motion_time_s')} s", "frames": len(frames), "seconds": len(frames) / FPS, "bytes": size}
print("side_by_side", f"{size / 1e6:.2f} MB", flush=True)
json.dump(manifest, open(OUT / "manifest.json", "w"), indent=1)
print("wrote", OUT / "manifest.json")
