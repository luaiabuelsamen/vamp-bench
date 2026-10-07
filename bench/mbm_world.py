"""Convert a VAMP/MotionBenchMaker problem dict into a cuRobo WorldConfig with the same geometry VAMP uses.

VAMP (vamp.problem_dict_to_vamp) models MBM cylinders as capsules (end points at +-length/2 on local z),
except in the `box` scenario where they become cuboids with half extents (r, r, length/2).
We reproduce exactly that here.
"""
from curobo.geom.types import Capsule, Cuboid, Sphere, WorldConfig


def _pose(o):
    x, y, z, w = o["orientation_quat_xyzw"]
    return list(o["position"]) + [w, x, y, z]


def problem_to_world(p):
    cuboids, capsules, spheres = [], [], []
    for o in p["sphere"]:
        spheres.append(Sphere(name=o["name"], radius=o["radius"], pose=list(o["position"]) + [1, 0, 0, 0]))
    for o in p["cylinder"]:
        if p["problem"] == "box":
            cuboids.append(Cuboid(name=o["name"], pose=_pose(o), dims=[2 * o["radius"], 2 * o["radius"], o["length"]]))
        else:
            h = o["length"] / 2
            capsules.append(Capsule(name=o["name"], radius=o["radius"], base=[0, 0, -h], tip=[0, 0, h], pose=_pose(o)))
    for o in p["box"]:
        cuboids.append(Cuboid(name=o["name"], pose=_pose(o), dims=[2 * v for v in o["half_extents"]]))
    # Cuboids stay analytic (cuRobo OBB checker); capsules/spheres become triangle meshes (cuRobo MESH checker),
    # since cuRobo v1 has no analytic capsule/sphere obstacle.
    meshes = [o.get_mesh() for o in capsules + spheres]
    return WorldConfig(cuboid=cuboids, mesh=meshes)
