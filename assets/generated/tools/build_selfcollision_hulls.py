"""Build the RB-Y1 self-collision layer: per-link convex hulls from the visual meshes.

The base model ships with physxArticulation:enabledSelfCollisions = False and capsule colliders that
miss 30-65 % of several arm/torso meshes; the head, link_torso_3, link_*_arm_6 and the gripper bodies
have no collider at all. This script writes generated/rby1_selfcollision_hulls.usda, which

  * turns articulation self-collision on,
  * deactivates the capsule colliders of link_torso_* and link_*_arm_0..5 (incl. the arm_5 baked_capsule),
  * adds one convex hull per link (torso_0..5, head_0..2, {left,right}_arm_0..6) built from that
    link's visual meshes, and two hulls per gripper body (ee_left/ee_right: hand body + D405 camera
    mount, kept separate so the hull does not cover the space between the fingers),
  * keeps base/wheel cylinders and the finger convex decompositions unchanged,
  * filters collisions inside each group of bodies welded together by fixed joints (base, torso_0..5,
    head_0..2 in this model): they cannot move relative to each other, and the coarse hulls of
    torso_2 and torso_4 overlap at the baked 30 deg torso_3 bend.

Each hull is reduced to at most 64 vertices (PhysX GPU convex limit) by greedy farthest-point
selection; the reduced hull lies inside the exact hull by at most a few millimetres (printed).
Link masses and inertias are authored explicitly in the model, so colliders do not change them.

Then rby1_ready_reach_table.usd is pointed at the new layer (which sublayers the old collision layer):

  rby1_ready_reach_table.usd -> rby1_selfcollision_hulls.usda -> rby1_fullbody_collision.usd -> ...

  python assets/generated/tools/build_selfcollision_hulls.py          # needs usd-core, numpy, scipy
"""
import argparse
from pathlib import Path

import numpy as np
from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics, Vt
from scipy.spatial import ConvexHull

GEN = Path(__file__).resolve().parents[1]
SOURCE = GEN / "rby1_fullbody_collision.usd"
OUTPUT = GEN / "rby1_selfcollision_hulls.usda"
TOP = GEN / "rby1_ready_reach_table.usd"
MAX_VERTS = 64

ARMS = [f"{s}_arm_{i}" for s in ("left", "right") for i in range(7)]
LINKS = [f"torso_{i}" for i in range(6)] + [f"head_{i}" for i in range(3)] + ARMS
CAPSULE_LINKS = ["torso_0", "torso_1", "torso_2", "torso_4", "torso_5"] + [a for a in ARMS if not a.endswith("_6")]
PRED = Usd.TraverseInstanceProxies(Usd.PrimAllPrimsPredicate)


def visual_points(body, root):
    """Visual mesh points under `root` that belong to rigid body `body`, in the body frame."""
    inv = np.linalg.inv(np.array(UsdGeom.Xformable(body).ComputeLocalToWorldTransform(0), float).T)
    pts = []
    for m in Usd.PrimRange(root, PRED):
        if not m.IsA(UsdGeom.Mesh) or m.HasAPI(UsdPhysics.CollisionAPI) or "collision" in str(m.GetPath()).lower():
            continue
        q = m.GetParent()
        while q and q != body and not q.HasAPI(UsdPhysics.RigidBodyAPI):
            q = q.GetParent()
        if q != body:
            continue  # belongs to a child rigid body
        v = np.asarray(UsdGeom.Mesh(m).GetPointsAttr().Get() or [], float)
        if len(v):
            M = inv @ np.array(UsdGeom.Xformable(m).ComputeLocalToWorldTransform(0), float).T
            pts.append(v @ M[:3, :3].T + M[:3, 3])
    return np.concatenate(pts)


def reduced_hull(P, nmax=MAX_VERTS):
    """(vertices, triangles, max inward error) of a hull of at most nmax vertices of conv(P)."""
    V = P[ConvexHull(P).vertices]
    dirs = np.array([[a, b, c] for a in (-1, 0, 1) for b in (-1, 0, 1) for c in (-1, 0, 1) if (a, b, c) != (0, 0, 0)], float)
    S = sorted({int(np.argmax(V @ d)) for d in dirs})
    while True:
        E = ConvexHull(V[S]).equations
        out = (V @ E[:, :3].T + E[:, 3]).max(axis=1)
        k = int(np.argmax(out))
        if len(S) >= nmax or out[k] < 1e-5:
            break
        S.append(k)
    h = ConvexHull(V[S])
    verts = V[S][h.vertices]
    remap = {int(o): i for i, o in enumerate(h.vertices)}
    tris = []
    for simplex, eq in zip(h.simplices, h.equations):
        t = [remap[int(i)] for i in simplex]
        a, b, d = verts[t]
        if np.cross(b - a, d - a) @ eq[:3] < 0:
            t = [t[0], t[2], t[1]]  # outward (counter-clockwise) winding
        tris.append(t)
    return verts, np.array(tris), max(float(out[k]), 0.0)


def rigid_groups(stage):
    """Bodies welded together by fixed joints (union-find over PhysicsFixedJoint), groups of 2+."""
    parent = {}

    def find(x):
        while parent.setdefault(x, x) != x:
            x = parent[x]
        return x
    for p in Usd.PrimRange(stage.GetPrimAtPath("/RBY1"), PRED):
        if p.IsA(UsdPhysics.FixedJoint):
            j = UsdPhysics.Joint(p)
            b = [str(t) for t in j.GetBody0Rel().GetTargets() + j.GetBody1Rel().GetTargets()]
            if len(b) == 2:
                parent[find(b[0])] = find(b[1])
    groups = {}
    for x in list(parent):
        groups.setdefault(find(x), []).append(x)
    return [sorted(g) for g in groups.values() if len(g) > 2]


def author_hull(stage, path, verts, tris):
    mesh = UsdGeom.Mesh.Define(stage, path)
    mesh.CreatePointsAttr(Vt.Vec3fArray([Gf.Vec3f(*map(float, p)) for p in verts]))
    mesh.CreateFaceVertexCountsAttr(Vt.IntArray([3] * len(tris)))
    mesh.CreateFaceVertexIndicesAttr(Vt.IntArray([int(i) for i in tris.ravel()]))
    mesh.CreateExtentAttr(Vt.Vec3fArray([Gf.Vec3f(*map(float, verts.min(0))), Gf.Vec3f(*map(float, verts.max(0)))]))
    mesh.CreateSubdivisionSchemeAttr("none")
    mesh.CreatePurposeAttr(UsdGeom.Tokens.guide)  # collider only, not rendered
    UsdPhysics.CollisionAPI.Apply(mesh.GetPrim())
    UsdPhysics.MeshCollisionAPI.Apply(mesh.GetPrim()).CreateApproximationAttr(UsdPhysics.Tokens.convexHull)
    prim = mesh.GetPrim()
    prim.AddAppliedSchema("PhysxConvexHullCollisionAPI")
    prim.CreateAttribute("physxConvexHullCollision:hullVertexLimit", Sdf.ValueTypeNames.Int).Set(MAX_VERTS)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--no-link", action="store_true", help="write the layer only; leave rby1_ready_reach_table.usd alone")
    a = ap.parse_args()
    src = Usd.Stage.Open(str(SOURCE))
    parts = []  # (body path, hull prim name, root prim of the visual meshes)
    for n in LINKS:
        body = src.GetPrimAtPath(f"/RBY1/link_{n}")
        parts.append((body, "collision_hull", body))
    for side in ("left", "right"):
        body = src.GetPrimAtPath(f"/RBY1/{side}_gripper/ee_{side}")
        parts.append((body, "collision_hull_body", body.GetChild("visuals")))
        parts.append((body, "collision_hull_camera", body.GetChild("CameraMount")))

    OUTPUT.unlink(missing_ok=True)
    layer = Sdf.Layer.CreateNew(str(OUTPUT))
    layer.subLayerPaths.append(SOURCE.name)
    layer.defaultPrim = "RBY1"
    layer.comment = "Generated by tools/build_selfcollision_hulls.py; do not edit by hand."
    out = Usd.Stage.Open(layer)
    out.OverridePrim("/RBY1").CreateAttribute("physxArticulation:enabledSelfCollisions", Sdf.ValueTypeNames.Bool).Set(True)
    for n in CAPSULE_LINKS:
        out.OverridePrim(f"/RBY1/link_{n}/collisions").SetActive(False)
    for group in rigid_groups(src):
        for i, body in enumerate(group):
            api = UsdPhysics.FilteredPairsAPI.Apply(out.OverridePrim(body))
            for other in group[i + 1:]:
                api.GetFilteredPairsRel().AddTarget(other)
        print("filtered rigid group:", [g.rsplit("/", 1)[-1] for g in group])
    print(f"{'hull':45s} {'points':>7s} {'verts':>5s} {'err mm':>6s}")
    for body, name, root in parts:
        P = visual_points(body, root)
        verts, tris, err = reduced_hull(P)
        path = body.GetPath().AppendChild(name)
        author_hull(out, path, verts, tris)
        print(f"{str(path):45s} {len(P):7d} {len(verts):5d} {1000 * err:6.2f}")
    layer.Save()
    print("wrote", OUTPUT)

    if not a.no_link:
        top = Sdf.Layer.FindOrOpen(str(TOP))
        subs = list(top.subLayerPaths)
        if OUTPUT.name not in subs:
            top.subLayerPaths[subs.index(SOURCE.name)] = OUTPUT.name
            top.Save()
        print(TOP.name, "sublayers", list(top.subLayerPaths))


if __name__ == "__main__":
    main()
