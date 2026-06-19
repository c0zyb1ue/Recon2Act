#!/usr/bin/env python3
"""Print SO101 prim paths and approximate local bounding boxes."""

from __future__ import annotations

from pathlib import Path

from read_robot_spawn_frame import _bootstrap_pxr


def _fmt(values) -> str:
    return "(" + ", ".join(f"{float(value):.6f}" for value in values) + ")"


def main() -> None:
    _bootstrap_pxr()
    from pxr import Gf, Usd, UsdGeom

    usd_path = Path("assets/robots/so101_follower.usd")
    stage = Usd.Stage.Open(str(usd_path))
    if stage is None:
        raise SystemExit(f"Could not open {usd_path}")

    default_prim = stage.GetDefaultPrim()
    print(f"defaultPrim: {default_prim.GetPath() if default_prim else None}")
    print("matching prims:")
    for prim in stage.Traverse():
        name = prim.GetName().lower()
        if any(key in name for key in ("base", "visual", "collision", "root", "shoulder", "wrist", "gripper", "jaw")):
            print(f"  {prim.GetPath()} {prim.GetTypeName()}")

    cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_, UsdGeom.Tokens.render, UsdGeom.Tokens.proxy])
    root_path = default_prim.GetPath() if default_prim else None
    if root_path is None:
        return

    root_xform_inv = UsdGeom.Xformable(default_prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default()).GetInverse()
    print("local-space world-bound boxes:")
    for prim in stage.Traverse():
        name = prim.GetName().lower()
        if not any(key in name for key in ("base", "visuals", "collisions", "shoulder")):
            continue
        try:
            box = cache.ComputeWorldBound(prim).ComputeAlignedBox()
        except Exception:
            continue
        if box.IsEmpty():
            continue
        min_pt = box.GetMin()
        max_pt = box.GetMax()
        corners = [
            Gf.Vec3d(x, y, z)
            for x in (min_pt[0], max_pt[0])
            for y in (min_pt[1], max_pt[1])
            for z in (min_pt[2], max_pt[2])
        ]
        local = [root_xform_inv.Transform(corner) for corner in corners]
        local_min = tuple(min(point[i] for point in local) for i in range(3))
        local_max = tuple(max(point[i] for point in local) for i in range(3))
        print(f"  {prim.GetPath()} min={_fmt(local_min)} max={_fmt(local_max)}")


if __name__ == "__main__":
    main()
