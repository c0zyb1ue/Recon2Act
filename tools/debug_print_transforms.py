#!/usr/bin/env python3
"""Print local/world transforms for desk scene debugging."""

from __future__ import annotations

import argparse
import math
import os
import sys
from pathlib import Path


DEFAULT_PRIMS = [
    "/World",
    "/World/envs",
    "/World/envs/env_0",
    "/World/envs/env_0/Scene",
    "/World/envs/env_0/Scene/DeskVisual",
    "/World/envs/env_0/DebugMarkers",
    "/World/envs/env_0/DebugMarkers/RobotSpawnMarker",
    "/World/envs/env_0/DebugMarkers/PlateSpawnMarker",
    "/World/envs/env_0/DebugMarkers/Orange001SpawnMarker",
    "/World/envs/env_0/DebugMarkers/Orange002SpawnMarker",
    "/World/envs/env_0/DebugMarkers/Orange003SpawnMarker",
    "/World/envs/env_0/Scene/RobotSpawnMarker",
    "/World/envs/env_0/Scene/PlateSpawnMarker",
    "/World/envs/env_0/Scene/Orange001SpawnMarker",
    "/World/envs/env_0/Scene/Orange002SpawnMarker",
    "/World/envs/env_0/Scene/Orange003SpawnMarker",
]


def _bootstrap_pxr() -> None:
    try:
        import pxr  # noqa: F401

        return
    except Exception:
        pass

    roots = [
        Path("/home/rvi/anaconda3/envs/leisaac/lib/python3.11/site-packages/isaacsim/extscache"),
        Path(sys.executable).resolve().parents[1] / "lib/python3.11/site-packages/isaacsim/extscache",
    ]
    for root in roots:
        if not root.exists():
            continue
        matches = sorted(root.glob("omni.usd.libs-*/pxr/Usd/_usd.so"))
        if not matches:
            continue
        package = matches[-1].parents[2]
        env = os.environ.copy()
        env["PYTHONPATH"] = f"{package}:{env.get('PYTHONPATH', '')}"
        env["LD_LIBRARY_PATH"] = (
            f"{Path(sys.executable).resolve().parents[1] / 'lib'}:{package / 'bin'}:"
            f"{env.get('LD_LIBRARY_PATH', '')}"
        )
        env["LEISAAC_DEBUG_TRANSFORMS_REEXEC"] = "1"
        if os.environ.get("LEISAAC_DEBUG_TRANSFORMS_REEXEC") != "1":
            os.execvpe(sys.executable, [sys.executable, *sys.argv], env)
        sys.path.insert(0, str(package))
        return


def _fmt_vec(values, precision: int = 6) -> str:
    return "(" + ", ".join(f"{float(value):.{precision}f}" for value in values) + ")"


def _fmt_matrix(matrix) -> str:
    rows = []
    for i in range(4):
        rows.append("    " + _fmt_vec([matrix[i][j] for j in range(4)]))
    return "\n".join(rows)


def _find_by_suffix(stage, prim_path: str):
    suffix = "/" + prim_path.rsplit("/", 1)[-1]
    return [prim for prim in stage.Traverse() if str(prim.GetPath()).endswith(suffix)]


def _local_ops(prim):
    from pxr import UsdGeom

    if not prim or not prim.IsValid() or not prim.IsA(UsdGeom.Xformable):
        return []
    ops = []
    for op in UsdGeom.Xformable(prim).GetOrderedXformOps():
        value = op.Get()
        ops.append((op.GetOpName(), value))
    return ops


def _print_prim(stage, prim_path: str) -> None:
    from pxr import Usd, UsdGeom

    prim = stage.GetPrimAtPath(prim_path)
    print(f"\n=== {prim_path} ===")
    if not prim:
        print("status: MISSING")
        matches = _find_by_suffix(stage, prim_path)
        if matches:
            print("same-name candidates:")
            for match in matches:
                print(f"  {match.GetPath()}")
        return

    print(f"type: {prim.GetTypeName()}")
    print(f"path: {prim.GetPath()}")
    ops = _local_ops(prim)
    if not ops:
        print("local ops: none")
    else:
        print("local ops:")
        for name, value in ops:
            print(f"  {name} = {value}")

    xformable = UsdGeom.Xformable(prim)
    matrix = xformable.ComputeLocalToWorldTransform(Usd.TimeCode.Default())
    translation = matrix.ExtractTranslation()
    rotation = matrix.ExtractRotationQuat()
    imag = rotation.GetImaginary()
    print("world matrix:")
    print(_fmt_matrix(matrix))
    print(f"world position: {_fmt_vec([translation[0], translation[1], translation[2]])}")
    print(
        "world orientation_wxyz: "
        f"{_fmt_vec([rotation.GetReal(), imag[0], imag[1], imag[2]])}"
    )


def _has_nonidentity_rotation(prim) -> bool:
    if not prim:
        return False
    for name, value in _local_ops(prim):
        if name.endswith("rotateXYZ"):
            return any(abs(float(component)) > 1e-6 for component in value)
        if name.endswith("orient"):
            return abs(float(value.GetReal()) - 1.0) > 1e-6 or any(
                abs(float(component)) > 1e-6 for component in value.GetImaginary()
            )
        if name.endswith("transform"):
            rotation = value.ExtractRotationQuat()
            imag = rotation.GetImaginary()
            return abs(float(rotation.GetReal()) - 1.0) > 1e-6 or any(
                abs(float(component)) > 1e-6 for component in imag
            )
    return False


def _print_parent_rotation_check(stage) -> None:
    print("\n=== Parent Rotation Check ===")
    for path in ["/World/envs/env_0", "/World/envs/env_0/Scene", "/World/envs/env_0/DebugMarkers"]:
        prim = stage.GetPrimAtPath(path)
        if prim:
            print(f"{path}: rotation_present={_has_nonidentity_rotation(prim)}")
        else:
            print(f"{path}: missing")


def _print_table_plane(stage) -> None:
    from pxr import Gf, Usd, UsdGeom

    candidates = [
        "/World/envs/env_0/Scene/DeskRegionPlaneMarker",
        "/World/envs/env_0/Scene/DeskTabletopCollider",
        "/World/DeskRegionPlaneMarker",
        "/World/DeskTabletopCollider",
    ]
    prim = None
    for path in candidates:
        prim = stage.GetPrimAtPath(path)
        if prim:
            break
    print("\n=== Table Plane ===")
    if not prim:
        print("table plane prim: missing")
        return
    matrix = UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
    center = matrix.Transform(Gf.Vec3d(0, 0, 0))
    normal_end = matrix.Transform(Gf.Vec3d(0, 0, 1))
    normal = normal_end - center
    length = normal.GetLength()
    if length == 0:
        print(f"table plane prim: {prim.GetPath()}")
        print("normal: unavailable")
        return
    normal = normal / length
    angle = math.degrees(math.acos(max(-1.0, min(1.0, float(normal[2])))))
    print(f"table plane prim: {prim.GetPath()}")
    print(f"table plane normal: {_fmt_vec([normal[0], normal[1], normal[2]])}")
    print(f"angle_to_world_z_deg: {angle:.6f}")
    if angle >= 5.0:
        print("warning: table plane is tilted more than 5 degrees from world +Z")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--usd", required=True, type=Path)
    parser.add_argument("--prim", action="append", dest="prims")
    args = parser.parse_args()

    _bootstrap_pxr()
    from pxr import Usd

    stage = Usd.Stage.Open(str(args.usd))
    if stage is None:
        raise SystemExit(f"Could not open USD: {args.usd}")

    for prim_path in args.prims or DEFAULT_PRIMS:
        _print_prim(stage, prim_path)
    _print_parent_rotation_check(stage)
    _print_table_plane(stage)


if __name__ == "__main__":
    main()
