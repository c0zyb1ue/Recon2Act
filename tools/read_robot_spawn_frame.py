#!/usr/bin/env python3
"""Read the robot spawn marker world frame from a USD file."""

from __future__ import annotations

import argparse
import math
import os
import sys
from pathlib import Path


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
        env["LEISAAC_READ_FRAME_REEXEC"] = "1"
        if os.environ.get("LEISAAC_READ_FRAME_REEXEC") != "1":
            os.execvpe(sys.executable, [sys.executable, *sys.argv], env)
        sys.path.insert(0, str(package))
        return


def _fmt(values, precision: int = 6) -> str:
    return "(" + ", ".join(f"{float(value):.{precision}f}" for value in values) + ")"


def _fmt_matrix(matrix) -> str:
    return "\n".join("  " + _fmt([matrix[i][j] for j in range(4)]) for i in range(4))


def _normalize(vec):
    length = math.sqrt(sum(float(v) * float(v) for v in vec))
    if length == 0:
        raise ValueError("Cannot normalize zero-length axis")
    return tuple(float(v) / length for v in vec)


def _quat_wxyz_from_axes(x_axis, y_axis, z_axis):
    r00, r01, r02 = x_axis[0], y_axis[0], z_axis[0]
    r10, r11, r12 = x_axis[1], y_axis[1], z_axis[1]
    r20, r21, r22 = x_axis[2], y_axis[2], z_axis[2]
    trace = r00 + r11 + r22
    if trace > 0:
        s = math.sqrt(trace + 1.0) * 2.0
        w = 0.25 * s
        x = (r21 - r12) / s
        y = (r02 - r20) / s
        z = (r10 - r01) / s
    elif r00 > r11 and r00 > r22:
        s = math.sqrt(1.0 + r00 - r11 - r22) * 2.0
        w = (r21 - r12) / s
        x = 0.25 * s
        y = (r01 + r10) / s
        z = (r02 + r20) / s
    elif r11 > r22:
        s = math.sqrt(1.0 + r11 - r00 - r22) * 2.0
        w = (r02 - r20) / s
        x = (r01 + r10) / s
        y = 0.25 * s
        z = (r12 + r21) / s
    else:
        s = math.sqrt(1.0 + r22 - r00 - r11) * 2.0
        w = (r10 - r01) / s
        x = (r02 + r20) / s
        y = (r12 + r21) / s
        z = 0.25 * s
    length = math.sqrt(w * w + x * x + y * y + z * z)
    return (w / length, x / length, y / length, z / length)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--usd", required=True, type=Path)
    parser.add_argument("--prim", default="/World/envs/env_0/DebugMarkers/RobotSpawnMarker")
    args = parser.parse_args()

    _bootstrap_pxr()
    from pxr import Gf, Usd, UsdGeom

    stage = Usd.Stage.Open(str(args.usd))
    if stage is None:
        raise SystemExit(f"Could not open USD: {args.usd}")
    prim = stage.GetPrimAtPath(args.prim)
    if not prim:
        matches = [candidate for candidate in stage.Traverse() if candidate.GetName() == args.prim.rsplit("/", 1)[-1]]
        if len(matches) == 1:
            prim = matches[0]
        elif matches:
            raise SystemExit("Multiple matching prims: " + ", ".join(str(match.GetPath()) for match in matches))
    if not prim:
        raise SystemExit(f"Missing prim: {args.prim}")

    xformable = UsdGeom.Xformable(prim)
    local_translate = None
    local_orientation = None
    for op in xformable.GetOrderedXformOps():
        value = op.Get()
        if op.GetOpName() == "xformOp:translate":
            local_translate = tuple(float(value[i]) for i in range(3))
        elif op.GetOpName() == "xformOp:orient":
            imag = value.GetImaginary()
            local_orientation = (float(value.GetReal()), float(imag[0]), float(imag[1]), float(imag[2]))
        elif op.GetOpName() == "xformOp:rotateXYZ":
            local_orientation = tuple(float(value[i]) for i in range(3))

    matrix = xformable.ComputeLocalToWorldTransform(Usd.TimeCode.Default())
    world_pos = matrix.ExtractTranslation()
    origin = matrix.Transform(Gf.Vec3d(0, 0, 0))
    x_axis = _normalize(matrix.Transform(Gf.Vec3d(1, 0, 0)) - origin)
    y_axis = _normalize(matrix.Transform(Gf.Vec3d(0, 1, 0)) - origin)
    z_axis = _normalize(matrix.Transform(Gf.Vec3d(0, 0, 1)) - origin)
    world_quat_wxyz = _quat_wxyz_from_axes(x_axis, y_axis, z_axis)

    print(f"prim: {prim.GetPath()}")
    print(f"marker local translate: {_fmt(local_translate or (0, 0, 0))}")
    print(f"marker local orientation: {local_orientation}")
    print(f"marker world position: {_fmt([world_pos[0], world_pos[1], world_pos[2]])}")
    print(f"marker world quaternion_wxyz: {_fmt(world_quat_wxyz)}")
    print("marker world transform matrix:")
    print(_fmt_matrix(matrix))
    print(f"marker_x_axis_world: {_fmt(x_axis)}")
    print(f"marker_y_axis_world: {_fmt(y_axis)}")
    print(f"marker_z_axis_world: {_fmt(z_axis)}")


if __name__ == "__main__":
    main()
