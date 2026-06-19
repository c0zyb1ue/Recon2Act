#!/usr/bin/env python3
"""Read a robot spawn marker from a USD stage and print the robot spawn pose candidate."""

from __future__ import annotations

import argparse
import math
import re
from pathlib import Path


def _fmt3(values: tuple[float, float, float]) -> str:
    return f"({values[0]:.6f}, {values[1]:.6f}, {values[2]:.6f})"


def _fmt4(values: tuple[float, float, float, float]) -> str:
    return f"({values[0]:.6f}, {values[1]:.6f}, {values[2]:.6f}, {values[3]:.6f})"


def _find_block(text: str, prim_name: str) -> str:
    match = re.search(
        rf'(?m)^[ \t]*(?:def|over)(?:\s+(?:Cube|Xform))?\s+"{re.escape(prim_name)}"(?:\n| \()',
        text,
    )
    if not match:
        raise RuntimeError(f"Could not find prim named {prim_name}")
    brace_index = text.find("{", match.end())
    if brace_index == -1:
        raise RuntimeError(f"Could not find opening brace for prim named {prim_name}")
    depth = 0
    for index in range(brace_index, len(text)):
        char = text[index]
        if char == "{":
            depth += 1
        elif char == "}":
            depth -= 1
            if depth == 0:
                return text[match.start() : index + 1]
    raise RuntimeError(f"Could not find end of prim named {prim_name}")


def _read_text_marker(usd_path: Path, prim_path: str) -> tuple[tuple[float, float, float], tuple[float, float, float, float]]:
    prim_name = prim_path.rstrip("/").split("/")[-1]
    block = _find_block(usd_path.read_text(), prim_name)
    translate_match = re.search(r"double3 xformOp:translate = \(([^)]+)\)", block)
    if not translate_match:
        raise RuntimeError(f"Missing xformOp:translate on {prim_name}")
    pos = tuple(float(part.strip()) for part in translate_match.group(1).split(","))
    if len(pos) != 3:
        raise RuntimeError(f"Invalid translate on {prim_name}")

    quat_match = re.search(r"quat[fd] xformOp:orient = \(([^)]+)\)", block)
    if quat_match:
        rot = tuple(float(part.strip()) for part in quat_match.group(1).split(","))
        if len(rot) == 4:
            return pos, rot
    return pos, (1.0, 0.0, 0.0, 0.0)


def _read_pxr_marker(usd_path: Path, prim_path: str) -> tuple[tuple[float, float, float], tuple[float, float, float, float]]:
    from pxr import Usd, UsdGeom

    stage = Usd.Stage.Open(str(usd_path))
    if stage is None:
        raise RuntimeError(f"Could not open USD stage: {usd_path}")

    prim = stage.GetPrimAtPath(prim_path)
    if not prim:
        prim_name = prim_path.rstrip("/").split("/")[-1]
        matches = [candidate for candidate in stage.Traverse() if candidate.GetName() == prim_name]
        if len(matches) == 1:
            prim = matches[0]
        elif len(matches) > 1:
            paths = ", ".join(str(candidate.GetPath()) for candidate in matches)
            raise RuntimeError(f"Found multiple prims named {prim_name}: {paths}")
    if not prim:
        raise RuntimeError(f"Could not find prim: {prim_path}")

    matrix = UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
    translation = matrix.ExtractTranslation()
    rotation = matrix.ExtractRotationQuat()
    pos = (float(translation[0]), float(translation[1]), float(translation[2]))
    imag = rotation.GetImaginary()
    rot = (float(rotation.GetReal()), float(imag[0]), float(imag[1]), float(imag[2]))
    return pos, rot


def _try_read_marker(
    usd_path: Path, prim_path: str
) -> tuple[tuple[float, float, float], tuple[float, float, float, float]] | None:
    try:
        return _read_pxr_marker(usd_path, prim_path)
    except Exception:
        try:
            return _read_text_marker(usd_path, prim_path)
        except Exception:
            return None


def _yaw_quat_wxyz(spawn_pos: tuple[float, float, float], facing_pos: tuple[float, float, float] | None) -> tuple[float, float, float, float]:
    if facing_pos is None:
        return (1.0, 0.0, 0.0, 0.0)
    dx = facing_pos[0] - spawn_pos[0]
    dy = facing_pos[1] - spawn_pos[1]
    if abs(dx) < 1e-6 and abs(dy) < 1e-6:
        return (1.0, 0.0, 0.0, 0.0)
    yaw = math.atan2(dy, dx)
    return (math.cos(yaw * 0.5), 0.0, 0.0, math.sin(yaw * 0.5))


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--usd", required=True, type=Path)
    parser.add_argument("--prim", default="/World/RobotSpawnMarker")
    parser.add_argument("--facing-prim", default="/World/RobotFacingMarker")
    parser.add_argument("--robot-base-offset", type=float, default=0.0)
    args = parser.parse_args()

    try:
        marker_pos, marker_rot = _read_pxr_marker(args.usd, args.prim)
    except Exception as exc:
        print(f"[WARN] pxr marker read failed, falling back to USDA text parsing: {exc}")
        marker_pos, marker_rot = _read_text_marker(args.usd, args.prim)

    robot_pos = (marker_pos[0], marker_pos[1], marker_pos[2] + args.robot_base_offset)
    facing_marker = _try_read_marker(args.usd, args.facing_prim)
    facing_pos = facing_marker[0] if facing_marker else None
    robot_rot = _yaw_quat_wxyz(marker_pos, facing_pos)

    print(f"RobotSpawnMarker world position: {_fmt3(marker_pos)}")
    print(f"RobotSpawnMarker world orientation_wxyz: {_fmt4(marker_rot)}")
    print(f"RobotFacingMarker world position: {_fmt3(facing_pos) if facing_pos else 'missing'}")
    print(f"robot_spawn_position_candidate: {_fmt3(robot_pos)}")
    print(f"robot_spawn_orientation_wxyz: {_fmt4(robot_rot)}")


if __name__ == "__main__":
    main()
