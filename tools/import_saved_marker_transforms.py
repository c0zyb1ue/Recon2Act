#!/usr/bin/env python3
"""Import human-placed runtime marker transforms from a saved Isaac USD stage."""

from __future__ import annotations

import argparse
import math
import os
import re
import shutil
import sys
from pathlib import Path


DEBUG_MARKERS = (
    "DebugDeskCenterMarker",
    "DebugDeskFrontLeftMarker",
    "DebugDeskFrontRightMarker",
    "DebugDeskBackLeftMarker",
    "DebugDeskBackRightMarker",
)
SPAWN_MARKERS = (
    "RobotSpawnMarker",
    "RobotFacingMarker",
    "PlateSpawnMarker",
    "Orange001SpawnMarker",
    "Orange002SpawnMarker",
    "Orange003SpawnMarker",
)
OBJECT_FROM_MARKER = {
    "Plate": "PlateSpawnMarker",
    "Orange001": "Orange001SpawnMarker",
    "Orange002": "Orange002SpawnMarker",
    "Orange003": "Orange003SpawnMarker",
}


def _ensure_pxr() -> None:
    try:
        from pxr import Usd  # noqa: F401

        return
    except Exception:
        pass

    if os.environ.get("LEISAAC_USD_REEXEC") == "1":
        return

    candidates = sorted(
        Path(sys.prefix).glob("lib/python*/site-packages/isaacsim/extscache/omni.usd.libs-*.cp*/")
    )
    if not candidates:
        return

    ext_dir = candidates[-1]
    env = os.environ.copy()
    env["LEISAAC_USD_REEXEC"] = "1"
    env["PYTHONPATH"] = f"{ext_dir}{os.pathsep}{env.get('PYTHONPATH', '')}"
    lib_paths = [Path(sys.prefix) / "lib", ext_dir / "bin", ext_dir / "bin" / "deps"]
    env["LD_LIBRARY_PATH"] = os.pathsep.join(str(path) for path in lib_paths) + os.pathsep + env.get(
        "LD_LIBRARY_PATH", ""
    )
    os.execvpe(sys.executable, [sys.executable, *sys.argv], env)


def _sub(a: tuple[float, float, float], b: tuple[float, float, float]) -> tuple[float, float, float]:
    return (a[0] - b[0], a[1] - b[1], a[2] - b[2])


def _add(a: tuple[float, float, float], b: tuple[float, float, float]) -> tuple[float, float, float]:
    return (a[0] + b[0], a[1] + b[1], a[2] + b[2])


def _mul(a: tuple[float, float, float], scalar: float) -> tuple[float, float, float]:
    return (a[0] * scalar, a[1] * scalar, a[2] * scalar)


def _dot(a: tuple[float, float, float], b: tuple[float, float, float]) -> float:
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def _cross(a: tuple[float, float, float], b: tuple[float, float, float]) -> tuple[float, float, float]:
    return (
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    )


def _norm(a: tuple[float, float, float]) -> float:
    return math.sqrt(_dot(a, a))


def _normalize(a: tuple[float, float, float]) -> tuple[float, float, float]:
    length = _norm(a)
    if length == 0:
        raise ValueError("Cannot normalize a zero-length vector")
    return (a[0] / length, a[1] / length, a[2] / length)


def _avg(points: list[tuple[float, float, float]]) -> tuple[float, float, float]:
    total = (0.0, 0.0, 0.0)
    for point in points:
        total = _add(total, point)
    return _mul(total, 1.0 / len(points))


def _fmt(values: tuple[float, float, float]) -> str:
    return ", ".join(f"{value:.6f}" for value in values)


def _find_unique_prim(stage, name: str):
    matches = [prim for prim in stage.Traverse() if prim.GetName() == name]
    if len(matches) != 1:
        paths = ", ".join(str(prim.GetPath()) for prim in matches) or "none"
        raise RuntimeError(f"Expected exactly one prim named {name}, found {len(matches)}: {paths}")
    return matches[0]


def _find_optional_unique_prim(stage, name: str):
    matches = [prim for prim in stage.Traverse() if prim.GetName() == name]
    if len(matches) > 1:
        paths = ", ".join(str(prim.GetPath()) for prim in matches)
        raise RuntimeError(f"Expected at most one prim named {name}, found {len(matches)}: {paths}")
    return matches[0] if matches else None


def _read_world_positions(source_usd: Path, names: tuple[str, ...]) -> dict[str, tuple[float, float, float]]:
    from pxr import Usd, UsdGeom

    stage = Usd.Stage.Open(str(source_usd))
    if stage is None:
        raise RuntimeError(f"Could not open USD stage: {source_usd}")

    positions: dict[str, tuple[float, float, float]] = {}
    for name in names:
        prim = _find_optional_unique_prim(stage, name) if name == "RobotFacingMarker" else _find_unique_prim(stage, name)
        if prim is None:
            continue
        matrix = UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
        translation = matrix.ExtractTranslation()
        positions[name] = (float(translation[0]), float(translation[1]), float(translation[2]))
    return positions


def _compute_collider(
    positions: dict[str, tuple[float, float, float]], thickness: float
) -> tuple[tuple[float, float, float], tuple[float, float, float], tuple[float, float, float]]:
    center = positions["DebugDeskCenterMarker"]
    fl = positions["DebugDeskFrontLeftMarker"]
    fr = positions["DebugDeskFrontRightMarker"]
    bl = positions["DebugDeskBackLeftMarker"]
    br = positions["DebugDeskBackRightMarker"]
    right_axis = _normalize(_avg([_sub(fr, fl), _sub(br, bl)]))
    back_axis = _normalize(_avg([_sub(bl, fl), _sub(br, fr)]))
    width = (_norm(_sub(fr, fl)) + _norm(_sub(br, bl))) * 0.5
    depth = (_norm(_sub(bl, fl)) + _norm(_sub(br, fr))) * 0.5
    translate = (center[0], center[1], center[2] - thickness * 0.5)
    rotate = (0.0, 0.0, math.degrees(math.atan2(right_axis[1], right_axis[0])))
    scale = (width * 0.5, depth * 0.5, thickness)
    return translate, rotate, scale


def _find_prim_block(text: str, prim_name: str) -> tuple[int, int]:
    match = re.search(rf'(?m)^    def (?:Cube|Xform) "{re.escape(prim_name)}"(?:\n| \()', text)
    if not match:
        raise RuntimeError(f"Could not find /World/{prim_name}")
    brace_index = text.find("{", match.end())
    if brace_index == -1:
        raise RuntimeError(f"Could not find opening brace for /World/{prim_name}")
    depth = 0
    for index in range(brace_index, len(text)):
        char = text[index]
        if char == "{":
            depth += 1
        elif char == "}":
            depth -= 1
            if depth == 0:
                return match.start(), index + 1
    raise RuntimeError(f"Could not find end of /World/{prim_name}")


def _replace_vec(block: str, op_name: str, values: tuple[float, float, float]) -> str:
    pattern = rf"double3 {re.escape(op_name)} = \([^)]+\)"
    replacement = f"double3 {op_name} = ({_fmt(values)})"
    block, count = re.subn(pattern, replacement, block, count=1)
    if count != 1:
        raise RuntimeError(f"Could not update {op_name}")
    return block


def _replace_visibility(block: str, visibility: str) -> str:
    pattern = r'token visibility = "(?:inherited|invisible)"'
    replacement = f'token visibility = "{visibility}"'
    block, count = re.subn(pattern, replacement, block, count=1)
    if count == 0:
        insert_at = block.find("{") + 1
        block = block[:insert_at] + f'\n        token visibility = "{visibility}"' + block[insert_at:]
    return block


def _update_prim(
    text: str,
    prim_name: str,
    *,
    translate: tuple[float, float, float] | None = None,
    rotate: tuple[float, float, float] | None = None,
    scale: tuple[float, float, float] | None = None,
    visibility: str | None = None,
) -> str:
    start, end = _find_prim_block(text, prim_name)
    block = text[start:end]
    if translate is not None:
        block = _replace_vec(block, "xformOp:translate", translate)
    if rotate is not None:
        block = _replace_vec(block, "xformOp:rotateXYZ", rotate)
    if scale is not None:
        block = _replace_vec(block, "xformOp:scale", scale)
    if visibility is not None:
        block = _replace_visibility(block, visibility)
    return text[:start] + block + text[end:]


def _apply_scene(
    scene_usd: Path,
    positions: dict[str, tuple[float, float, float]],
    *,
    marker_visibility: str,
    collider_thickness: float,
    apply_objects: bool,
    dry_run: bool,
) -> None:
    text = scene_usd.read_text()
    collider_translate, collider_rotate, collider_scale = _compute_collider(positions, collider_thickness)
    text = _update_prim(
        text,
        "DeskTabletopCollider",
        translate=collider_translate,
        rotate=collider_rotate,
        scale=collider_scale,
        visibility="invisible",
    )
    for marker in DEBUG_MARKERS + SPAWN_MARKERS:
        if marker in positions:
            try:
                text = _update_prim(text, marker, translate=positions[marker], visibility=marker_visibility)
            except RuntimeError:
                if marker in SPAWN_MARKERS:
                    raise
    if apply_objects:
        for obj_name, marker_name in OBJECT_FROM_MARKER.items():
            text = _update_prim(text, obj_name, translate=positions[marker_name])

    print(f"scene_usd={scene_usd}")
    print(f"  DeskTabletopCollider.translate=({_fmt(collider_translate)})")
    print(f"  DeskTabletopCollider.rotateXYZ=({_fmt(collider_rotate)})")
    print(f"  DeskTabletopCollider.scale=({_fmt(collider_scale)})")
    print(f"  apply_objects={str(apply_objects).lower()}")
    if dry_run:
        print("  dry_run=true")
        return

    backup = scene_usd.with_suffix(scene_usd.suffix + ".marker-import.bak")
    shutil.copy2(scene_usd, backup)
    scene_usd.write_text(text)
    print(f"  backup={backup}")
    print("  applied=true")


def main() -> None:
    _ensure_pxr()

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source-usd", required=True, type=Path)
    parser.add_argument("--scene-usd", required=True, action="append", type=Path)
    parser.add_argument("--debug-scene-usd", type=Path)
    parser.add_argument("--robot-base-offset", type=float, default=0.05)
    parser.add_argument("--collider-thickness", type=float, default=0.04)
    parser.add_argument("--apply-objects", action="store_true")
    parser.add_argument("--dry-run", action="store_true")
    args = parser.parse_args()

    positions = _read_world_positions(args.source_usd, DEBUG_MARKERS + SPAWN_MARKERS)
    robot_marker = positions["RobotSpawnMarker"]
    robot_pos = (robot_marker[0], robot_marker[1], robot_marker[2] + args.robot_base_offset)

    print(f"source_usd={args.source_usd}")
    for marker in DEBUG_MARKERS + SPAWN_MARKERS:
        if marker in positions:
            print(f"{marker}=({_fmt(positions[marker])})")
        else:
            print(f"{marker}=missing")
    print(f"robot_spawn_position=({_fmt(robot_pos)})")
    print("robot_spawn_orientation_wxyz=(1.000000, 0.000000, 0.000000, 0.000000)")

    for scene_usd in args.scene_usd:
        marker_visibility = "inherited" if args.debug_scene_usd and scene_usd == args.debug_scene_usd else "invisible"
        _apply_scene(
            scene_usd,
            positions,
            marker_visibility=marker_visibility,
            collider_thickness=args.collider_thickness,
            apply_objects=args.apply_objects,
            dry_run=args.dry_run,
        )


if __name__ == "__main__":
    main()
