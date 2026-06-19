#!/usr/bin/env python3
"""Read human-placed desk debug markers and compute tabletop alignment values."""

from __future__ import annotations

import argparse
import math
import re
from pathlib import Path


CENTER_MARKER = "DebugDeskCenterMarker"
CORNER_MARKERS = (
    "DebugDeskFrontLeftMarker",
    "DebugDeskFrontRightMarker",
    "DebugDeskBackLeftMarker",
    "DebugDeskBackRightMarker",
)
MARKERS = (CENTER_MARKER, *CORNER_MARKERS)


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


def _fmt(value: tuple[float, float, float]) -> str:
    return f"({value[0]:.6f}, {value[1]:.6f}, {value[2]:.6f})"


def _read_with_pxr(usd_path: Path) -> dict[str, tuple[float, float, float]]:
    from pxr import Usd, UsdGeom

    stage = Usd.Stage.Open(str(usd_path))
    if stage is None:
        raise RuntimeError(f"Could not open USD stage: {usd_path}")

    result: dict[str, tuple[float, float, float]] = {}
    for marker in MARKERS:
        prim = stage.GetPrimAtPath(f"/World/{marker}")
        if not prim:
            matches = [candidate for candidate in stage.Traverse() if candidate.GetName() == marker]
            if len(matches) == 1:
                prim = matches[0]
            elif len(matches) > 1:
                paths = ", ".join(str(candidate.GetPath()) for candidate in matches)
                raise RuntimeError(f"Found multiple marker prims named {marker}: {paths}")
        if not prim:
            if marker == CENTER_MARKER:
                raise RuntimeError(f"Missing required marker prim named: {marker}")
            continue
        matrix = UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(0)
        translation = matrix.ExtractTranslation()
        result[marker] = (float(translation[0]), float(translation[1]), float(translation[2]))
    return result


def _find_block(text: str, prim_name: str) -> str:
    match = re.search(
        rf'(?m)^[ \t]*(?:def|over)(?:\s+(?:Cube|Xform))?\s+"{re.escape(prim_name)}"(?:\n| \()',
        text,
    )
    if not match:
        raise RuntimeError(f"Missing marker prim ending with: {prim_name}")
    brace_index = text.find("{", match.end())
    if brace_index == -1:
        raise RuntimeError(f"Could not find opening brace for marker: {prim_name}")
    depth = 0
    for index in range(brace_index, len(text)):
        char = text[index]
        if char == "{":
            depth += 1
        elif char == "}":
            depth -= 1
            if depth == 0:
                return text[match.start() : index + 1]
    raise RuntimeError(f"Could not find end of marker: {prim_name}")


def _read_from_text(usd_path: Path) -> dict[str, tuple[float, float, float]]:
    text = usd_path.read_text()
    result: dict[str, tuple[float, float, float]] = {}
    for marker in MARKERS:
        try:
            block = _find_block(text, marker)
        except RuntimeError:
            if marker == CENTER_MARKER:
                raise
            continue
        match = re.search(r"double3 xformOp:translate = \(([^)]+)\)", block)
        if match:
            values = tuple(float(part.strip()) for part in match.group(1).split(","))
            if len(values) != 3:
                raise RuntimeError(f"Invalid translate on /World/{marker}")
            result[marker] = values
            continue

        matrix_match = re.search(
            r"matrix4d xformOp:transform\s*=\s*\(\s*"
            r"\(([^)]*)\),\s*"
            r"\(([^)]*)\),\s*"
            r"\(([^)]*)\),\s*"
            r"\(([^)]*)\)\s*"
            r"\)",
            block,
            re.DOTALL,
        )
        if matrix_match:
            values = tuple(float(part.strip()) for part in matrix_match.group(4).split(",")[:3])
            if len(values) == 3:
                result[marker] = values
                continue

        if marker == CENTER_MARKER:
            raise RuntimeError(f"Missing xformOp translate/transform on /World/{marker}")
    return result


def _read_markers(usd_path: Path) -> dict[str, tuple[float, float, float]]:
    try:
        return _read_with_pxr(usd_path)
    except Exception as exc:
        print(f"[WARN] pxr world-transform read failed, falling back to USDA text parsing: {exc}")
        return _read_from_text(usd_path)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--usd", required=True, type=Path)
    parser.add_argument("--collider-thickness", type=float, default=0.04)
    parser.add_argument("--robot-clearance", type=float, default=0.05)
    parser.add_argument("--object-z-offset", type=float, default=0.05)
    parser.add_argument("--default-width", type=float, default=0.86)
    parser.add_argument("--default-depth", type=float, default=0.56)
    args = parser.parse_args()

    markers = _read_markers(args.usd)
    center = markers[CENTER_MARKER]
    have_corners = all(marker in markers for marker in CORNER_MARKERS)

    desk_center = center
    corner_center = None
    if have_corners:
        fl = markers["DebugDeskFrontLeftMarker"]
        fr = markers["DebugDeskFrontRightMarker"]
        bl = markers["DebugDeskBackLeftMarker"]
        br = markers["DebugDeskBackRightMarker"]
        corners = [fl, fr, bl, br]
        corner_center = _avg(corners)
        right_axis = _normalize(_avg([_sub(fr, fl), _sub(br, bl)]))
        back_axis = _normalize(_avg([_sub(bl, fl), _sub(br, fr)]))
        normal = _normalize(_cross(right_axis, back_axis))
        if normal[2] < 0:
            normal = _mul(normal, -1.0)
        width = (_norm(_sub(fr, fl)) + _norm(_sub(br, bl))) * 0.5
        depth = (_norm(_sub(bl, fl)) + _norm(_sub(br, fr))) * 0.5
    else:
        right_axis = (1.0, 0.0, 0.0)
        back_axis = (0.0, 1.0, 0.0)
        normal = (0.0, 0.0, 1.0)
        width = args.default_width
        depth = args.default_depth
    surface_z = desk_center[2]
    collider_center = (desk_center[0], desk_center[1], surface_z - args.collider_thickness * 0.5)
    collider_scale = (width * 0.5, depth * 0.5, args.collider_thickness)
    collider_rotate = (0.0, 0.0, math.degrees(math.atan2(right_axis[1], right_axis[0])))
    robot_pos = (desk_center[0], desk_center[1], surface_z + args.robot_clearance)

    plate_pos = _add(desk_center, _add(_mul(right_axis, width * 0.22), _mul(back_axis, 0.0)))
    orange1_pos = _add(desk_center, _add(_mul(right_axis, width * 0.15), _mul(back_axis, -depth * 0.22)))
    orange2_pos = _add(desk_center, _add(_mul(right_axis, width * 0.27), _mul(back_axis, depth * 0.08)))
    orange3_pos = _add(desk_center, _add(_mul(right_axis, width * 0.10), _mul(back_axis, depth * 0.25)))
    plate_pos = (plate_pos[0], plate_pos[1], surface_z + args.object_z_offset)
    orange1_pos = (orange1_pos[0], orange1_pos[1], surface_z + args.object_z_offset)
    orange2_pos = (orange2_pos[0], orange2_pos[1], surface_z + args.object_z_offset)
    orange3_pos = (orange3_pos[0], orange3_pos[1], surface_z + args.object_z_offset)

    tilt_deg = math.degrees(math.acos(max(-1.0, min(1.0, _dot(normal, (0.0, 0.0, 1.0))))))
    rotation_axis = _cross(normal, (0.0, 0.0, 1.0))

    print("Markers:")
    for marker in MARKERS:
        if marker in markers:
            print(f"  {marker}: {_fmt(markers[marker])}")
        else:
            print(f"  {marker}: missing")
    print()
    print("Computed from human-placed markers:")
    print(f"  desk_center: {_fmt(desk_center)}")
    if corner_center is not None:
        print(f"  corner_center: {_fmt(corner_center)}")
    else:
        print("  corner_center: unavailable")
    print(f"  tabletop_plane_normal: {_fmt(normal)}")
    print(f"  tabletop_width: {width:.6f}")
    print(f"  tabletop_depth: {depth:.6f}")
    print(f"  tabletop_tilt_from_world_z_deg: {tilt_deg:.6f}")
    print()
    print("DeskTabletopCollider candidate:")
    print(f"  translate: {_fmt(collider_center)}")
    print(f"  rotateXYZ: {_fmt(collider_rotate)}")
    print(f"  scale: {_fmt(collider_scale)}")
    print("  visibility: invisible for final, inherited only while debugging")
    if tilt_deg > 1.0:
        print("  note: tabletop markers are not world-horizontal; ignoring roll/pitch and using yaw-only collider rotation.")
    print()
    print("Robot init candidate:")
    print(f"  LEISAAC_ROBOT_INIT_POS={robot_pos[0]:.6f},{robot_pos[1]:.6f},{robot_pos[2]:.6f}")
    print()
    print("Plate/Orange candidates:")
    print(f"  Plate: {_fmt(plate_pos)}")
    print(f"  Orange001: {_fmt(orange1_pos)}")
    print(f"  Orange002: {_fmt(orange2_pos)}")
    print(f"  Orange003: {_fmt(orange3_pos)}")
    print()
    print("DeskVisual rotation guidance:")
    if tilt_deg <= 1.0:
        print("  Marker plane is already close to Isaac Z-up. Do not rotate DeskVisual for tilt.")
    else:
        print(f"  Plane normal differs from world Z by {tilt_deg:.3f} deg.")
        print(f"  Approx correction axis normal->Z: {_fmt(rotation_axis)}")
        print("  Use this only as a suggestion; visual table identity comes from human marker placement.")


if __name__ == "__main__":
    main()
