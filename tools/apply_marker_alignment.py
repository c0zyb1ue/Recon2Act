#!/usr/bin/env python3
"""Apply collider and object positions from human-placed desk debug markers."""

from __future__ import annotations

import argparse
import re
import shutil
import subprocess
import sys
from pathlib import Path


def _run_reader(usd_path: Path) -> str:
    script = Path(__file__).with_name("read_debug_marker_transforms.py")
    return subprocess.check_output([sys.executable, str(script), "--usd", str(usd_path)], text=True)


def _parse_vec(label: str, output: str) -> tuple[float, float, float]:
    match = re.search(rf"^\s*{re.escape(label)}:\s*\(([^)]+)\)", output, re.MULTILINE)
    if not match:
        raise RuntimeError(f"Could not parse vector for {label}")
    values = tuple(float(part.strip()) for part in match.group(1).split(","))
    if len(values) != 3:
        raise RuntimeError(f"Invalid vector for {label}: {match.group(1)}")
    return values


def _parse_optional_vec(label: str, output: str) -> tuple[float, float, float] | None:
    match = re.search(rf"^\s*{re.escape(label)}:\s*\(([^)]+)\)", output, re.MULTILINE)
    if not match:
        return None
    values = tuple(float(part.strip()) for part in match.group(1).split(","))
    if len(values) != 3:
        raise RuntimeError(f"Invalid vector for {label}: {match.group(1)}")
    return values


def _fmt(values: tuple[float, float, float]) -> str:
    return ", ".join(f"{value:.6f}".rstrip("0").rstrip(".") for value in values)


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


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--usd", type=Path, help="Backward-compatible path used as both marker input and scene output")
    parser.add_argument("--markers-usd", type=Path, help="USD saved from Isaac GUI that contains moved debug markers")
    parser.add_argument("--scene-usd", type=Path, help="Scene USD to update, usually scene_debug.usd")
    parser.add_argument("--mode", choices=("debug", "final"), default="debug")
    parser.add_argument("--dry-run", action="store_true")
    args = parser.parse_args()

    markers_usd = args.markers_usd or args.usd
    scene_usd = args.scene_usd or args.usd
    if markers_usd is None or scene_usd is None:
        raise SystemExit("Provide --usd or both --markers-usd and --scene-usd")

    output = _run_reader(markers_usd)
    collider_translate = _parse_vec("translate", output)
    collider_rotate = _parse_vec("rotateXYZ", output)
    collider_scale = _parse_vec("scale", output)
    marker_center = _parse_vec("DebugDeskCenterMarker", output)
    marker_fl = _parse_optional_vec("DebugDeskFrontLeftMarker", output)
    marker_fr = _parse_optional_vec("DebugDeskFrontRightMarker", output)
    marker_bl = _parse_optional_vec("DebugDeskBackLeftMarker", output)
    marker_br = _parse_optional_vec("DebugDeskBackRightMarker", output)
    plate = _parse_vec("Plate", output)
    orange001 = _parse_vec("Orange001", output)
    orange002 = _parse_vec("Orange002", output)
    orange003 = _parse_vec("Orange003", output)
    robot_match = re.search(r"LEISAAC_ROBOT_INIT_POS=([^\n]+)", output)
    if not robot_match:
        raise RuntimeError("Could not parse robot init candidate")
    robot_init = robot_match.group(1).strip()

    visibility = "inherited" if args.mode == "debug" else "invisible"
    text = scene_usd.read_text()
    text = _update_prim(
        text,
        "DeskTabletopCollider",
        translate=collider_translate,
        rotate=collider_rotate,
        scale=collider_scale,
        visibility=visibility,
    )
    text = _update_prim(text, "DebugDeskCenterMarker", translate=marker_center, visibility=visibility)
    if marker_fl is not None:
        text = _update_prim(text, "DebugDeskFrontLeftMarker", translate=marker_fl, visibility=visibility)
    if marker_fr is not None:
        text = _update_prim(text, "DebugDeskFrontRightMarker", translate=marker_fr, visibility=visibility)
    if marker_bl is not None:
        text = _update_prim(text, "DebugDeskBackLeftMarker", translate=marker_bl, visibility=visibility)
    if marker_br is not None:
        text = _update_prim(text, "DebugDeskBackRightMarker", translate=marker_br, visibility=visibility)
    text = _update_prim(text, "Plate", translate=plate)
    text = _update_prim(text, "Orange001", translate=orange001)
    text = _update_prim(text, "Orange002", translate=orange002)
    text = _update_prim(text, "Orange003", translate=orange003)

    print(output)
    print("Apply summary:")
    print(f"  markers_usd={markers_usd}")
    print(f"  scene_usd={scene_usd}")
    print(f"  mode={args.mode}")
    print(f"  robot_env=LEISAAC_ROBOT_INIT_POS={robot_init}")

    if args.dry_run:
        print("  dry_run=true")
        return

    backup = scene_usd.with_suffix(scene_usd.suffix + ".align.bak")
    shutil.copy2(scene_usd, backup)
    scene_usd.write_text(text)
    print(f"  backup={backup}")
    print("  applied=true")


if __name__ == "__main__":
    main()
