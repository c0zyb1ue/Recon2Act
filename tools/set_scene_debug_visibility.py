#!/usr/bin/env python3
"""Switch desk scene collider and marker visibility between debug and final modes."""

from __future__ import annotations

import argparse
import re
import shutil
from pathlib import Path


COLLIDER_TARGETS = (
    "DeskTabletopCollider",
    "DeskBackCollider",
)

MARKER_TARGETS = (
    "DebugDeskCenterMarker",
    "DebugDeskFrontLeftMarker",
    "DebugDeskFrontRightMarker",
    "DebugDeskBackLeftMarker",
    "DebugDeskBackRightMarker",
    "RobotSpawnMarker",
    "RobotFacingMarker",
    "PlateSpawnMarker",
    "Orange001SpawnMarker",
    "Orange002SpawnMarker",
    "Orange003SpawnMarker",
)
TARGETS = COLLIDER_TARGETS + MARKER_TARGETS


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


def _set_visibility(block: str, visibility: str) -> str:
    pattern = r'token visibility = "(?:inherited|invisible)"'
    replacement = f'token visibility = "{visibility}"'
    block, count = re.subn(pattern, replacement, block, count=1)
    if count == 0:
        insert_at = block.find("{") + 1
        block = block[:insert_at] + f'\n        token visibility = "{visibility}"' + block[insert_at:]
    return block


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--usd", required=True, type=Path)
    parser.add_argument("--mode", required=True, choices=("debug", "final"))
    args = parser.parse_args()

    text = args.usd.read_text()

    for prim_name in COLLIDER_TARGETS:
        try:
            start, end = _find_prim_block(text, prim_name)
        except RuntimeError:
            continue
        block = _set_visibility(text[start:end], "invisible")
        text = text[:start] + block + text[end:]

    marker_visibility = "inherited" if args.mode == "debug" else "invisible"
    for prim_name in MARKER_TARGETS:
        try:
            start, end = _find_prim_block(text, prim_name)
        except RuntimeError:
            continue
        block = _set_visibility(text[start:end], marker_visibility)
        text = text[:start] + block + text[end:]

    backup = args.usd.with_suffix(args.usd.suffix + ".bak")
    shutil.copy2(args.usd, backup)
    args.usd.write_text(text)

    print(f"Updated {args.usd}")
    print(f"  backup={backup}")
    print(f"  mode={args.mode}")
    print("  collider_visibility=invisible")
    print(f"  marker_visibility={marker_visibility}")


if __name__ == "__main__":
    main()
