#!/usr/bin/env python3
"""Update only /World/DeskVisual transform in a LeIsaac desk scene USDA wrapper."""

from __future__ import annotations

import argparse
import re
import shutil
from pathlib import Path


def _format_vec(values: list[float]) -> str:
    return ", ".join(f"{value:g}" for value in values)


def _find_desk_visual_block(text: str) -> tuple[int, int]:
    match = re.search(r'(?m)^    def Xform "DeskVisual"\n    \{', text)
    if not match:
        raise RuntimeError('Could not find /World/DeskVisual block')

    depth = 0
    block_start = match.start()
    for index in range(match.end() - 1, len(text)):
        char = text[index]
        if char == "{":
            depth += 1
        elif char == "}":
            depth -= 1
            if depth == 0:
                return block_start, index + 1
    raise RuntimeError("Could not find end of /World/DeskVisual block")


def _replace_vec(block: str, op_name: str, values: list[float]) -> str:
    pattern = rf"double3 {re.escape(op_name)} = \([^)]+\)"
    replacement = f"double3 {op_name} = ({_format_vec(values)})"
    block, count = re.subn(pattern, replacement, block, count=1)
    if count != 1:
        raise RuntimeError(f"Could not find transform op: {op_name}")
    return block


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--usd", required=True, type=Path, help="Path to scene_debug.usd")
    parser.add_argument("--translate", required=True, nargs=3, type=float, metavar=("X", "Y", "Z"))
    parser.add_argument("--rotate", required=True, nargs=3, type=float, metavar=("RX", "RY", "RZ"))
    parser.add_argument("--scale", required=True, nargs=3, type=float, metavar=("SX", "SY", "SZ"))
    args = parser.parse_args()

    text = args.usd.read_text()
    start, end = _find_desk_visual_block(text)
    block = text[start:end]
    block = _replace_vec(block, "xformOp:translate", args.translate)
    block = _replace_vec(block, "xformOp:rotateXYZ", args.rotate)
    block = _replace_vec(block, "xformOp:scale", args.scale)
    text = text[:start] + block + text[end:]

    backup = args.usd.with_suffix(args.usd.suffix + ".bak")
    shutil.copy2(args.usd, backup)
    args.usd.write_text(text)

    print(f"Updated {args.usd}")
    print(f"  backup={backup}")
    print(f"  translate=({_format_vec(args.translate)})")
    print(f"  rotateXYZ=({_format_vec(args.rotate)})")
    print(f"  scale=({_format_vec(args.scale)})")


if __name__ == "__main__":
    main()
