#!/usr/bin/env python3
"""Read transform ops for a prim in a USD file."""

from __future__ import annotations

import argparse
import re
from pathlib import Path


def _find_block(text: str, prim_name: str) -> str:
    match = re.search(rf'(?m)^[ \t]*def\s+\w+\s+"{re.escape(prim_name)}"', text)
    if not match:
        raise RuntimeError(f"Could not find prim named {prim_name}")
    brace_index = text.find("{", match.end())
    if brace_index == -1:
        raise RuntimeError(f"Could not find opening brace for {prim_name}")
    depth = 0
    for index in range(brace_index, len(text)):
        char = text[index]
        if char == "{":
            depth += 1
        elif char == "}":
            depth -= 1
            if depth == 0:
                return text[match.start() : index + 1]
    raise RuntimeError(f"Could not find end of {prim_name}")


def _read_text_ops(usd_path: Path, prim_path: str) -> None:
    text = usd_path.read_text()
    prim_name = prim_path.rstrip("/").split("/")[-1]
    block = _find_block(text, prim_name)
    print(f"prim: {prim_path}")
    for pattern in (
        r"double3 xformOp:translate = \([^)]+\)",
        r"double3 xformOp:rotateXYZ = \([^)]+\)",
        r"double3 xformOp:scale = \([^)]+\)",
        r"uniform token\[\] xformOpOrder = \[[^\]]+\]",
    ):
        for match in re.finditer(pattern, block):
            print(match.group(0))
    matrix_start = block.find("matrix4d xformOp:transform")
    if matrix_start != -1:
        matrix_end = block.find(")\n", matrix_start)
        print(block[matrix_start : matrix_end + 1])


def _read_pxr_ops(usd_path: Path, prim_path: str) -> None:
    from pxr import Usd, UsdGeom

    stage = Usd.Stage.Open(str(usd_path))
    if stage is None:
        raise RuntimeError(f"Could not open stage: {usd_path}")
    prim = stage.GetPrimAtPath(prim_path)
    if not prim:
        matches = [candidate for candidate in stage.Traverse() if str(candidate.GetPath()).endswith(prim_path)]
        if len(matches) == 1:
            prim = matches[0]
    if not prim:
        raise RuntimeError(f"Could not find prim: {prim_path}")

    xformable = UsdGeom.Xformable(prim)
    print(f"prim: {prim.GetPath()}")
    print("ordered xform ops:")
    for op in xformable.GetOrderedXformOps():
        print(f"  {op.GetOpName()} = {op.Get()}")
    matrix = xformable.ComputeLocalToWorldTransform(0)
    print("world matrix:")
    print(matrix)
    print(f"world translate: {matrix.ExtractTranslation()}")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--usd", required=True, type=Path)
    parser.add_argument("--prim", required=True)
    args = parser.parse_args()

    try:
        _read_pxr_ops(args.usd, args.prim)
    except Exception as exc:
        print(f"[WARN] pxr read failed, falling back to text parsing: {exc}")
        _read_text_ops(args.usd, args.prim)


if __name__ == "__main__":
    main()
