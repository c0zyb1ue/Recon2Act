#!/usr/bin/env python3
"""Inspect marker, object, and collider prim transforms in a saved USD stage."""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import import_saved_marker_transforms as marker_import  # noqa: E402


NAMES = {
    "Robot",
    "RobotSpawnMarker",
    "RobotFacingMarker",
    "Plate",
    "PlateSpawnMarker",
    "Orange001",
    "Orange001SpawnMarker",
    "Orange002",
    "Orange002SpawnMarker",
    "Orange003",
    "Orange003SpawnMarker",
    "DeskTabletopCollider",
    "DebugDeskCenterMarker",
    "DebugDeskFrontLeftMarker",
    "DebugDeskFrontRightMarker",
    "DebugDeskBackLeftMarker",
    "DebugDeskBackRightMarker",
}


def main() -> None:
    marker_import._ensure_pxr()
    from pxr import Usd, UsdGeom

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("usd", type=Path)
    args = parser.parse_args()

    stage = Usd.Stage.Open(str(args.usd))
    if stage is None:
        raise RuntimeError(f"Could not open USD stage: {args.usd}")

    found = 0
    for prim in stage.Traverse():
        if prim.GetName() not in NAMES:
            continue
        found += 1
        matrix = UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
        translation = matrix.ExtractTranslation()
        rotation = matrix.ExtractRotationQuat()
        imag = rotation.GetImaginary()
        print(str(prim.GetPath()))
        print(f"  name={prim.GetName()}")
        print(f"  type={prim.GetTypeName()}")
        print(f"  world_translate=({float(translation[0]):.6f}, {float(translation[1]):.6f}, {float(translation[2]):.6f})")
        print(
            "  world_orient_wxyz="
            f"({float(rotation.GetReal()):.6f}, {float(imag[0]):.6f}, {float(imag[1]):.6f}, {float(imag[2]):.6f})"
        )
        xformable = UsdGeom.Xformable(prim)
        for op in xformable.GetOrderedXformOps():
            print(f"  {op.GetOpName()}={op.Get()}")

    print(f"found={found}")


if __name__ == "__main__":
    main()
