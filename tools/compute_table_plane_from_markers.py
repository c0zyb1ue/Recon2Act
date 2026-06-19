#!/usr/bin/env python3
"""Compute tabletop plane from DebugDesk marker positions."""

from __future__ import annotations

import runpy
from pathlib import Path


if __name__ == "__main__":
    runpy.run_path(str(Path(__file__).with_name("read_debug_marker_transforms.py")), run_name="__main__")
