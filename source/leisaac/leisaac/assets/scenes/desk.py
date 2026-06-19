import os
from pathlib import Path

import isaaclab.sim as sim_utils
from isaaclab.assets import AssetBaseCfg
from leisaac.utils.constant import ASSETS_ROOT

"""Configuration for the Desk with Orange Scene"""
SCENES_ROOT = Path(ASSETS_ROOT) / "scenes"

DESK_WITH_ORANGE_USD_PATH = os.environ.get(
    "LEISAAC_DESK_SCENE_USD",
    str(SCENES_ROOT / "desk_with_orange" / "scene.usd"),
)
print(f"[LeIsaac] Using desk scene USD: {DESK_WITH_ORANGE_USD_PATH}")

DESK_DEBUG_MARKERS_USD_PATH = os.environ.get(
    "LEISAAC_DESK_DEBUG_MARKERS_USD",
    str(SCENES_ROOT / "desk_with_orange" / "debug_markers.usd"),
)
print(f"[LeIsaac] Using desk debug markers USD: {DESK_DEBUG_MARKERS_USD_PATH}")

DESK_WITH_ORANGE_CFG = AssetBaseCfg(
    spawn=sim_utils.UsdFileCfg(
        usd_path=DESK_WITH_ORANGE_USD_PATH,
    )
)

DESK_DEBUG_MARKERS_CFG = AssetBaseCfg(
    spawn=sim_utils.UsdFileCfg(
        usd_path=DESK_DEBUG_MARKERS_USD_PATH,
    )
)
