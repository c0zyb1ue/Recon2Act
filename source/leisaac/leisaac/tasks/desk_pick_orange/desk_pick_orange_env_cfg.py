import os
import re
import math
from pathlib import Path

import torch
from isaaclab.assets import AssetBaseCfg
from isaaclab.managers import EventTermCfg as EventTerm
from isaaclab.managers import ObservationGroupCfg as ObsGroup
from isaaclab.managers import ObservationTermCfg as ObsTerm
from isaaclab.managers import SceneEntityCfg
from isaaclab.managers import TerminationTermCfg as DoneTerm
from isaaclab.sensors import TiledCameraCfg
from isaaclab.utils import configclass
from leisaac.assets.scenes.desk import (
    DESK_DEBUG_MARKERS_CFG,
    DESK_DEBUG_MARKERS_USD_PATH,
    DESK_WITH_ORANGE_CFG,
    DESK_WITH_ORANGE_USD_PATH,
)
from leisaac.utils.general_assets import parse_usd_and_create_subassets

from ..pick_orange import mdp
from ..template import (
    SingleArmEventCfg,
    SingleArmObservationsCfg,
    SingleArmTaskEnvCfg,
    SingleArmTaskSceneCfg,
    SingleArmTerminationsCfg,
)


OBJECT_SPAWN_MARKERS = {
    "Plate": "PlateSpawnMarker",
    "Orange001": "Orange001SpawnMarker",
    "Orange003": "Orange003SpawnMarker",
}

FRONT_CAMERA_POS = (1.3255667074339175, 2.1554210690751097, -0.39224101265889155)
FRONT_CAMERA_TARGET_POS = (1.421302, 3.085018, -0.845254)
FRONT_CAMERA_ROT_ROS = (0.0, 0.0, 1.0, 0.0)


def _env_vec3(name: str, default: tuple[float, float, float]) -> tuple[float, float, float]:
    raw = os.environ.get(name)
    if not raw:
        return default
    try:
        values = tuple(float(part.strip()) for part in raw.split(","))
    except ValueError as exc:
        raise ValueError(f"{name} must be comma-separated floats, e.g. 1.05,-0.65,0.89") from exc
    if len(values) != 3:
        raise ValueError(f"{name} must have exactly three values, e.g. 1.05,-0.65,0.89")
    return values


def _env_vec4(name: str, default: tuple[float, float, float, float]) -> tuple[float, float, float, float]:
    raw = os.environ.get(name)
    if not raw:
        return default
    try:
        values = tuple(float(part.strip()) for part in raw.split(","))
    except ValueError as exc:
        raise ValueError(f"{name} must be comma-separated floats, e.g. 1,0,0,0") from exc
    if len(values) != 4:
        raise ValueError(f"{name} must have exactly four values, e.g. 1,0,0,0")
    return values


def _optional_env_vec4(name: str) -> tuple[float, float, float, float] | None:
    raw = os.environ.get(name)
    if not raw:
        return None
    return _env_vec4(name, (1.0, 0.0, 0.0, 0.0))


def _env_float(name: str, default: float) -> float:
    raw = os.environ.get(name)
    if not raw:
        return default
    try:
        return float(raw)
    except ValueError as exc:
        raise ValueError(f"{name} must be a float, e.g. 0.05") from exc


def _env_bool(name: str, default: bool = False) -> bool:
    raw = os.environ.get(name)
    if raw is None:
        return default
    return raw.strip().lower() in {"1", "true", "yes", "on"}


def _env_float_list(name: str, default: tuple[float, ...]) -> tuple[float, ...]:
    raw = os.environ.get(name)
    if not raw:
        return default
    try:
        return tuple(float(part.strip()) for part in raw.split(",") if part.strip())
    except ValueError as exc:
        raise ValueError(f"{name} must be comma-separated floats, e.g. -0.2,-0.1,0,0.05") from exc


def _tensor_slice_to_list(tensor: torch.Tensor | None) -> list | None:
    if tensor is None:
        return None
    return tensor.detach().cpu().tolist()


def _find_usda_prim_block(text: str, prim_name: str) -> str:
    match = re.search(
        rf'(?m)^[ \t]*(?:def|over)(?:\s+(?:Cube|Sphere|Xform))?\s+"{re.escape(prim_name)}"(?:\n| \()',
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


def _read_marker_pos_from_usda_text(usd_path: Path, marker_name: str) -> tuple[float, float, float]:
    block = _find_usda_prim_block(usd_path.read_text(), marker_name)
    translate_match = re.search(r"double3 xformOp:translate = \(([^)]+)\)", block)
    if translate_match:
        values = tuple(float(part.strip()) for part in translate_match.group(1).split(","))
        if len(values) == 3:
            return values

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
            return values

    raise RuntimeError(f"Could not read xform translate for marker named {marker_name}")


def _read_marker_world_pos(usd_path: str, marker_name: str) -> tuple[float, float, float]:
    path = Path(usd_path)
    try:
        from pxr import Usd, UsdGeom

        stage = Usd.Stage.Open(str(path))
        if stage is None:
            raise RuntimeError(f"Could not open USD stage: {path}")
        prim = stage.GetPrimAtPath(f"/World/{marker_name}")
        if not prim:
            matches = [candidate for candidate in stage.Traverse() if candidate.GetName() == marker_name]
            if len(matches) == 1:
                prim = matches[0]
            elif len(matches) > 1:
                paths = ", ".join(str(candidate.GetPath()) for candidate in matches)
                raise RuntimeError(f"Found multiple marker prims named {marker_name}: {paths}")
        if not prim:
            raise RuntimeError(f"Missing marker prim named {marker_name}")
        translation = UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default()).ExtractTranslation()
        return (float(translation[0]), float(translation[1]), float(translation[2]))
    except Exception:
        return _read_marker_pos_from_usda_text(path, marker_name)


def _default_robot_init_pos_from_marker() -> tuple[float, float, float]:
    marker_pos = _read_marker_world_pos(DESK_DEBUG_MARKERS_USD_PATH, "RobotSpawnMarker")
    robot_base_offset = _env_float("LEISAAC_ROBOT_BASE_OFFSET", -0.035)
    return (marker_pos[0], marker_pos[1], marker_pos[2] + robot_base_offset)


def _front_camera_fixed_pos() -> tuple[float, float, float]:
    marker_name = os.environ.get("LEISAAC_FRONT_CAMERA_MARKER", "").strip()
    if marker_name:
        return _read_marker_world_pos(DESK_DEBUG_MARKERS_USD_PATH, marker_name)
    return _env_vec3("LEISAAC_FRONT_CAMERA_POS", FRONT_CAMERA_POS)


def _front_camera_target_pos() -> tuple[float, float, float]:
    return _env_vec3("LEISAAC_FRONT_CAMERA_TARGET_POS", FRONT_CAMERA_TARGET_POS)


def _front_camera_fixed_rot() -> tuple[float, float, float, float]:
    return _env_vec4("LEISAAC_FRONT_CAMERA_ROT", FRONT_CAMERA_ROT_ROS)


def _yaw_quat_wxyz(yaw_rad: float) -> tuple[float, float, float, float]:
    half_yaw = yaw_rad * 0.5
    return (math.cos(half_yaw), 0.0, 0.0, math.sin(half_yaw))


def _normalize_vec(vec: tuple[float, float, float]) -> tuple[float, float, float]:
    length = math.sqrt(sum(value * value for value in vec))
    if length == 0:
        raise ValueError("Cannot normalize zero-length vector")
    return tuple(value / length for value in vec)


def _quat_wxyz_mul(
    a: tuple[float, float, float, float], b: tuple[float, float, float, float]
) -> tuple[float, float, float, float]:
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return (
        aw * bw - ax * bx - ay * by - az * bz,
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
    )


def _quat_wxyz_from_euler_xyz_deg(
    rot_deg: tuple[float, float, float]
) -> tuple[float, float, float, float]:
    rx, ry, rz = (math.radians(value) for value in rot_deg)
    sx, cx = math.sin(rx * 0.5), math.cos(rx * 0.5)
    sy, cy = math.sin(ry * 0.5), math.cos(ry * 0.5)
    sz, cz = math.sin(rz * 0.5), math.cos(rz * 0.5)
    qx = (cx, sx, 0.0, 0.0)
    qy = (cy, 0.0, sy, 0.0)
    qz = (cz, 0.0, 0.0, sz)
    return _quat_wxyz_mul(qz, _quat_wxyz_mul(qy, qx))


def _quat_wxyz_to_xyzw(quat: tuple[float, float, float, float]) -> tuple[float, float, float, float]:
    return (quat[1], quat[2], quat[3], quat[0])


def _quat_xyzw_to_wxyz(quat: tuple[float, float, float, float]) -> tuple[float, float, float, float]:
    return (quat[3], quat[0], quat[1], quat[2])


def _quat_wxyz_from_axes(
    x_axis: tuple[float, float, float],
    y_axis: tuple[float, float, float],
    z_axis: tuple[float, float, float],
) -> tuple[float, float, float, float]:
    r00, r01, r02 = x_axis[0], y_axis[0], z_axis[0]
    r10, r11, r12 = x_axis[1], y_axis[1], z_axis[1]
    r20, r21, r22 = x_axis[2], y_axis[2], z_axis[2]
    trace = r00 + r11 + r22
    if trace > 0.0:
        s = math.sqrt(trace + 1.0) * 2.0
        w = 0.25 * s
        x = (r21 - r12) / s
        y = (r02 - r20) / s
        z = (r10 - r01) / s
    elif r00 > r11 and r00 > r22:
        s = math.sqrt(1.0 + r00 - r11 - r22) * 2.0
        w = (r21 - r12) / s
        x = 0.25 * s
        y = (r01 + r10) / s
        z = (r02 + r20) / s
    elif r11 > r22:
        s = math.sqrt(1.0 + r11 - r00 - r22) * 2.0
        w = (r02 - r20) / s
        x = (r01 + r10) / s
        y = 0.25 * s
        z = (r12 + r21) / s
    else:
        s = math.sqrt(1.0 + r22 - r00 - r11) * 2.0
        w = (r10 - r01) / s
        x = (r02 + r20) / s
        y = (r12 + r21) / s
        z = 0.25 * s
    length = math.sqrt(w * w + x * x + y * y + z * z)
    return (w / length, x / length, y / length, z / length)


def _robot_model_correction_quat_wxyz() -> tuple[float, float, float, float]:
    env_quat_wxyz = _optional_env_vec4("LEISAAC_ROBOT_MODEL_CORRECTION_QUAT_WXYZ")
    if env_quat_wxyz is not None:
        return env_quat_wxyz
    env_quat_xyzw = _optional_env_vec4("LEISAAC_ROBOT_MODEL_CORRECTION_QUAT_XYZW")
    if env_quat_xyzw is not None:
        return _quat_xyzw_to_wxyz(env_quat_xyzw)
    rot = _env_vec3("LEISAAC_ROBOT_MODEL_CORRECTION_ROT_XYZ_DEG", (0.0, 0.0, 0.0))
    return _quat_wxyz_from_euler_xyz_deg(rot)


def _dot(a: tuple[float, float, float], b: tuple[float, float, float]) -> float:
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def _cross(a: tuple[float, float, float], b: tuple[float, float, float]) -> tuple[float, float, float]:
    return (
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    )


def _sub(a: tuple[float, float, float], b: tuple[float, float, float]) -> tuple[float, float, float]:
    return (a[0] - b[0], a[1] - b[1], a[2] - b[2])


def _add(a: tuple[float, float, float], b: tuple[float, float, float]) -> tuple[float, float, float]:
    return (a[0] + b[0], a[1] + b[1], a[2] + b[2])


def _mul(a: tuple[float, float, float], scalar: float) -> tuple[float, float, float]:
    return (a[0] * scalar, a[1] * scalar, a[2] * scalar)


def _angle_deg(a: tuple[float, float, float], b: tuple[float, float, float]) -> float:
    a_norm = _normalize_vec(a)
    b_norm = _normalize_vec(b)
    value = max(-1.0, min(1.0, _dot(a_norm, b_norm)))
    return math.degrees(math.acos(value))


def _project_axis_to_plane(
    axis: tuple[float, float, float],
    normal: tuple[float, float, float],
    fallback: tuple[float, float, float],
) -> tuple[float, float, float]:
    projected = _sub(axis, _mul(normal, _dot(axis, normal)))
    try:
        return _normalize_vec(projected)
    except ValueError:
        projected = _sub(fallback, _mul(normal, _dot(fallback, normal)))
        return _normalize_vec(projected)


def _quat_wxyz_from_normalized_axes(
    x_axis: tuple[float, float, float],
    y_axis: tuple[float, float, float],
    z_axis: tuple[float, float, float],
) -> tuple[float, float, float, float]:
    return _quat_wxyz_from_axes(x_axis, y_axis, z_axis)


def _forward_xy_and_yaw_from_markers(
    spawn_pos: tuple[float, float, float],
    facing_pos: tuple[float, float, float] | None,
) -> tuple[tuple[float, float], float]:
    if facing_pos is None:
        raise RuntimeError("LEISAAC_ROBOT_ORIENTATION_SOURCE=facing_marker_yaw requires RobotFacingMarker")
    dx = facing_pos[0] - spawn_pos[0]
    dy = facing_pos[1] - spawn_pos[1]
    length = math.sqrt(dx * dx + dy * dy)
    if length < 1e-6:
        raise RuntimeError("RobotFacingMarker must not be at the same x/y position as RobotSpawnMarker")
    forward_xy = (dx / length, dy / length)
    return forward_xy, math.atan2(forward_xy[1], forward_xy[0])


def _robot_spawn_orientation_from_marker_frame(
    marker_frame: dict,
    correction_quat: tuple[float, float, float, float],
    facing_marker_pos: tuple[float, float, float] | None = None,
    facing_marker_frame: dict | None = None,
) -> tuple[tuple[float, float, float, float], str]:
    source = os.environ.get("LEISAAC_ROBOT_ORIENTATION_SOURCE", "marker_yaw_only").strip().lower()
    if source in {"marker_full", "marker_frame", "full_marker", "marker"}:
        base_quat = marker_frame["world_quat_wxyz"]
    elif source in {"facing_marker_orientation_yaw", "facing_orientation_yaw", "facing_marker_x_yaw"}:
        if facing_marker_frame is None:
            raise RuntimeError(
                "LEISAAC_ROBOT_ORIENTATION_SOURCE=facing_marker_orientation_yaw requires RobotFacingMarker frame"
            )
        x_axis = _project_axis_to_plane(facing_marker_frame["x_axis"], (0.0, 0.0, 1.0), (1.0, 0.0, 0.0))
        base_quat = _yaw_quat_wxyz(math.atan2(x_axis[1], x_axis[0]))
    elif source in {"spawn_to_facing_yaw", "facing_marker_yaw", "spawn_facing_yaw"}:
        _, yaw_rad = _forward_xy_and_yaw_from_markers(marker_frame["world_pos"], facing_marker_pos)
        base_quat = _yaw_quat_wxyz(yaw_rad)
    elif source in {"desk_normal_marker_x", "table_normal_marker_x", "marker_normal_marker_x"}:
        z_axis = _normalize_vec(marker_frame["z_axis"])
        x_axis = _project_axis_to_plane(marker_frame["x_axis"], z_axis, (1.0, 0.0, 0.0))
        y_axis = _normalize_vec(_cross(z_axis, x_axis))
        x_axis = _normalize_vec(_cross(y_axis, z_axis))
        base_quat = _quat_wxyz_from_normalized_axes(x_axis, y_axis, z_axis)
    elif source in {"marker_yaw_only", "yaw_only", "upright_yaw"}:
        z_axis = (0.0, 0.0, 1.0)
        x_axis = _project_axis_to_plane(marker_frame["x_axis"], z_axis, (1.0, 0.0, 0.0))
        y_axis = _normalize_vec(_cross(z_axis, x_axis))
        x_axis = _normalize_vec(_cross(y_axis, z_axis))
        base_quat = _quat_wxyz_from_normalized_axes(x_axis, y_axis, z_axis)
    elif source in {"identity", "upright", "so101_upright_identity"}:
        base_quat = (1.0, 0.0, 0.0, 0.0)
    else:
        raise ValueError(
            "Unsupported LEISAAC_ROBOT_ORIENTATION_SOURCE="
            f"{source!r}. Use facing_marker_orientation_yaw, facing_marker_yaw, marker_full, "
            "desk_normal_marker_x, marker_yaw_only, or identity."
        )
    return _quat_wxyz_mul(base_quat, correction_quat), source


def _robot_orientation_from_markers(
    spawn_pos: tuple[float, float, float],
    facing_pos: tuple[float, float, float] | None,
) -> tuple[float, float, float, float]:
    env_rot_wxyz = _optional_env_vec4("LEISAAC_ROBOT_INIT_ROT_WXYZ")
    if env_rot_wxyz is not None:
        return env_rot_wxyz
    env_rot_xyzw = _optional_env_vec4("LEISAAC_ROBOT_INIT_ROT_XYZW")
    if env_rot_xyzw is not None:
        return _quat_xyzw_to_wxyz(env_rot_xyzw)
    env_rot = _optional_env_vec4("LEISAAC_ROBOT_INIT_ROT")
    if env_rot is not None:
        return env_rot

    orientation_mode = os.environ.get("LEISAAC_ROBOT_ORIENTATION_MODE", "so101_upright_identity").strip().lower()
    if orientation_mode in {"so101_upright_identity", "identity", "upright"}:
        return (1.0, 0.0, 0.0, 0.0)

    yaw_override = os.environ.get("LEISAAC_ROBOT_YAW_DEG")
    if yaw_override:
        yaw_rad = math.radians(_env_float("LEISAAC_ROBOT_YAW_DEG", 0.0))
        return _yaw_quat_wxyz(yaw_rad)

    if facing_pos is None:
        return (1.0, 0.0, 0.0, 0.0)

    dx = facing_pos[0] - spawn_pos[0]
    dy = facing_pos[1] - spawn_pos[1]
    if abs(dx) < 1e-6 and abs(dy) < 1e-6:
        return (1.0, 0.0, 0.0, 0.0)

    yaw_rad = math.atan2(dy, dx) + math.radians(_env_float("LEISAAC_ROBOT_YAW_OFFSET_DEG", 0.0))
    return _yaw_quat_wxyz(yaw_rad)


def _default_robot_init_rot_from_markers() -> tuple[float, float, float, float]:
    marker_pos = _read_marker_world_pos(DESK_DEBUG_MARKERS_USD_PATH, "RobotSpawnMarker")
    try:
        facing_pos = _read_marker_world_pos(DESK_DEBUG_MARKERS_USD_PATH, "RobotFacingMarker")
    except Exception:
        facing_pos = None
    return _robot_orientation_from_markers(marker_pos, facing_pos)


def _print_robot_correction_candidates() -> None:
    candidates = (
        (0.0, 0.0, 0.0),
        (90.0, 0.0, 0.0),
        (-90.0, 0.0, 0.0),
        (0.0, 90.0, 0.0),
        (0.0, -90.0, 0.0),
        (0.0, 0.0, 90.0),
        (0.0, 0.0, -90.0),
        (180.0, 0.0, 0.0),
        (0.0, 180.0, 0.0),
        (0.0, 0.0, 180.0),
    )
    print("[DEBUG] robot_model_correction_rotation candidates:")
    for candidate in candidates:
        print(
            "[DEBUG]   "
            f"LEISAAC_ROBOT_MODEL_CORRECTION_ROT_XYZ_DEG="
            f"{candidate[0]:.0f},{candidate[1]:.0f},{candidate[2]:.0f} "
            f"quat_wxyz={_quat_wxyz_from_euler_xyz_deg(candidate)}"
        )


def _runtime_marker_transform(env, env_id: int, marker_name: str) -> tuple[str, tuple[float, float, float] | None, tuple[float, float, float]]:
    from isaacsim.core.utils.stage import get_current_stage
    from pxr import Usd, UsdGeom

    stage = get_current_stage()
    prim_paths = []
    if hasattr(env.scene, "env_prim_paths"):
        prim_paths.append(f"{env.scene.env_prim_paths[env_id]}/DebugMarkers/{marker_name}")
        prim_paths.append(f"{env.scene.env_prim_paths[env_id]}/Scene/{marker_name}")
    prim_paths.append(f"/World/envs/env_{env_id}/DebugMarkers/{marker_name}")
    prim_paths.append(f"/World/envs/env_{env_id}/Scene/{marker_name}")

    prim = None
    resolved_prim_path = None
    for prim_path in prim_paths:
        candidate = stage.GetPrimAtPath(prim_path)
        if candidate:
            prim = candidate
            resolved_prim_path = prim_path
            break
    if not prim:
        matches = [candidate for candidate in stage.Traverse() if candidate.GetName() == marker_name]
        if len(matches) == 1:
            prim = matches[0]
            resolved_prim_path = str(prim.GetPath())
        elif len(matches) > 1:
            paths = ", ".join(str(candidate.GetPath()) for candidate in matches)
            raise RuntimeError(f"Found multiple marker prims named {marker_name}: {paths}")
    if not prim:
        raise RuntimeError(f"Could not find {marker_name} in current Isaac stage")

    local_pos = None
    for op in UsdGeom.Xformable(prim).GetOrderedXformOps():
        if op.GetOpName() == "xformOp:translate":
            value = op.Get()
            local_pos = (float(value[0]), float(value[1]), float(value[2]))
            break
    translation = UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default()).ExtractTranslation()
    world_pos = (float(translation[0]), float(translation[1]), float(translation[2]))
    return resolved_prim_path or str(prim.GetPath()), local_pos, world_pos


def _runtime_marker_world_pos(env, env_id: int, marker_name: str) -> tuple[float, float, float]:
    return _runtime_marker_transform(env, env_id, marker_name)[2]


def _runtime_marker_frame(env, env_id: int, marker_name: str) -> dict:
    from isaacsim.core.utils.stage import get_current_stage
    from pxr import Gf, Usd, UsdGeom

    stage = get_current_stage()
    prim_path, local_pos, _ = _runtime_marker_transform(env, env_id, marker_name)
    prim = stage.GetPrimAtPath(prim_path)
    xformable = UsdGeom.Xformable(prim)
    matrix = xformable.ComputeLocalToWorldTransform(Usd.TimeCode.Default())
    translation = matrix.ExtractTranslation()
    origin = matrix.Transform(Gf.Vec3d(0.0, 0.0, 0.0))
    x_axis_raw = matrix.Transform(Gf.Vec3d(1.0, 0.0, 0.0)) - origin
    y_axis_raw = matrix.Transform(Gf.Vec3d(0.0, 1.0, 0.0)) - origin
    z_axis_raw = matrix.Transform(Gf.Vec3d(0.0, 0.0, 1.0)) - origin
    x_axis = _normalize_vec((float(x_axis_raw[0]), float(x_axis_raw[1]), float(x_axis_raw[2])))
    y_axis = _normalize_vec((float(y_axis_raw[0]), float(y_axis_raw[1]), float(y_axis_raw[2])))
    z_axis = _normalize_vec((float(z_axis_raw[0]), float(z_axis_raw[1]), float(z_axis_raw[2])))
    world_quat_wxyz = _quat_wxyz_from_axes(x_axis, y_axis, z_axis)
    world_quat_xyzw = (world_quat_wxyz[1], world_quat_wxyz[2], world_quat_wxyz[3], world_quat_wxyz[0])
    local_orientation = None
    for op in xformable.GetOrderedXformOps():
        if op.GetOpName() == "xformOp:rotateXYZ":
            value = op.Get()
            local_orientation = (float(value[0]), float(value[1]), float(value[2]))
        elif op.GetOpName() == "xformOp:orient":
            value = op.Get()
            local_imag = value.GetImaginary()
            local_orientation = (
                float(value.GetReal()),
                float(local_imag[0]),
                float(local_imag[1]),
                float(local_imag[2]),
            )
    return {
        "prim_path": prim_path,
        "local_pos": local_pos,
        "local_orientation": local_orientation,
        "world_pos": (float(translation[0]), float(translation[1]), float(translation[2])),
        "world_quat_wxyz": world_quat_wxyz,
        "world_quat_xyzw": world_quat_xyzw,
        "x_axis": x_axis,
        "y_axis": y_axis,
        "z_axis": z_axis,
        "matrix": matrix,
    }


def _runtime_optional_marker_world_pos(env, env_id: int, marker_name: str) -> tuple[float, float, float] | None:
    try:
        return _runtime_marker_world_pos(env, env_id, marker_name)
    except RuntimeError:
        return None


def _runtime_optional_marker_frame(env, env_id: int, marker_name: str) -> dict | None:
    try:
        return _runtime_marker_frame(env, env_id, marker_name)
    except RuntimeError:
        return None


def _runtime_prim_path(env, env_id: int, suffix: str) -> str:
    if hasattr(env.scene, "env_prim_paths"):
        return f"{env.scene.env_prim_paths[env_id]}/{suffix}"
    return f"/World/envs/env_{env_id}/{suffix}"


def _debug_tabletop_collider(env, env_id: int) -> tuple[float, float, float] | None:
    from isaacsim.core.utils.stage import get_current_stage
    from pxr import Usd, UsdGeom

    stage = get_current_stage()
    prim_path = _runtime_prim_path(env, env_id, "Scene/DeskTabletopCollider")
    prim = stage.GetPrimAtPath(prim_path)
    if not prim:
        print(f"[DEBUG] DeskTabletopCollider path missing env_{env_id}: {prim_path}")
        return None

    translation = UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default()).ExtractTranslation()
    world_pos = (float(translation[0]), float(translation[1]), float(translation[2]))
    collision_attr = prim.GetAttribute("physics:collisionEnabled")
    collision_enabled = collision_attr.Get() if collision_attr else None
    api_names = [str(api) for api in prim.GetAppliedSchemas()]
    print(f"[DEBUG] DeskTabletopCollider prim path env_{env_id}: {prim_path}")
    print(f"[DEBUG] DeskTabletopCollider world position env_{env_id}: {world_pos}")
    print(f"[DEBUG] DeskTabletopCollider collisionEnabled env_{env_id}: {collision_enabled}")
    print(f"[DEBUG] DeskTabletopCollider applied schemas env_{env_id}: {api_names}")
    return world_pos


def _robot_root_data_pose(robot, env_ids: torch.Tensor) -> tuple[list | None, list | None]:
    root_pos = getattr(robot.data, "root_pos_w", None)
    root_quat = getattr(robot.data, "root_quat_w", None)
    if root_pos is None or root_quat is None:
        root_state = getattr(robot.data, "root_state_w", None)
        if root_state is not None:
            return _tensor_slice_to_list(root_state[env_ids, :3]), _tensor_slice_to_list(root_state[env_ids, 3:7])
    return _tensor_slice_to_list(root_pos[env_ids] if root_pos is not None else None), _tensor_slice_to_list(
        root_quat[env_ids] if root_quat is not None else None
    )


def _robot_bbox_min_z(env, env_id: int, robot_prim_path: str) -> float | None:
    return _asset_bbox_min_z(env, env_id, robot_prim_path)


def _asset_bbox_min_z(env, env_id: int, prim_path: str) -> float | None:
    try:
        from isaacsim.core.utils.stage import get_current_stage
        from pxr import Usd, UsdGeom

        stage = get_current_stage()
        prim = stage.GetPrimAtPath(prim_path)
        if not prim:
            return None
        bbox_cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_, UsdGeom.Tokens.render, UsdGeom.Tokens.proxy])
        bbox = bbox_cache.ComputeWorldBound(prim).ComputeAlignedBox()
        if bbox.IsEmpty():
            return None
        return float(bbox.GetMin()[2])
    except Exception as exc:
        print(f"[DEBUG] bbox read failed env_{env_id} prim={prim_path}: {exc}")
        return None


def _asset_bbox_min_signed_distance_to_plane(
    env,
    env_id: int,
    prim_path: str,
    plane_point: tuple[float, float, float],
    plane_normal: tuple[float, float, float],
) -> float | None:
    try:
        from isaacsim.core.utils.stage import get_current_stage
        from pxr import Gf, Usd, UsdGeom

        stage = get_current_stage()
        prim = stage.GetPrimAtPath(prim_path)
        if not prim:
            return None
        bbox_cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_, UsdGeom.Tokens.render, UsdGeom.Tokens.proxy])
        bbox = bbox_cache.ComputeWorldBound(prim).ComputeAlignedBox()
        if bbox.IsEmpty():
            return None
        min_pt = bbox.GetMin()
        max_pt = bbox.GetMax()
        corners = [
            (x, y, z)
            for x in (float(min_pt[0]), float(max_pt[0]))
            for y in (float(min_pt[1]), float(max_pt[1]))
            for z in (float(min_pt[2]), float(max_pt[2]))
        ]
        return min(_dot(_sub(corner, plane_point), plane_normal) for corner in corners)
    except Exception as exc:
        print(f"[DEBUG] bbox plane distance read failed env_{env_id} prim={prim_path}: {exc}")
        return None


def _table_surface_z(env, env_id: int, fallback_marker_name: str = "RobotSpawnMarker") -> float:
    env_surface_z = os.environ.get("LEISAAC_TABLE_SURFACE_Z")
    if env_surface_z:
        return _env_float("LEISAAC_TABLE_SURFACE_Z", 0.0)

    query_pos = _runtime_marker_world_pos(env, env_id, fallback_marker_name)
    surface_source = os.environ.get("LEISAAC_TABLE_SURFACE_SOURCE", "collider").strip().lower()
    surface_marker_name = os.environ.get("LEISAAC_TABLE_SURFACE_MARKER", "").strip()
    if surface_marker_name:
        marker_pos = _runtime_marker_world_pos(env, env_id, surface_marker_name)
        return marker_pos[2]

    if surface_source in {"collider", "desk_tabletop_collider"}:
        try:
            from isaacsim.core.utils.stage import get_current_stage
            from pxr import Gf, Usd, UsdGeom

            stage = get_current_stage()
            prim = stage.GetPrimAtPath(_runtime_prim_path(env, env_id, "Scene/DeskTabletopCollider"))
            if prim:
                xformable = UsdGeom.Xformable(prim)
                matrix = xformable.ComputeLocalToWorldTransform(Usd.TimeCode.Default())
                top_center = matrix.Transform(Gf.Vec3d(0.0, 0.0, 0.5))
                normal_end = matrix.Transform(Gf.Vec3d(0.0, 0.0, 1.5))
                normal = normal_end - top_center
                if abs(float(normal[2])) > 1e-6:
                    dz = -(
                        float(normal[0]) * (query_pos[0] - float(top_center[0]))
                        + float(normal[1]) * (query_pos[1] - float(top_center[1]))
                    ) / float(normal[2])
                    return float(top_center[2]) + dz
        except Exception as exc:
            print(f"[DEBUG] DeskTabletopCollider surface z unavailable env_{env_id}: {exc}")

    # Marker fallback only. Do not fit a tilted plane from noisy desk corner
    # markers for spawn heights.
    return query_pos[2]


def _print_z_offset_sweep(marker_world_pos: tuple[float, float, float]) -> None:
    offsets = _env_float_list(
        "LEISAAC_ROBOT_Z_OFFSET_SWEEP_VALUES",
        (-0.20, -0.15, -0.10, -0.05, 0.00, 0.03, 0.05, 0.08, 0.10, 0.15),
    )
    print("[DEBUG] Robot z-offset sweep candidates:")
    for offset in offsets:
        print(
            "[DEBUG]   "
            f"offset={offset:.3f} -> desired_robot_root_pos="
            f"({marker_world_pos[0]:.6f}, {marker_world_pos[1]:.6f}, {marker_world_pos[2] + offset:.6f})"
        )


def reset_robot_to_spawn_marker(
    env,
    env_ids: torch.Tensor,
    robot_cfg: SceneEntityCfg = SceneEntityCfg("robot"),
    marker_name: str = "RobotSpawnMarker",
    facing_marker_name: str = "RobotFacingMarker",
    robot_base_offset: float = -0.035,
) -> None:
    robot = env.scene[robot_cfg.name]
    if env_ids is None:
        env_ids = torch.arange(env.scene.num_envs, device=env.device)
    else:
        env_ids = env_ids.to(device=env.device)
    root_pose = robot.data.default_root_state[env_ids, :7].clone()
    default_root_pose = root_pose.clone()
    root_before_pos, root_before_quat = _robot_root_data_pose(robot, env_ids)
    print(f"[DEBUG] IsaacLab robot entity name: {robot_cfg.name}")
    print(f"[DEBUG] IsaacLab robot cfg prim_path: {getattr(getattr(robot, 'cfg', None), 'prim_path', None)}")
    print(f"[DEBUG] robot root pose before reset data pos={root_before_pos} quat={root_before_quat}")

    for row, env_id_tensor in enumerate(env_ids):
        env_id = int(env_id_tensor.item())
        marker_frame = _runtime_marker_frame(env, env_id, marker_name)
        marker_prim_path = marker_frame["prim_path"]
        marker_local_pos = marker_frame["local_pos"]
        marker_pos = marker_frame["world_pos"]
        marker_z_axis = marker_frame["z_axis"]
        facing_marker_pos = _runtime_optional_marker_world_pos(env, env_id, facing_marker_name)
        facing_marker_frame = _runtime_optional_marker_frame(env, env_id, facing_marker_name)
        robot_root_z_offset = _env_float("LEISAAC_ROBOT_ROOT_Z_OFFSET", robot_base_offset)
        spawn_pos = _add(marker_pos, _mul(marker_z_axis, robot_root_z_offset))
        marker_yaw_rad = math.atan2(marker_frame["x_axis"][1], marker_frame["x_axis"][0])
        forward_xy = None
        marker_based_yaw_rad = None
        if facing_marker_pos is not None:
            forward_xy, marker_based_yaw_rad = _forward_xy_and_yaw_from_markers(marker_pos, facing_marker_pos)
        facing_marker_orientation_yaw_rad = None
        if facing_marker_frame is not None:
            facing_x_axis_xy = _project_axis_to_plane(
                facing_marker_frame["x_axis"], (0.0, 0.0, 1.0), (1.0, 0.0, 0.0)
            )
            facing_marker_orientation_yaw_rad = math.atan2(facing_x_axis_xy[1], facing_x_axis_xy[0])
        correction_quat_wxyz = _robot_model_correction_quat_wxyz()
        robot_orientation_wxyz, orientation_source = _robot_spawn_orientation_from_marker_frame(
            marker_frame, correction_quat_wxyz, facing_marker_pos, facing_marker_frame
        )
        robot_orientation_xyzw_debug = _quat_wxyz_to_xyzw(robot_orientation_wxyz)
        desired_robot_root_pose = (*spawn_pos, *robot_orientation_wxyz)
        env_origin = env.scene.env_origins[env_id].detach().cpu()
        default_pos = (spawn_pos[0] - float(env_origin[0]), spawn_pos[1] - float(env_origin[1]), spawn_pos[2] - float(env_origin[2]))
        root_pose[row, :3] = torch.tensor(spawn_pos, device=env.device, dtype=root_pose.dtype)
        root_pose[row, 3:7] = torch.tensor(robot_orientation_wxyz, device=env.device, dtype=root_pose.dtype)
        default_root_pose[row, :3] = torch.tensor(default_pos, device=env.device, dtype=default_root_pose.dtype)
        default_root_pose[row, 3:7] = torch.tensor(robot_orientation_wxyz, device=env.device, dtype=default_root_pose.dtype)
        robot_prim_path = _runtime_prim_path(env, env_id, "Robot")
        print(f"[DEBUG] Robot articulation root prim path env_{env_id}: {robot_prim_path}")
        print(f"[DEBUG] {marker_name} prim path env_{env_id}: {marker_prim_path}")
        print(f"[DEBUG] {marker_name} local position env_{env_id}: {marker_local_pos}")
        print(f"[DEBUG] {marker_name} local orientation env_{env_id}: {marker_frame['local_orientation']}")
        print(f"[DEBUG] {marker_name} world position env_{env_id}: {marker_pos}")
        print(f"[DEBUG] {marker_name} world pos = {marker_pos}")
        print(f"[DEBUG] {facing_marker_name} world position env_{env_id}: {facing_marker_pos}")
        print(f"[DEBUG] {facing_marker_name} world pos = {facing_marker_pos}")
        if facing_marker_frame is not None:
            print(f"[DEBUG] {facing_marker_name} world quaternion_wxyz env_{env_id}: {facing_marker_frame['world_quat_wxyz']}")
            print(f"[DEBUG] {facing_marker_name} x_axis_world env_{env_id}: {facing_marker_frame['x_axis']}")
        print(f"[DEBUG] {marker_name} world quaternion_wxyz env_{env_id}: {marker_frame['world_quat_wxyz']}")
        print(f"[DEBUG] {marker_name} world quaternion_xyzw env_{env_id}: {marker_frame['world_quat_xyzw']}")
        print(f"[DEBUG] marker_x_axis_world env_{env_id}: {marker_frame['x_axis']}")
        print(f"[DEBUG] marker_y_axis_world env_{env_id}: {marker_frame['y_axis']}")
        print(f"[DEBUG] marker_z_axis_world env_{env_id}: {marker_frame['z_axis']}")
        print(f"[DEBUG] marker_z_axis_angle_to_world_z_deg env_{env_id}: {_angle_deg(marker_frame['z_axis'], (0.0, 0.0, 1.0))}")
        print(f"[DEBUG] marker_yaw_rad env_{env_id}: {marker_yaw_rad}")
        print(f"[DEBUG] forward_xy env_{env_id}: {forward_xy}")
        print(f"[DEBUG] marker_based_yaw_rad env_{env_id}: {marker_based_yaw_rad}")
        print(
            f"[DEBUG] marker_based_yaw_deg env_{env_id}: "
            f"{math.degrees(marker_based_yaw_rad) if marker_based_yaw_rad is not None else None}"
        )
        print(f"[DEBUG] facing_marker_orientation_yaw_rad env_{env_id}: {facing_marker_orientation_yaw_rad}")
        print(
            f"[DEBUG] facing_marker_orientation_yaw_deg env_{env_id}: "
            f"{math.degrees(facing_marker_orientation_yaw_rad) if facing_marker_orientation_yaw_rad is not None else None}"
        )
        print(f"[DEBUG] robot_root_z_offset env_{env_id}: {robot_root_z_offset}")
        print(f"[DEBUG] robot_orientation_source env_{env_id}: {orientation_source}")
        print(f"[DEBUG] quaternion_convention env_{env_id}: wxyz")
        print(f"[DEBUG] write_root_pose_to_sim expects env_{env_id}: wxyz")
        print(f"[DEBUG] robot_model_correction_rotation_wxyz env_{env_id}: {correction_quat_wxyz}")
        if _env_bool("LEISAAC_PRINT_ROBOT_CORRECTION_CANDIDATES", False):
            _print_robot_correction_candidates()
        print(f"[DEBUG] desired robot root position from marker env_{env_id}: {spawn_pos}")
        print(f"[DEBUG] desired_robot_root_pose env_{env_id}: {desired_robot_root_pose}")
        print(f"[DEBUG] quat_xyzw_from_library_style env_{env_id}: {robot_orientation_xyzw_debug}")
        print(f"[DEBUG] quat_wxyz_to_isaaclab env_{env_id}: {robot_orientation_wxyz}")
        print(f"[DEBUG] final_robot_quat_written env_{env_id}: {robot_orientation_wxyz}")
        if _env_bool("LEISAAC_ROBOT_Z_OFFSET_SWEEP", False):
            _print_z_offset_sweep(marker_pos)
        _debug_tabletop_collider(env, env_id)

    root_velocity = torch.zeros((len(env_ids), 6), device=env.device, dtype=root_pose.dtype)
    robot.data.default_root_state[env_ids, :7] = default_root_pose
    robot.data.default_root_state[env_ids, 7:] = root_velocity
    robot.write_root_pose_to_sim(root_pose, env_ids=env_ids)
    robot.write_root_velocity_to_sim(root_velocity, env_ids=env_ids)
    print(f"[DEBUG] Robot root pose written during reset: {root_pose.detach().cpu().tolist()}")
    root_after_pos, root_after_quat = _robot_root_data_pose(robot, env_ids)
    print(f"[DEBUG] robot root pose after reset = pos={root_after_pos} quat={root_after_quat}")
    print(f"[DEBUG] robot root pose after reset data pos={root_after_pos} quat={root_after_quat}")

    if _env_bool("LEISAAC_ROBOT_AUTO_BBOX_OFFSET", False):
        corrected_root_pose = root_pose.clone()
        corrected_default_root_pose = default_root_pose.clone()
        corrected = False
        for row, env_id_tensor in enumerate(env_ids):
            env_id = int(env_id_tensor.item())
            robot_prim_path = _runtime_prim_path(env, env_id, "Robot")
            marker_frame = _runtime_marker_frame(env, env_id, marker_name)
            signed_error = _asset_bbox_min_signed_distance_to_plane(
                env,
                env_id,
                robot_prim_path,
                marker_frame["world_pos"],
                marker_frame["z_axis"],
            )
            if signed_error is None:
                print(f"[DEBUG] robot bbox signed distance env_{env_id}: unavailable")
                continue
            corrected_root_pos = _sub(
                tuple(float(value) for value in corrected_root_pose[row, :3].detach().cpu().tolist()),
                _mul(marker_frame["z_axis"], signed_error),
            )
            env_origin = env.scene.env_origins[env_id].detach().cpu()
            corrected_root_pose[row, :3] = torch.tensor(corrected_root_pos, device=env.device, dtype=corrected_root_pose.dtype)
            corrected_default_root_pose[row, :3] = torch.tensor(
                (
                    corrected_root_pos[0] - float(env_origin[0]),
                    corrected_root_pos[1] - float(env_origin[1]),
                    corrected_root_pos[2] - float(env_origin[2]),
                ),
                device=env.device,
                dtype=corrected_default_root_pose.dtype,
            )
            corrected = True
            print(f"[DEBUG] bbox signed_error env_{env_id}: {signed_error}")
            print(f"[DEBUG] desired robot root position after bbox correction env_{env_id}: {corrected_root_pos}")
        if corrected:
            robot.data.default_root_state[env_ids, :7] = corrected_default_root_pose
            robot.write_root_pose_to_sim(corrected_root_pose, env_ids=env_ids)
            root_pose = corrected_root_pose
            print(f"[DEBUG] Robot root pose after bbox correction: {root_pose.detach().cpu().tolist()}")
            root_corrected_pos, root_corrected_quat = _robot_root_data_pose(robot, env_ids)
            print(f"[DEBUG] robot root pose after bbox correction data pos={root_corrected_pos} quat={root_corrected_quat}")
    else:
        print("[DEBUG] Robot bbox correction disabled; robot root remains at RobotSpawnMarker pose.")


def reset_objects_to_spawn_markers(
    env,
    env_ids: torch.Tensor,
    object_marker_names: dict[str, str] = OBJECT_SPAWN_MARKERS,
) -> None:
    if env_ids is None:
        env_ids = torch.arange(env.scene.num_envs, device=env.device)
    else:
        env_ids = env_ids.to(device=env.device)

    for object_name, marker_name in object_marker_names.items():
        asset = env.scene[object_name]
        root_pose = asset.data.default_root_state[env_ids, :7].clone()
        default_root_pose = root_pose.clone()
        surface_z_by_env: dict[int, float] = {}

        for row, env_id_tensor in enumerate(env_ids):
            env_id = int(env_id_tensor.item())
            marker_pos = _runtime_marker_world_pos(env, env_id, marker_name)
            surface_z = _table_surface_z(env, env_id, marker_name)
            surface_z_by_env[env_id] = surface_z
            initial_pos = (marker_pos[0], marker_pos[1], surface_z + _env_float(f"LEISAAC_{object_name.upper()}_ROOT_Z_OFFSET", 0.0))
            env_origin = env.scene.env_origins[env_id].detach().cpu()
            default_pos = (
                initial_pos[0] - float(env_origin[0]),
                initial_pos[1] - float(env_origin[1]),
                initial_pos[2] - float(env_origin[2]),
            )
            root_pose[row, :3] = torch.tensor(initial_pos, device=env.device, dtype=root_pose.dtype)
            default_root_pose[row, :3] = torch.tensor(default_pos, device=env.device, dtype=default_root_pose.dtype)
            print(f"[DEBUG] {marker_name} world position env_{env_id}: {marker_pos}")
            print(f"[DEBUG] {marker_name} world pos = {marker_pos}")
            print(f"[DEBUG] table_plane_surface_z env_{env_id}: {surface_z}")
            print(f"[DEBUG] Applying {object_name} spawn x/y from marker and horizontal z env_{env_id}: {initial_pos}")

        root_velocity = torch.zeros((len(env_ids), 6), device=env.device, dtype=root_pose.dtype)
        asset.data.default_root_state[env_ids, :7] = default_root_pose
        asset.data.default_root_state[env_ids, 7:] = root_velocity
        asset.write_root_pose_to_sim(root_pose, env_ids=env_ids)
        asset.write_root_velocity_to_sim(root_velocity, env_ids=env_ids)

        if _env_bool("LEISAAC_OBJECT_AUTO_BBOX_OFFSET", False):
            corrected_root_pose = root_pose.clone()
            corrected_default_root_pose = default_root_pose.clone()
            corrected = False
            for row, env_id_tensor in enumerate(env_ids):
                env_id = int(env_id_tensor.item())
                object_prim_path = _runtime_prim_path(env, env_id, f"Scene/{object_name}")
                bbox_min_z = _asset_bbox_min_z(env, env_id, object_prim_path)
                if bbox_min_z is None:
                    print(f"[DEBUG] {object_name} bbox min_z env_{env_id}: unavailable")
                    continue
                surface_z = surface_z_by_env[env_id]
                error = bbox_min_z - surface_z
                corrected_root_z = float(corrected_root_pose[row, 2]) - error
                env_origin = env.scene.env_origins[env_id].detach().cpu()
                corrected_root_pose[row, 2] = corrected_root_z
                corrected_default_root_pose[row, 2] = corrected_root_z - float(env_origin[2])
                corrected = True
                print(f"[DEBUG] {object_name} bbox min_z env_{env_id}: {bbox_min_z}")
                print(f"[DEBUG] {object_name} bbox_table_error env_{env_id}: {error}")
                print(f"[DEBUG] {object_name} bbox_corrected_root_z env_{env_id}: {corrected_root_z}")
            if corrected:
                asset.data.default_root_state[env_ids, :7] = corrected_default_root_pose
                asset.write_root_pose_to_sim(corrected_root_pose, env_ids=env_ids)
                root_pose = corrected_root_pose

        print(f"[DEBUG] {object_name} root pose after reset: {root_pose.detach().cpu().tolist()}")


def reset_front_camera_to_fixed_pose(
    env,
    env_ids: torch.Tensor,
    camera_cfg: SceneEntityCfg = SceneEntityCfg("front"),
) -> None:
    camera = env.scene[camera_cfg.name]
    if env_ids is None:
        env_ids = torch.arange(env.scene.num_envs, device=env.device)
    else:
        env_ids = env_ids.to(device=env.device)

    base_position = torch.tensor(_front_camera_fixed_pos(), device=env.device, dtype=camera.data.pos_w.dtype)
    positions = base_position.repeat(len(env_ids), 1)
    if hasattr(env.scene, "env_origins"):
        positions = positions + env.scene.env_origins[env_ids].to(device=env.device, dtype=positions.dtype)

    base_target = torch.tensor(_front_camera_target_pos(), device=env.device, dtype=camera.data.pos_w.dtype)
    targets = base_target.repeat(len(env_ids), 1)
    if hasattr(env.scene, "env_origins"):
        targets = targets + env.scene.env_origins[env_ids].to(device=env.device, dtype=targets.dtype)

    camera.set_world_poses_from_view(positions, targets, env_ids)
    print(
        "[DEBUG] front camera fixed pose reset: "
        f"pos={positions.detach().cpu().tolist()} "
        f"target={targets.detach().cpu().tolist()}"
    )


@configclass
class DeskPickOrangeSceneCfg(SingleArmTaskSceneCfg):
    """Scene configuration for the desk pick orange task."""

    scene: AssetBaseCfg = DESK_WITH_ORANGE_CFG.replace(prim_path="{ENV_REGEX_NS}/Scene")
    debug_markers: AssetBaseCfg = DESK_DEBUG_MARKERS_CFG.replace(prim_path="{ENV_REGEX_NS}/DebugMarkers")


@configclass
class DeskPickOrangeEventCfg(SingleArmEventCfg):
    """Reset events for the desk pick orange task."""

    reset_robot_to_spawn_marker = EventTerm(
        func=reset_robot_to_spawn_marker,
        mode="reset",
        params={"robot_cfg": SceneEntityCfg("robot")},
    )

    reset_objects_to_spawn_markers = EventTerm(
        func=reset_objects_to_spawn_markers,
        mode="reset",
    )

    reset_front_camera_to_fixed_pose = EventTerm(
        func=reset_front_camera_to_fixed_pose,
        mode="reset",
        params={"camera_cfg": SceneEntityCfg("front")},
    )


@configclass
class ObservationsCfg(SingleArmObservationsCfg):

    @configclass
    class SubtaskCfg(ObsGroup):
        """Observations for subtask group."""

        pick_orange001 = ObsTerm(func=mdp.orange_grasped, params={"object_cfg": SceneEntityCfg("Orange001")})
        put_orange001_to_plate = ObsTerm(
            func=mdp.put_orange_to_plate,
            params={"object_cfg": SceneEntityCfg("Orange001"), "plate_cfg": SceneEntityCfg("Plate")},
        )
        pick_orange003 = ObsTerm(func=mdp.orange_grasped, params={"object_cfg": SceneEntityCfg("Orange003")})
        put_orange003_to_plate = ObsTerm(
            func=mdp.put_orange_to_plate,
            params={"object_cfg": SceneEntityCfg("Orange003"), "plate_cfg": SceneEntityCfg("Plate")},
        )

        def __post_init__(self):
            self.enable_corruption = False
            self.concatenate_terms = False

    subtask_terms: SubtaskCfg = SubtaskCfg()


@configclass
class TerminationsCfg(SingleArmTerminationsCfg):

    success = DoneTerm(
        func=mdp.task_done,
        params={
            "oranges_cfg": [SceneEntityCfg("Orange001"), SceneEntityCfg("Orange003")],
            "plate_cfg": SceneEntityCfg("Plate"),
        },
    )


@configclass
class DeskPickOrangeEnvCfg(SingleArmTaskEnvCfg):
    """Configuration for the desk pick orange environment."""

    scene: DeskPickOrangeSceneCfg = DeskPickOrangeSceneCfg(env_spacing=8.0)

    events: DeskPickOrangeEventCfg = DeskPickOrangeEventCfg()

    observations: ObservationsCfg = ObservationsCfg()

    terminations: TerminationsCfg = TerminationsCfg()

    task_description: str = "Pick the red orange and the orange from the desk and put them into the plate, then reset the arm to rest state."

    def __post_init__(self) -> None:
        super().__post_init__()

        self.viewer.eye = (2.573745, 1.869797, 0.104041)
        self.viewer.lookat = (1.423745, 3.069797, -0.845959)
        self.scene.front.prim_path = "{ENV_REGEX_NS}/Scene/front_camera"
        self.scene.front.offset = TiledCameraCfg.OffsetCfg(
            pos=_front_camera_fixed_pos(),
            rot=_front_camera_fixed_rot(),
            convention="ros",
        )

        self.scene.robot.init_state.pos = _env_vec3("LEISAAC_ROBOT_INIT_POS", _default_robot_init_pos_from_marker())
        self.scene.robot.init_state.rot = _default_robot_init_rot_from_markers()
        print(f"[LeIsaac] Desk robot init pos: {self.scene.robot.init_state.pos}")
        print(f"[LeIsaac] Desk robot init rot: {self.scene.robot.init_state.rot}")

        parse_usd_and_create_subassets(
            DESK_WITH_ORANGE_USD_PATH, self, specific_name_list=["Orange001", "Orange003", "Plate"]
        )
