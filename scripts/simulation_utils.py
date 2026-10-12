"""Shared standalone Isaac Sim startup helpers."""

from __future__ import annotations

import math
import os
import sys
import time
from collections import deque
from functools import lru_cache
from pathlib import Path


def bootstrap_repo_path(script_path: str | Path) -> None:
    repo_root = Path(script_path).resolve().parents[1]
    if str(repo_root) not in sys.path:
        sys.path.insert(0, str(repo_root))


def prefer_internal_ros_libraries() -> None:
    """Remove sourced ROS overlays so Isaac can load its bundled ROS libraries."""
    ros_markers = ("/opt/ros/", "/ros/", "site-packages/rclpy", "_rclpy_pybind11")
    for key in ("ROS_DISTRO", "AMENT_PREFIX_PATH", "CMAKE_PREFIX_PATH", "COLCON_PREFIX_PATH"):
        os.environ.pop(key, None)

    for key in ("PYTHONPATH", "LD_LIBRARY_PATH"):
        if key not in os.environ:
            continue
        os.environ[key] = os.pathsep.join(
            path
            for path in os.environ[key].split(os.pathsep)
            if not any(marker in path for marker in ros_markers)
        )
    os.environ.setdefault("RMW_IMPLEMENTATION", "rmw_fastrtps_cpp")


def append_internal_ros_libs_from_isaacsim_path() -> None:
    configured_root = os.environ.get("ISAACSIM_PATH") or os.environ.get(
        "isaac_sim_package_path"
    )
    if configured_root:
        root = Path(configured_root)
    else:
        # Under isaacsim/python.sh this resolves from
        # <isaacsim>/kit/python/bin/python3 without requiring shell variables.
        executable = Path(sys.executable).resolve()
        root = executable.parents[3] if len(executable.parents) > 3 else Path()
    candidates = (
        root / "exts/isaacsim.ros2.core/humble/lib",
        root / "exts/isaacsim.ros2.bridge/humble/lib",
    )
    existing = [str(path) for path in candidates if path.exists()]
    current = os.environ.get("LD_LIBRARY_PATH")
    if current:
        existing.append(current)
    if existing:
        os.environ["LD_LIBRARY_PATH"] = os.pathsep.join(existing)


def _append_internal_rclpy_path() -> None:
    """Expose bundled rclpy after the ROS bridge has finished initializing."""
    configured_root = os.environ.get("ISAACSIM_PATH") or os.environ.get(
        "isaac_sim_package_path"
    )
    if configured_root:
        root = Path(configured_root)
    else:
        executable = Path(sys.executable).resolve()
        root = executable.parents[3] if len(executable.parents) > 3 else Path()
    bundled_rclpy = root / "exts/isaacsim.ros2.core/humble/rclpy"
    if bundled_rclpy.is_dir() and str(bundled_rclpy) not in sys.path:
        sys.path.insert(0, str(bundled_rclpy))


def create_simulation_app(headless: bool):
    from isaacsim.simulation_app import SimulationApp

    return SimulationApp({"headless": headless})


def configure_independent_physics_rate(
    world, physics_dt: float, rendering_dt: float
) -> int:
    """Keep fixed-rate physics able to catch up independently of rendering."""
    if physics_dt <= 0.0 or rendering_dt <= 0.0:
        raise ValueError("physics_dt and rendering_dt must be positive")

    import carb

    # Non-divisible rates need the ceiling so, for example, 200/60 can use
    # three or four physics steps per rendered frame instead of being capped at three.
    maximum_substeps = max(1, math.ceil(rendering_dt / physics_dt))
    carb.settings.get_settings().set_bool("/app/player/useFixedTimeStepping", True)
    world.get_physics_context().set_physics_dt(physics_dt, maximum_substeps)
    return maximum_substeps


class SimulationRateTracker:
    """Measure rendered FPS and simulation-time progress over a rolling window."""

    def __init__(self, window_seconds: float = 1.0) -> None:
        if window_seconds <= 0.0:
            raise ValueError("window_seconds must be positive")
        self._window_seconds = window_seconds
        self._samples: deque[tuple[float, float]] = deque()

    def sample(
        self, simulation_time: float, wall_time: float | None = None
    ) -> tuple[float, float] | None:
        """Record one rendered frame and return (FPS, real-time factor)."""
        now = time.perf_counter() if wall_time is None else wall_time
        if self._samples and (
            now <= self._samples[-1][0]
            or simulation_time < self._samples[-1][1]
        ):
            self._samples.clear()
        self._samples.append((now, simulation_time))

        # Keep enough samples to cover the requested averaging interval.
        cutoff = now - self._window_seconds
        while len(self._samples) > 2 and self._samples[1][0] <= cutoff:
            self._samples.popleft()
        if len(self._samples) < 2:
            return None

        wall_delta = now - self._samples[0][0]
        if wall_delta <= 0.0:
            return None
        simulation_delta = simulation_time - self._samples[0][1]
        fps = (len(self._samples) - 1) / wall_delta
        return fps, max(0.0, simulation_delta / wall_delta)


class ViewportStatsOverlay:
    """Display rendered FPS and real-time factor in the active viewport."""

    def __init__(self, enabled: bool = True) -> None:
        self._frame = None
        self._fps_label = None
        self._rtf_label = None
        self._tracker = SimulationRateTracker()
        self._last_text_update = 0.0
        if not enabled:
            return

        import omni.ui as ui
        from omni.kit.viewport.utility import get_active_viewport_window

        viewport_window = get_active_viewport_window()
        if viewport_window is None:
            print("[simulation_stats] Active viewport is unavailable")
            return

        self._frame = viewport_window.get_frame("kinova_simulation_stats")
        self._frame.visible = True
        with self._frame:
            with ui.Placer(offset_x=12, offset_y=52):
                with ui.ZStack(width=190, height=62):
                    ui.Rectangle(
                        style={
                            "background_color": 0xCC202020,
                            "border_color": 0xFF707070,
                            "border_width": 1,
                            "border_radius": 4,
                        }
                    )
                    with ui.VStack(spacing=2):
                        ui.Spacer(height=7)
                        self._fps_label = ui.Label(
                            "  FPS: measuring...",
                            height=20,
                            style={"color": 0xFFFFFFFF, "font_size": 15},
                        )
                        self._rtf_label = ui.Label(
                            "  Real-time factor: measuring...",
                            height=20,
                            style={"color": 0xFFFFFFFF, "font_size": 15},
                        )

    def update(self, simulation_time: float) -> None:
        """Sample one rendered frame and refresh the HUD at a low UI rate."""
        now = time.perf_counter()
        rates = self._tracker.sample(float(simulation_time), now)
        if (
            rates is None
            or self._fps_label is None
            or self._rtf_label is None
            or now - self._last_text_update < 0.2
        ):
            return
        fps, real_time_factor = rates
        self._fps_label.text = f"  FPS: {fps:6.1f}"
        self._rtf_label.text = f"  Real-time factor: {real_time_factor:5.2f}x"
        self._last_text_update = now

    def destroy(self) -> None:
        """Hide the overlay before the application shuts down."""
        if self._frame is not None:
            self._frame.visible = False
        self._frame = None
        self._fps_label = None
        self._rtf_label = None


def enable_ros2_bridge(simulation_app) -> None:
    import carb
    import isaacsim.core.experimental.utils.app as app_utils

    carb.settings.get_settings().set("/app/omni.graph.scriptnode/enable_opt_in", False)
    app_utils.enable_extension("omni.graph.scriptnode")
    app_utils.enable_extension("isaacsim.ros2.bridge")
    simulation_app.update()

def enable_collision_viewer(simulation_app) -> None:
    import carb
    import isaacsim.core.experimental.utils.app as app_utils
    app_utils.enable_extension("omni.physx.ui")
    settings = carb.settings.get_settings()
    settings.set_int(
        "/persistent/physics/visualizationDisplayColliders",
        1,
    )
    simulation_app.update()


def wait_for_stage_load(simulation_app) -> None:
    from isaacsim.core.utils.stage import is_stage_loading

    while is_stage_loading():
        simulation_app.update()

def enable_graph_ui(simulation_app):
    import isaacsim.core.experimental.utils.app as app_utils
    app_utils.enable_extension("omni.graph.window.action")
    app_utils.enable_extension("omni.graph.window.generic")
    app_utils.enable_extension("omni.graph.ui")
    simulation_app.update()


@lru_cache(maxsize=1)
def _load_camera_calibration() -> dict:
    """Load the shared hand-eye camera intrinsics and mount extrinsics."""
    import yaml

    calibration_path = Path(__file__).resolve().parents[1] / "env/urdf/camera.yaml"
    with calibration_path.open(encoding="utf-8") as stream:
        calibration = yaml.safe_load(stream)
    if not isinstance(calibration, dict):
        raise ValueError(f"Camera calibration must be a mapping: {calibration_path}")
    return calibration


def _configure_camera_from_yaml(camera_prim_path: str, topic_prefix: str) -> dict:
    """Apply YAML intrinsics to a camera and its configured pose to its parent."""
    import omni.usd
    from pxr import Gf, Sdf, UsdGeom

    calibration = _load_camera_calibration()
    intrinsics = calibration["intrinsics"]
    camera_key = topic_prefix.strip("/")
    extrinsics = calibration["cameras"][camera_key]["extrinsics"]

    width, height = (int(value) for value in intrinsics["resolution"])
    fx, fy, cx, cy = (
        float(intrinsics[name]) for name in ("fx", "fy", "cx", "cy")
    )
    coefficients = [float(value) for value in intrinsics["distortion_coefficients"]]
    if width <= 0 or height <= 0 or fx <= 0.0 or fy <= 0.0:
        raise ValueError("camera.yaml resolution and focal lengths must be positive")
    if intrinsics.get("distortion_model") != "opencvPinhole":
        raise ValueError("camera.yaml distortion_model must be opencvPinhole")
    if len(coefficients) != 12:
        raise ValueError("camera.yaml distortion_coefficients must contain 12 values")

    stage = omni.usd.get_context().get_stage()
    if stage is None:
        raise RuntimeError("Cannot configure cameras without an open USD stage")
    camera_prim = stage.GetPrimAtPath(camera_prim_path)
    if not camera_prim.IsValid() or not camera_prim.IsA(UsdGeom.Camera):
        raise ValueError(f"Invalid USD camera prim: {camera_prim_path}")

    frame_id = str(calibration["cameras"][camera_key]["frame_id"])
    # IsaacComputeTransformTree honors this attribute. It keeps the TF child
    # frame identical to the frame_id used by RGB, aligned depth, and CameraInfo.
    camera_prim.CreateAttribute(
        "isaac:nameOverride", Sdf.ValueTypeNames.String
    ).Set(frame_id)

    # Isaac Sim's ROS2CameraInfoHelper reads K and D from this modern schema.
    camera_prim.ApplyAPI("OmniLensDistortionOpenCvPinholeAPI")
    camera_prim.GetAttribute("omni:lensdistortion:model").Set("opencvPinhole")
    prefix = "omni:lensdistortion:opencvPinhole"
    camera_prim.GetAttribute(f"{prefix}:imageSize").Set(Gf.Vec2i(width, height))
    for name, value in (("fx", fx), ("fy", fy), ("cx", cx), ("cy", cy)):
        camera_prim.GetAttribute(f"{prefix}:{name}").Set(value)
    coefficient_names = (
        "k1", "k2", "p1", "p2", "k3", "k4",
        "k5", "k6", "s1", "s2", "s3", "s4",
    )
    for name, value in zip(coefficient_names, coefficients):
        camera_prim.GetAttribute(f"{prefix}:{name}").Set(value)

    # Keep the ordinary USD pinhole attributes consistent with the calibration.
    camera = UsdGeom.Camera(camera_prim)
    focal_length = 20.0
    camera.GetFocalLengthAttr().Set(focal_length)
    camera.GetHorizontalApertureAttr().Set(width * focal_length / fx)
    camera.GetVerticalApertureAttr().Set(height * focal_length / fy)

    mount_prim = camera_prim.GetParent()
    attachment_prim = mount_prim.GetParent()
    if not attachment_prim.IsValid():
        raise ValueError(f"Camera mount has no valid attachment parent: {mount_prim.GetPath()}")
    mount_xform = UsdGeom.Xformable(mount_prim)
    translation = Gf.Vec3d(*map(float, extrinsics["translation_xyz_m"]))
    rotation = Gf.Vec3f(*map(float, extrinsics["rotation_rpy_degrees"]))
    translate_op = next(
        (op for op in mount_xform.GetOrderedXformOps() if op.GetOpType() == UsdGeom.XformOp.TypeTranslate),
        None,
    )
    rotate_op = next(
        (op for op in mount_xform.GetOrderedXformOps() if op.GetOpType() == UsdGeom.XformOp.TypeRotateXYZ),
        None,
    )
    if translate_op is None:
        translate_op = mount_xform.AddTranslateOp()
    if rotate_op is None:
        rotate_op = mount_xform.AddRotateXYZOp()
    translate_op.Set(translation)
    rotate_op.Set(rotation)

    return {
        "width": width,
        "height": height,
        "frame_id": frame_id,
        "parent_prim_path": str(attachment_prim.GetPath()),
    }


def create_camera_graph(graph_path: str, camera_prim_path: str, topic_prefix: str) -> None:
    import omni.graph.core as og
    import usdrt.Sdf
    import isaacsim.core.experimental.utils.prim as prim_utils

    camera_config = _configure_camera_from_yaml(camera_prim_path, topic_prefix)
    try:
        og.Controller.edit(
            {"graph_path": graph_path, "evaluator_name": "execution"},
            {
                og.Controller.Keys.CREATE_NODES: [
                    ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                    ("ReadSimTime", "isaacsim.core.nodes.IsaacReadSimulationTime"),
                    ("CreateRenderProduct", "isaacsim.core.nodes.IsaacCreateRenderProduct"),
                    ("ComputeCameraTF", "isaacsim.core.nodes.IsaacComputeTransformTree"),
                    ("Context", "isaacsim.ros2.bridge.ROS2Context"),
                    ("RGBPublish", "isaacsim.ros2.bridge.ROS2CameraHelper"),
                    ("DepthPublish", "isaacsim.ros2.bridge.ROS2CameraHelper"),
                    ("CameraInfoPublish", "isaacsim.ros2.bridge.ROS2CameraInfoHelper"),
                    ("TFPublish", "isaacsim.ros2.bridge.ROS2PublishTransformTree"),
                ],
                og.Controller.Keys.SET_VALUES: [
                    ("CreateRenderProduct.inputs:cameraPrim", [usdrt.Sdf.Path(camera_prim_path)]),
                    ("CreateRenderProduct.inputs:width", camera_config["width"]),
                    ("CreateRenderProduct.inputs:height", camera_config["height"]),
                    ("ComputeCameraTF.inputs:targetPrims", [usdrt.Sdf.Path(camera_prim_path)]),
                    (
                        "ComputeCameraTF.inputs:parentPrim",
                        [usdrt.Sdf.Path(camera_config["parent_prim_path"])],
                    ),
                    ("RGBPublish.inputs:topicName", f"{topic_prefix}/color/image_raw/compressed"),
                    ("RGBPublish.inputs:type", "rgb_h264"),
                    ("RGBPublish.inputs:queueSize", 2),
                    ("RGBPublish.inputs:frameId", camera_config["frame_id"]),
                    ("RGBPublish.inputs:resetSimulationTimeOnStop", True),
                    (
                        "DepthPublish.inputs:topicName",
                        f"{topic_prefix}/aligned_depth_to_color/image_raw",
                    ),
                    ("DepthPublish.inputs:type", "depth"),
                    ("RGBPublish.inputs:queueSize", 2),
                    ("DepthPublish.inputs:frameId", camera_config["frame_id"]),
                    ("DepthPublish.inputs:resetSimulationTimeOnStop", True),
                    (
                        "CameraInfoPublish.inputs:topicName",
                        f"{topic_prefix}/color/camera_info",
                    ),
                    ("CameraInfoPublish.inputs:frameId", camera_config["frame_id"]),
                    ("CameraInfoPublish.inputs:resetSimulationTimeOnStop", True),
                    ("TFPublish.inputs:topicName", "/tf_static"),
                    ("TFPublish.inputs:staticPublisher", True),
                ],
                og.Controller.Keys.CONNECT: [
                    ("OnPlaybackTick.outputs:tick", "CreateRenderProduct.inputs:execIn"),
                    ("OnPlaybackTick.outputs:tick", "ComputeCameraTF.inputs:execIn"),
                    ("CreateRenderProduct.outputs:execOut", "RGBPublish.inputs:execIn"),
                    ("CreateRenderProduct.outputs:execOut", "DepthPublish.inputs:execIn"),
                    ("CreateRenderProduct.outputs:execOut", "CameraInfoPublish.inputs:execIn"),
                    ("CreateRenderProduct.outputs:renderProductPath", "RGBPublish.inputs:renderProductPath"),
                    ("CreateRenderProduct.outputs:renderProductPath", "DepthPublish.inputs:renderProductPath"),
                    ("CreateRenderProduct.outputs:renderProductPath", "CameraInfoPublish.inputs:renderProductPath"),
                    ("Context.outputs:context", "RGBPublish.inputs:context"),
                    ("Context.outputs:context", "DepthPublish.inputs:context"),
                    ("Context.outputs:context", "CameraInfoPublish.inputs:context"),
                    ("ComputeCameraTF.outputs:execOut", "TFPublish.inputs:execIn"),
                    ("ComputeCameraTF.outputs:parentFrames", "TFPublish.inputs:parentFrames"),
                    ("ComputeCameraTF.outputs:childFrames", "TFPublish.inputs:childFrames"),
                    ("ComputeCameraTF.outputs:translations", "TFPublish.inputs:translations"),
                    ("ComputeCameraTF.outputs:orientations", "TFPublish.inputs:orientations"),
                    ("ReadSimTime.outputs:simulationTime", "TFPublish.inputs:timeStamp"),
                    ("Context.outputs:context", "TFPublish.inputs:context"),
                ],
            },
        )

        # Set camera tick rate
        camera_prim = prim_utils.get_prim_at_path(camera_prim_path)
        camera_prim.ApplyAPI("OmniSensorAPI")
        camera_prim.GetAttribute("omni:sensor:tickRate").Set(15)

    except Exception as e:
        print(e)


def _collect_joint_drive_gains(
    robot_root_path: str,
    disable_native_drives: bool = False,
    preserve_native_drive_joints: tuple[str, ...] = (),
) -> dict[str, tuple[float, float, float | None]]:
    """Read gains and optionally disable native drives for effort control."""
    import omni.usd

    stage = omni.usd.get_context().get_stage()
    if stage is None:
        raise RuntimeError("Cannot configure joint control without an open USD stage")

    gains: dict[str, tuple[float, float, float | None]] = {}
    for prim in stage.Traverse():
        if not str(prim.GetPath()).startswith(robot_root_path + "/"):
            continue

        # Revolute and prismatic joints use different DriveAPI instance names.
        for drive_type in ("angular", "linear"):
            prefix = f"drive:{drive_type}:physics"
            stiffness = prim.GetAttribute(f"{prefix}:stiffness")
            damping = prim.GetAttribute(f"{prefix}:damping")
            if not stiffness.IsValid() or not damping.IsValid():
                continue

            kp = float(stiffness.Get() or 0.0)
            kd = float(damping.Get() or 0.0)
            max_force_attr = prim.GetAttribute(f"{prefix}:maxForce")
            max_force_raw = max_force_attr.Get() if max_force_attr.IsValid() else None
            max_force_value = float(max_force_raw) if max_force_raw is not None else math.inf
            max_force = max_force_value if math.isfinite(max_force_value) else None
            gains[prim.GetName()] = (kp, kd, max_force)

            if (
                disable_native_drives
                and prim.GetName() not in preserve_native_drive_joints
            ):
                # Explicit torque modes must not also receive native PD effort.
                stiffness.Set(0.0)
                damping.Set(0.0)
            break

    if not gains:
        raise RuntimeError(f"No driven joints found below {robot_root_path}")
    return gains


def _prepare_mit_joint_drives(
    robot_root_path: str,
    native_pd_joint_names: tuple[str, ...] = (),
) -> dict[str, tuple[float, float, float | None]]:
    """Disable native drives except joints that must remain under PD control."""
    return _collect_joint_drive_gains(
        robot_root_path,
        disable_native_drives=True,
        preserve_native_drive_joints=native_pd_joint_names,
    )


def _make_mit_controller_script(
    gains: dict[str, tuple[float, float, float | None]],
    native_pd_joint_names: tuple[str, ...] = (),
) -> str:
    """Build the ScriptNode program implementing runtime-adjustable MIT control."""
    force_limits = {name: values[2] for name, values in gains.items()}
    template = """
FORCE_LIMITS = __FORCE_LIMITS__
NATIVE_PD_JOINTS = __NATIVE_PD_JOINTS__

def compute(db):
    names = [str(name) for name in db.inputs.jointNames]
    positions = list(db.inputs.positionCommand)
    velocities = list(db.inputs.velocityCommand)
    feedforward = list(db.inputs.effortCommand)

    # A valid MIT command supplies q_des, v_des, and torque_ff per joint.
    if not (len(names) == len(positions) == len(velocities) == len(feedforward)):
        db.outputs.jointNames = []
        db.outputs.effortCommand = []
        return

    measured = {
        str(name): (float(position), float(velocity))
        for name, position, velocity in zip(
            db.inputs.measuredJointNames,
            db.inputs.measuredPositions,
            db.inputs.measuredVelocities,
        )
    }
    gains = {
        str(name): (float(kp), float(kd))
        for name, kp, kd in zip(
            db.inputs.gainJointNames,
            db.inputs.proportionalGains,
            db.inputs.derivativeGains,
        )
    }
    output_names = []
    output_efforts = []
    for name, q_des, v_des, torque_ff in zip(
        names, positions, velocities, feedforward
    ):
        if name not in measured or name not in gains:
            continue
        q, velocity = measured[name]
        kp, kd = gains[name]
        max_force = FORCE_LIMITS.get(name)
        if name in NATIVE_PD_JOINTS:
            # Native PD joints consume q_des/v_des; explicit effort stays zero.
            torque = 0.0
        else:
            torque = kp * (float(q_des) - q)
            torque += kd * (float(v_des) - velocity) + float(torque_ff)

        # Match the physical joint's configured effort/force limit.
        if max_force is not None:
            torque = max(-max_force, min(max_force, torque))
        output_names.append(name)
        output_efforts.append(torque)

    db.outputs.jointNames = output_names
    db.outputs.effortCommand = output_efforts
"""
    return template.replace("__FORCE_LIMITS__", repr(force_limits)).replace(
        "__NATIVE_PD_JOINTS__", repr(native_pd_joint_names)
    )


def _make_direct_force_g_compensation_script(
    gains: dict[str, tuple[float, float, float | None]],
    native_pd_joint_names: tuple[str, ...] = (),
) -> str:
    """Build an effort controller with a native-PD startup hold."""
    force_limits = {name: values[2] for name, values in gains.items()}
    template = """
from isaacsim.core.experimental.prims import Articulation
import omni.usd

FORCE_LIMITS = __FORCE_LIMITS__
NATIVE_PD_JOINTS = __NATIVE_PD_JOINTS__
_ARTICULATION = None
_DOF_INDICES = None
_DIRECT_FORCE_ACTIVE = False

def _clear_outputs(db):
    db.outputs.jointNames = []
    db.outputs.effortCommand = []

def _disable_native_drives(robot_root_path):
    stage = omni.usd.get_context().get_stage()
    if stage is None:
        raise RuntimeError("USD stage is unavailable")

    # Switch every articulation drive from startup PD hold to direct effort.
    for prim in stage.Traverse():
        if not str(prim.GetPath()).startswith(robot_root_path + "/"):
            continue
        if str(prim.GetName()) in NATIVE_PD_JOINTS:
            continue
        for drive_type in ("angular", "linear"):
            prefix = f"drive:{drive_type}:physics"
            stiffness = prim.GetAttribute(f"{prefix}:stiffness")
            damping = prim.GetAttribute(f"{prefix}:damping")
            if stiffness.IsValid() and damping.IsValid():
                stiffness.Set(0.0)
                damping.Set(0.0)
                break

def compute(db):
    global _ARTICULATION, _DOF_INDICES, _DIRECT_FORCE_ACTIVE

    names = [str(name) for name in db.inputs.jointNames]
    commanded_efforts = list(db.inputs.effortCommand)

    # This mode accepts only matching name and effort arrays.
    if not names or len(names) != len(commanded_efforts):
        _clear_outputs(db)
        return

    # Initialize the tensor articulation after physics has started.
    if _ARTICULATION is None:
        _ARTICULATION = Articulation(str(db.inputs.robotPath))
    if not _ARTICULATION.is_physics_tensor_entity_valid():
        _clear_outputs(db)
        return
    if _DOF_INDICES is None:
        _DOF_INDICES = {
            str(name): index for index, name in enumerate(_ARTICULATION.dof_names)
        }

    gravity_efforts = (
        _ARTICULATION.get_dof_gravity_compensation_forces().numpy()[0]
    )

    # A native-PD-only command must not switch the arm out of startup hold.
    has_direct_force_command = any(name not in NATIVE_PD_JOINTS for name in names)
    if not _DIRECT_FORCE_ACTIVE and has_direct_force_command:
        _disable_native_drives(str(db.inputs.driveRootPath))
        _DIRECT_FORCE_ACTIVE = True

    output_names = []
    output_efforts = []
    for name, commanded_effort in zip(names, commanded_efforts):
        dof_index = _DOF_INDICES.get(name)
        if dof_index is None:
            continue

        if name in NATIVE_PD_JOINTS:
            # Fixed joints remain native PD, without direct or gravity effort added.
            effort = 0.0
        else:
            # Limit incoming torque before adding the gravity compensation term.
            clipped_effort = float(commanded_effort)
            max_force = FORCE_LIMITS.get(name)
            if max_force is not None:
                clipped_effort = max(-max_force, min(max_force, clipped_effort))
            effort = clipped_effort + float(gravity_efforts[dof_index])
        output_names.append(name)
        output_efforts.append(effort)

    db.outputs.jointNames = output_names
    db.outputs.effortCommand = output_efforts
"""
    return template.replace("__FORCE_LIMITS__", repr(force_limits)).replace(
        "__NATIVE_PD_JOINTS__", repr(native_pd_joint_names)
    )


class JointGainService:
    """Apply atomic runtime Kp/Kd updates from a standard ROS 2 service."""

    def __init__(
        self,
        control_mode: str,
        robot_root_path: str,
        gains: dict[str, tuple[float, float, float | None]],
    ) -> None:
        _append_internal_rclpy_path()
        import rclpy
        from rcl_interfaces.srv import SetParametersAtomically
        from env.config import JOINT_GAIN_SERVICE

        self._rclpy = rclpy
        self._control_mode = control_mode
        self._robot_root_path = robot_root_path
        self._joint_names = list(gains)
        self._kp = {name: values[0] for name, values in gains.items()}
        self._kd = {name: values[1] for name, values in gains.items()}
        self._owns_context = not rclpy.ok()
        if self._owns_context:
            rclpy.init()
        self._node = rclpy.create_node("isaac_joint_gain_server")
        self._service = self._node.create_service(
            SetParametersAtomically,
            JOINT_GAIN_SERVICE,
            self._handle_request,
        )
        print(
            f"[joint_gain_service] Ready: {JOINT_GAIN_SERVICE} "
            f"({control_mode}, {len(self._joint_names)} joints)"
        )

    @staticmethod
    def _parameter_number(parameter) -> float:
        """Read a ROS parameter value while rejecting nonnumeric types."""
        from rcl_interfaces.msg import ParameterType

        value = parameter.value
        if value.type == ParameterType.PARAMETER_DOUBLE:
            return float(value.double_value)
        if value.type == ParameterType.PARAMETER_INTEGER:
            return float(value.integer_value)
        raise ValueError(f"{parameter.name!r} must be a double or integer")

    def _parse_updates(self, parameters) -> dict[str, dict[str, float]]:
        """Validate the complete request before changing any controller state."""
        updates: dict[str, dict[str, float]] = {}
        for parameter in parameters:
            joint_name, separator, gain_name = parameter.name.rpartition(".")
            if not separator or gain_name not in ("kp", "kd"):
                raise ValueError(
                    f"{parameter.name!r} must use the form '<joint_name>.kp' or "
                    "'<joint_name>.kd'"
                )
            if joint_name not in self._kp:
                raise ValueError(f"unknown joint {joint_name!r}")
            value = self._parameter_number(parameter)
            if not math.isfinite(value) or value < 0.0:
                raise ValueError(f"{parameter.name!r} must be finite and nonnegative")
            joint_updates = updates.setdefault(joint_name, {})
            if gain_name in joint_updates:
                raise ValueError(f"duplicate parameter {parameter.name!r}")
            joint_updates[gain_name] = value
        if not updates:
            raise ValueError("at least one gain parameter is required")
        return updates

    def _set_pd_gains(self, updates: dict[str, dict[str, float]]) -> None:
        """Write native PhysX drive gains, rolling back if an update fails."""
        import omni.usd

        stage = omni.usd.get_context().get_stage()
        if stage is None:
            raise RuntimeError("USD stage is unavailable")
        attributes = {}
        for prim in stage.Traverse():
            if not str(prim.GetPath()).startswith(self._robot_root_path + "/"):
                continue
            if prim.GetName() not in updates:
                continue
            for drive_type in ("angular", "linear"):
                prefix = f"drive:{drive_type}:physics"
                stiffness = prim.GetAttribute(f"{prefix}:stiffness")
                damping = prim.GetAttribute(f"{prefix}:damping")
                if stiffness.IsValid() and damping.IsValid():
                    attributes[prim.GetName()] = (stiffness, damping)
                    break
        missing = set(updates) - set(attributes)
        if missing:
            raise ValueError(f"no native drive found for {sorted(missing)}")

        previous = {
            name: (float(attrs[0].Get() or 0.0), float(attrs[1].Get() or 0.0))
            for name, attrs in attributes.items()
        }
        try:
            for name, change in updates.items():
                stiffness, damping = attributes[name]
                if "kp" in change:
                    if not stiffness.Set(change["kp"]):
                        raise RuntimeError(f"failed to set {name}.kp")
                if "kd" in change:
                    if not damping.Set(change["kd"]):
                        raise RuntimeError(f"failed to set {name}.kd")
        except Exception:
            for name, (kp, kd) in previous.items():
                attributes[name][0].Set(kp)
                attributes[name][1].Set(kd)
            raise

    def _set_mit_gains(self, updates: dict[str, dict[str, float]]) -> None:
        """Replace both ScriptNode gain arrays as one controller snapshot."""
        import omni.graph.core as og

        next_kp = dict(self._kp)
        next_kd = dict(self._kd)
        for name, change in updates.items():
            next_kp[name] = change.get("kp", next_kp[name])
            next_kd[name] = change.get("kd", next_kd[name])
        kp_attribute = og.Controller.attribute(
            "/ROS_Control_KinovaGen3/MITController.inputs:proportionalGains"
        )
        kd_attribute = og.Controller.attribute(
            "/ROS_Control_KinovaGen3/MITController.inputs:derivativeGains"
        )
        try:
            og.Controller.set(
                kp_attribute, [next_kp[name] for name in self._joint_names]
            )
            og.Controller.set(
                kd_attribute, [next_kd[name] for name in self._joint_names]
            )
        except Exception:
            # Restore the previous complete snapshot before returning failure.
            og.Controller.set(
                kp_attribute, [self._kp[name] for name in self._joint_names]
            )
            og.Controller.set(
                kd_attribute, [self._kd[name] for name in self._joint_names]
            )
            raise

    def _handle_request(self, request, response):
        """Validate and apply one atomic SetParameters request."""
        try:
            updates = self._parse_updates(request.parameters)
            if self._control_mode == "pd":
                self._set_pd_gains(updates)
            else:
                self._set_mit_gains(updates)
            for name, change in updates.items():
                self._kp[name] = change.get("kp", self._kp[name])
                self._kd[name] = change.get("kd", self._kd[name])
            response.result.successful = True
            response.result.reason = f"updated {len(updates)} joint(s)"
            print(f"[joint_gain_service] {response.result.reason}: {updates}")
        except Exception as error:
            response.result.successful = False
            response.result.reason = str(error)
            print(f"[joint_gain_service] Rejected request: {error}")
        return response

    def spin_once(self) -> None:
        """Process pending requests without blocking simulation rendering."""
        self._rclpy.spin_once(self._node, timeout_sec=0.0)

    def close(self) -> None:
        """Release the ROS node and the context it initialized."""
        self._node.destroy_node()
        if self._owns_context and self._rclpy.ok():
            self._rclpy.shutdown()


def create_ros_action_graph(
    articulation_root_path: str,
    control_mode: str = "pd",
    robot_root_path: str | None = None,
    fixed_joint_names: tuple[str, ...] = (),
) -> JointGainService | None:
    """Create the ROS 2 action graph for joint state communication.

    Unlike a dual-arm setup with renamed left_/right_ URDF prefixes, this
    single Kinova arm's URDF joint names (`joint_1`..`joint_7`,
    `robotiq_85_left_knuckle_joint`, ...) are already the desired ROS names,
    so no per-joint rename table is needed here.

    Args:
        articulation_root_path: USD prim path for the active articulation root.
        control_mode: ``pd`` uses native position/velocity drives; ``mit``
            computes PD plus feed-forward torque; ``direct_force_g_compensation``
            adds Isaac-computed gravity effort to the ROS effort command.
        robot_root_path: Root used to discover drives for runtime gain updates.
        fixed_joint_names: Joints held at their startup targets by native PD.
    """
    import omni.graph.core as og
    import usdrt.Sdf
    from env.config import (
        GRIPPER_JOINT_PREFIX,
        JOINT_COMMAND_TOPIC,
        JOINT_CONTROL_MODES,
        JOINT_STATE_TOPIC,
    )

    if control_mode not in JOINT_CONTROL_MODES:
        raise ValueError(
            f"Unknown joint control mode {control_mode!r}; expected {JOINT_CONTROL_MODES}"
        )
    if robot_root_path is None:
        raise ValueError("robot_root_path is required for runtime gain control")

    fixed_joint_names = tuple(dict.fromkeys(fixed_joint_names))
    initial_gains = _collect_joint_drive_gains(robot_root_path)
    unknown_fixed_joints = sorted(set(fixed_joint_names) - set(initial_gains))
    if unknown_fixed_joints:
        raise ValueError(f"Unknown fixed joints: {unknown_fixed_joints}")
    gripper_joint_names = tuple(
        name for name in initial_gains if name.startswith(GRIPPER_JOINT_PREFIX)
    )
    native_pd_joint_names = tuple(
        dict.fromkeys((*fixed_joint_names, *gripper_joint_names))
    )

    publish_name_mapper_script = """
import omni.graph.core as og

def compute(db):
    names = [str(name) for name in db.inputs.jointNames]
    db.outputs.jointNames = names
    # The first physics event can arrive before the joint-state sensor is ready.
    # Do not execute the ROS publisher until a complete sensor sample exists.
    db.outputs.execOut = (
        og.ExecutionAttributeState.ENABLED
        if names
        else og.ExecutionAttributeState.DISABLED
    )
"""

    joint_name_mapper_script = """
FIXED_JOINTS = __FIXED_JOINTS__

def compute(db):
    raw_names = [str(name) for name in db.inputs.jointNames]
    kept_indices = [
        index for index, name in enumerate(raw_names) if name not in FIXED_JOINTS
    ]

    # Filter every populated command array with the same joint-name mask.
    def filtered(values):
        values = list(values)
        if len(values) != len(raw_names):
            return values
        return [values[index] for index in kept_indices]

    db.outputs.jointNames = [raw_names[index] for index in kept_indices]
    db.outputs.positionCommand = filtered(db.inputs.positionCommand)
    db.outputs.velocityCommand = filtered(db.inputs.velocityCommand)
    db.outputs.effortCommand = filtered(db.inputs.effortCommand)
""".replace("__FIXED_JOINTS__", repr(fixed_joint_names))

    # Build the mode-specific command path while sharing ROS state publishing.
    controller_nodes = []
    controller_attributes = []
    controller_values = []
    if control_mode == "mit":
        mit_gains = _prepare_mit_joint_drives(
            robot_root_path, native_pd_joint_names
        )
        controller_nodes.append(("MITController", "omni.graph.scriptnode.ScriptNode"))
        controller_attributes.extend(
            [
                ("MITController.inputs:jointNames", "token[]"),
                ("MITController.inputs:positionCommand", "double[]"),
                ("MITController.inputs:velocityCommand", "double[]"),
                ("MITController.inputs:effortCommand", "double[]"),
                ("MITController.inputs:measuredJointNames", "token[]"),
                ("MITController.inputs:measuredPositions", "double[]"),
                ("MITController.inputs:measuredVelocities", "double[]"),
                ("MITController.inputs:gainJointNames", "token[]"),
                ("MITController.inputs:proportionalGains", "double[]"),
                ("MITController.inputs:derivativeGains", "double[]"),
                ("MITController.outputs:jointNames", "token[]"),
                ("MITController.outputs:effortCommand", "double[]"),
            ]
        )
        controller_values.append(
            (
                "MITController.inputs:script",
                _make_mit_controller_script(mit_gains, native_pd_joint_names),
            )
        )
        controller_values.extend(
            [
                ("MITController.inputs:gainJointNames", list(mit_gains)),
                (
                    "MITController.inputs:proportionalGains",
                    [values[0] for values in mit_gains.values()],
                ),
                (
                    "MITController.inputs:derivativeGains",
                    [values[1] for values in mit_gains.values()],
                ),
            ]
        )
    elif control_mode == "direct_force_g_compensation":
        direct_force_gains = _collect_joint_drive_gains(
            robot_root_path, disable_native_drives=False
        )
        controller_nodes.append(
            ("DirectForceGravityController", "omni.graph.scriptnode.ScriptNode")
        )
        controller_attributes.extend(
            [
                ("DirectForceGravityController.inputs:jointNames", "token[]"),
                ("DirectForceGravityController.inputs:effortCommand", "double[]"),
                ("DirectForceGravityController.inputs:robotPath", "string"),
                ("DirectForceGravityController.inputs:driveRootPath", "string"),
                ("DirectForceGravityController.outputs:jointNames", "token[]"),
                ("DirectForceGravityController.outputs:effortCommand", "double[]"),
            ]
        )
        controller_values.extend(
            [
                (
                    "DirectForceGravityController.inputs:script",
                    _make_direct_force_g_compensation_script(
                        direct_force_gains, native_pd_joint_names
                    ),
                ),
                (
                    "DirectForceGravityController.inputs:robotPath",
                    articulation_root_path,
                ),
                (
                    "DirectForceGravityController.inputs:driveRootPath",
                    robot_root_path,
                ),
            ]
        )

    if control_mode == "pd":
        controller_connections = [
            ("JointNameMapper.outputs:execOut", "ArticulationController.inputs:execIn"),
            (
                "JointNameMapper.outputs:jointNames",
                "ArticulationController.inputs:jointNames",
            ),
            (
                "JointNameMapper.outputs:positionCommand",
                "ArticulationController.inputs:positionCommand",
            ),
            (
                "JointNameMapper.outputs:velocityCommand",
                "ArticulationController.inputs:velocityCommand",
            ),
        ]
    elif control_mode == "mit":
        controller_connections = [
            ("PublishNameMapper.outputs:execOut", "MITController.inputs:execIn"),
            ("JointNameMapper.outputs:jointNames", "MITController.inputs:jointNames"),
            (
                "JointNameMapper.outputs:positionCommand",
                "MITController.inputs:positionCommand",
            ),
            (
                "JointNameMapper.outputs:velocityCommand",
                "MITController.inputs:velocityCommand",
            ),
            (
                "JointNameMapper.outputs:effortCommand",
                "MITController.inputs:effortCommand",
            ),
            (
                "PublishNameMapper.outputs:jointNames",
                "MITController.inputs:measuredJointNames",
            ),
            (
                "ReadJointState.outputs:jointPositions",
                "MITController.inputs:measuredPositions",
            ),
            (
                "ReadJointState.outputs:jointVelocities",
                "MITController.inputs:measuredVelocities",
            ),
            ("MITController.outputs:execOut", "ArticulationController.inputs:execIn"),
            (
                "MITController.outputs:jointNames",
                "ArticulationController.inputs:jointNames",
            ),
            (
                "MITController.outputs:effortCommand",
                "ArticulationController.inputs:effortCommand",
            ),
            (
                "JointNameMapper.outputs:positionCommand",
                "ArticulationController.inputs:positionCommand",
            ),
            (
                "JointNameMapper.outputs:velocityCommand",
                "ArticulationController.inputs:velocityCommand",
            ),
        ]
    else:
        controller_connections = [
            (
                "JointNameMapper.outputs:execOut",
                "DirectForceGravityController.inputs:execIn",
            ),
            (
                "JointNameMapper.outputs:jointNames",
                "DirectForceGravityController.inputs:jointNames",
            ),
            (
                "JointNameMapper.outputs:effortCommand",
                "DirectForceGravityController.inputs:effortCommand",
            ),
            (
                "DirectForceGravityController.outputs:execOut",
                "ArticulationController.inputs:execIn",
            ),
            (
                "DirectForceGravityController.outputs:jointNames",
                "ArticulationController.inputs:jointNames",
            ),
            (
                "DirectForceGravityController.outputs:effortCommand",
                "ArticulationController.inputs:effortCommand",
            ),
            (
                "JointNameMapper.outputs:positionCommand",
                "ArticulationController.inputs:positionCommand",
            ),
            (
                "JointNameMapper.outputs:velocityCommand",
                "ArticulationController.inputs:velocityCommand",
            ),
        ]

    try:
        og.Controller.edit(
            {
                "graph_path": "/ROS_Control_KinovaGen3",
                "pipeline_stage": og.GraphPipelineStage.GRAPH_PIPELINE_STAGE_ONDEMAND,
            },
            {
                og.Controller.Keys.CREATE_NODES: [
                    ("OnPhysicsStep", "isaacsim.core.nodes.OnPhysicsStep"),
                    ("ReadSimTime", "isaacsim.core.nodes.IsaacReadSimulationTime"),
                    ("ReadJointState", "isaacsim.sensors.physics.IsaacReadJointState"),
                    ("Context", "isaacsim.ros2.bridge.ROS2Context"),
                    ("PublishClock", "isaacsim.ros2.bridge.ROS2PublishClock"),
                    ("PublishJointState", "isaacsim.ros2.bridge.ROS2PublishJointState"),
                    ("PublishNameMapper", "omni.graph.scriptnode.ScriptNode"),
                    ("SubscribeJointState", "isaacsim.ros2.bridge.ROS2SubscribeJointState"),
                    ("JointNameMapper", "omni.graph.scriptnode.ScriptNode"),
                    ("ArticulationController", "isaacsim.core.nodes.IsaacArticulationController"),
                ] + controller_nodes,
                og.Controller.Keys.CREATE_ATTRIBUTES: [
                    ("PublishNameMapper.inputs:jointNames", "token[]"),
                    ("PublishNameMapper.outputs:jointNames", "token[]"),
                    ("JointNameMapper.inputs:jointNames", "token[]"),
                    ("JointNameMapper.outputs:jointNames", "token[]"),
                    ("JointNameMapper.inputs:positionCommand", "double[]"),
                    ("JointNameMapper.inputs:velocityCommand", "double[]"),
                    ("JointNameMapper.inputs:effortCommand", "double[]"),
                    ("JointNameMapper.outputs:positionCommand", "double[]"),
                    ("JointNameMapper.outputs:velocityCommand", "double[]"),
                    ("JointNameMapper.outputs:effortCommand", "double[]"),
                ] + controller_attributes,
                og.Controller.Keys.CONNECT: [
                    ("OnPhysicsStep.outputs:step", "ReadJointState.inputs:execIn"),
                    ("OnPhysicsStep.outputs:step", "PublishClock.inputs:execIn"),
                    (
                        "ReadSimTime.outputs:simulationTime",
                        "PublishClock.inputs:timeStamp",
                    ),
                    ("Context.outputs:context", "PublishClock.inputs:context"),
                    ("ReadJointState.outputs:execOut", "PublishNameMapper.inputs:execIn"),
                    ("ReadJointState.outputs:jointNames", "PublishNameMapper.inputs:jointNames"),
                    ("PublishNameMapper.outputs:execOut", "PublishJointState.inputs:execIn"),
                    ("PublishNameMapper.outputs:jointNames", "PublishJointState.inputs:jointNames"),
                    ("ReadJointState.outputs:jointPositions", "PublishJointState.inputs:jointPositions"),
                    ("ReadJointState.outputs:jointVelocities", "PublishJointState.inputs:jointVelocities"),
                    ("ReadJointState.outputs:jointEfforts", "PublishJointState.inputs:jointEfforts"),
                    ("ReadJointState.outputs:jointDofTypes", "PublishJointState.inputs:jointDofTypes"),
                    ("ReadJointState.outputs:stageMetersPerUnit", "PublishJointState.inputs:stageMetersPerUnit"),
                    ("ReadJointState.outputs:sensorTime", "PublishJointState.inputs:sensorTime"),
                    ("OnPhysicsStep.outputs:step", "SubscribeJointState.inputs:execIn"),
                    ("Context.outputs:context", "PublishJointState.inputs:context"),
                    ("Context.outputs:context", "SubscribeJointState.inputs:context"),
                    ("ReadSimTime.outputs:simulationTime", "PublishJointState.inputs:timeStamp"),
                    ("SubscribeJointState.outputs:execOut", "JointNameMapper.inputs:execIn"),
                    ("SubscribeJointState.outputs:jointNames", "JointNameMapper.inputs:jointNames"),
                    (
                        "SubscribeJointState.outputs:positionCommand",
                        "JointNameMapper.inputs:positionCommand",
                    ),
                    (
                        "SubscribeJointState.outputs:velocityCommand",
                        "JointNameMapper.inputs:velocityCommand",
                    ),
                    (
                        "SubscribeJointState.outputs:effortCommand",
                        "JointNameMapper.inputs:effortCommand",
                    ),
                ] + controller_connections,
                og.Controller.Keys.SET_VALUES: [
                    ("ReadJointState.inputs:prim", [usdrt.Sdf.Path(articulation_root_path)]),
                    ("PublishNameMapper.inputs:script", publish_name_mapper_script),
                    ("JointNameMapper.inputs:script", joint_name_mapper_script),
                    ("ArticulationController.inputs:robotPath", articulation_root_path),
                    ("PublishJointState.inputs:topicName", JOINT_STATE_TOPIC),
                    ("SubscribeJointState.inputs:topicName", JOINT_COMMAND_TOPIC),
                ] + controller_values,
            },
        )
    except Exception as error:
        raise RuntimeError(
            f"Failed to create ROS joint graph in {control_mode!r} mode"
        ) from error

    if control_mode == "direct_force_g_compensation":
        return None
    service_gains = (
        mit_gains
        if control_mode == "mit"
        else initial_gains
    )
    return JointGainService(control_mode, robot_root_path, service_gains)


def _find_articulation_root_path(robot_root_path: str) -> str:
    """Resolve the exact prim carrying ArticulationRootAPI below `robot_root_path`."""
    import omni.usd
    from pxr import UsdPhysics

    stage = omni.usd.get_context().get_stage()
    if stage is None:
        raise RuntimeError("Cannot resolve the articulation root without an open stage")
    matches = [
        prim
        for prim in stage.Traverse()
        if str(prim.GetPath()).startswith(robot_root_path)
        and prim.HasAPI(UsdPhysics.ArticulationRootAPI)
    ]
    if len(matches) != 1:
        raise RuntimeError(
            f"Expected exactly one articulation root below {robot_root_path}; "
            f"found {[str(prim.GetPath()) for prim in matches]}"
        )
    return str(matches[0].GetPath())


def _find_camera_prim_path(robot_root_path: str, link_name: str, camera_name: str) -> str:
    """Resolve the hand-eye camera prim path without hardcoding import nesting depth."""
    import omni.usd
    from pxr import UsdGeom

    stage = omni.usd.get_context().get_stage()
    if stage is None:
        raise RuntimeError("Cannot resolve the camera prim without an open stage")
    matches = [
        prim
        for prim in stage.Traverse()
        if str(prim.GetPath()).startswith(robot_root_path + "/")
        and prim.GetName() == link_name
    ]
    if len(matches) != 1:
        raise RuntimeError(
            f"Expected one {link_name!r} below {robot_root_path}; "
            f"found {[str(prim.GetPath()) for prim in matches]}"
        )
    return f"{matches[0].GetPath()}/HandEyeCamera/{camera_name}"


def _rpy_degrees_to_quaternion(
    rpy_degrees: tuple[float, float, float]
) -> tuple[float, float, float, float]:
    """Convert an extrinsic-XYZ roll/pitch/yaw (degrees) into an (x, y, z, w) quaternion.

    Matches UsdGeom's RotateXYZOp convention used everywhere else in this
    project for camera mounts: R = Rz(yaw) @ Ry(pitch) @ Rx(roll), applied to
    a vector expressed in the child frame to get it in the parent frame.
    """
    roll, pitch, yaw = (math.radians(value) / 2.0 for value in rpy_degrees)

    def axis_quaternion(axis: int, half_angle: float) -> tuple[float, float, float, float]:
        components = [0.0, 0.0, 0.0]
        components[axis] = math.sin(half_angle)
        return (*components, math.cos(half_angle))

    def multiply(a, b):
        ax, ay, az, aw = a
        bx, by, bz, bw = b
        return (
            aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
            aw * bw - ax * bx - ay * by - az * bz,
        )

    qx = axis_quaternion(0, roll)
    qy = axis_quaternion(1, pitch)
    qz = axis_quaternion(2, yaw)
    return multiply(multiply(qz, qy), qx)


class FixedCameraOpticalTFPublisher:
    """Publish, once, the fixed camera_color_frame -> optical-frame static TF.

    This is a standalone, static_transform_publisher-style broadcast,
    independent of the per-tick camera OmniGraph (create_camera_graph already
    recomputes and republishes the same constant transform every frame, but
    only as a side effect of driving its own render product / CameraInfo —
    it is not meant to stand in for a proper static TF). This class assumes
    something else in the system (e.g. a robot_state_publisher fed by
    /isaac_joint_states) publishes camera_color_frame's own pose in the rest
    of the TF tree; it only adds the fixed optical-frame offset the vendor
    URDF does not define, matching how real camera drivers (e.g. RealSense)
    always add `<link>_optical_frame` via a static transform.

    The node must stay alive for the life of the process: tf2's
    transient-local QoS only serves late-joining subscribers while the
    publisher itself still exists, so call close() only at shutdown.
    """

    def __init__(
        self,
        parent_frame: str,
        child_frame: str,
        translation_xyz_m,
        rotation_rpy_degrees,
    ) -> None:
        _append_internal_rclpy_path()
        import rclpy
        from geometry_msgs.msg import TransformStamped
        from tf2_ros import StaticTransformBroadcaster

        self._rclpy = rclpy
        self._owns_context = not rclpy.ok()
        if self._owns_context:
            rclpy.init()
        self._node = rclpy.create_node("kinova_camera_static_tf_publisher")
        self._broadcaster = StaticTransformBroadcaster(self._node)

        quaternion = _rpy_degrees_to_quaternion(
            tuple(float(value) for value in rotation_rpy_degrees)
        )
        transform = TransformStamped()
        transform.header.stamp = self._node.get_clock().now().to_msg()
        transform.header.frame_id = parent_frame
        transform.child_frame_id = child_frame
        (
            transform.transform.translation.x,
            transform.transform.translation.y,
            transform.transform.translation.z,
        ) = (float(value) for value in translation_xyz_m)
        (
            transform.transform.rotation.x,
            transform.transform.rotation.y,
            transform.transform.rotation.z,
            transform.transform.rotation.w,
        ) = quaternion
        self._broadcaster.sendTransform(transform)
        print(f"[camera_static_tf] Publishing fixed TF: {parent_frame} -> {child_frame}")

    def spin_once(self) -> None:
        """No-op work loop, kept for lifecycle symmetry with JointGainService."""
        self._rclpy.spin_once(self._node, timeout_sec=0.0)

    def close(self) -> None:
        self._node.destroy_node()
        if self._owns_context and self._rclpy.ok():
            self._rclpy.shutdown()


def build_ros2_interface(
    control_mode: str = "pd", fixed_joint_names: tuple[str, ...] = ()
) -> tuple[JointGainService | None, FixedCameraOpticalTFPublisher]:
    from env.config import ARM, CAMERA_COLOR_FRAME_LINK

    articulation_root_path = _find_articulation_root_path(ARM)
    handeye_camera_prim = _find_camera_prim_path(
        ARM, CAMERA_COLOR_FRAME_LINK, "RGBDCameraHandEye"
    )
    create_camera_graph("/ROS_Camera_HandEye", handeye_camera_prim, "/handeye")

    calibration = _load_camera_calibration()
    handeye = calibration["cameras"]["handeye"]
    camera_tf_publisher = FixedCameraOpticalTFPublisher(
        parent_frame=CAMERA_COLOR_FRAME_LINK,
        child_frame=str(handeye["frame_id"]),
        translation_xyz_m=handeye["extrinsics"]["translation_xyz_m"],
        rotation_rpy_degrees=handeye["extrinsics"]["rotation_rpy_degrees"],
    )

    joint_gain_service = create_ros_action_graph(
        articulation_root_path, control_mode, ARM, fixed_joint_names
    )
    return joint_gain_service, camera_tf_publisher
