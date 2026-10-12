"""Run the single-Kinova-arm scene and randomly drop YCB objects on the table."""

from __future__ import annotations

import argparse
import hashlib
import math
import random
import re
import time
import xml.etree.ElementTree as ET
from dataclasses import dataclass, replace
from pathlib import Path

try:
    from scripts.simulation_utils import (
        append_internal_ros_libs_from_isaacsim_path,
        bootstrap_repo_path,
        configure_independent_physics_rate,
        create_simulation_app,
        enable_collision_viewer,
        enable_ros2_bridge,
        prefer_internal_ros_libraries,
        ViewportStatsOverlay,
        wait_for_stage_load,
    )
except ModuleNotFoundError:
    from simulation_utils import (  # type: ignore[no-redef]
        append_internal_ros_libs_from_isaacsim_path,
        bootstrap_repo_path,
        configure_independent_physics_rate,
        create_simulation_app,
        enable_collision_viewer,
        enable_ros2_bridge,
        prefer_internal_ros_libraries,
        ViewportStatsOverlay,
        wait_for_stage_load,
    )


@dataclass(frozen=True)
class SpawnArea:
    """Spawn limits in the table frame plus its XY transform to world space."""

    x_min: float
    x_max: float
    y_min: float
    y_max: float
    table_top: float
    origin_xy: tuple[float, float] = (0.0, 0.0)
    x_axis_xy: tuple[float, float] = (1.0, 0.0)
    y_axis_xy: tuple[float, float] = (0.0, 1.0)


def _table_to_world_xy(area: SpawnArea, x: float, y: float) -> tuple[float, float]:
    """Transform a table-local XY position to world XY."""
    return (
        area.origin_xy[0] + x * area.x_axis_xy[0] + y * area.y_axis_xy[0],
        area.origin_xy[1] + x * area.x_axis_xy[1] + y * area.y_axis_xy[1],
    )


def _asset_digest(urdf_path: Path) -> str:
    root = ET.parse(urdf_path).getroot()
    digest = hashlib.sha256(urdf_path.read_bytes())
    for mesh in root.findall(".//mesh"):
        filename = mesh.get("filename")
        if not filename:
            continue
        path = Path(filename)
        if not path.is_absolute():
            path = urdf_path.parent / path
        stat = path.stat()
        digest.update(str(path.resolve()).encode())
        digest.update(f"{stat.st_size}:{stat.st_mtime_ns}".encode())
    return digest.hexdigest()[:12]


def import_ycb_urdf(urdf_path: Path, asset_root: Path, collision_type: str) -> Path:
    """Convert one floating YCB URDF to a cached USD asset."""
    from isaacsim.asset.importer.urdf import URDFImporter, URDFImporterConfig

    digest = _asset_digest(urdf_path)
    output_dir = asset_root / f"{urdf_path.stem}_{digest}"
    marker = output_dir / ".asset_path"
    if marker.is_file():
        cached = Path(marker.read_text(encoding="utf-8").strip())
        if cached.is_file():
            return cached

    output_dir.mkdir(parents=True, exist_ok=True)
    config = URDFImporterConfig(
        urdf_path=str(urdf_path.resolve()),
        usd_path=str(output_dir.resolve()),
        merge_fixed_joints=False,
        merge_mesh=False,
        collision_from_visuals=False,
        collision_type=collision_type,
        allow_self_collision=False,
        fix_base=False,
    )
    result = Path(URDFImporter(config).import_urdf()).resolve()
    if not result.is_file():
        raise RuntimeError(f"YCB URDF importer did not create an asset: {result}")
    marker.write_text(str(result), encoding="utf-8")
    return result


def _table_bounds(stage, margin: float, fallback_height: float) -> SpawnArea:
    from pxr import Gf, Usd, UsdGeom

    table = stage.GetPrimAtPath("/World/Environment/Table")
    if table.IsValid():
        table_to_world = UsdGeom.XformCache(
            Usd.TimeCode.Default()
        ).GetLocalToWorldTransform(table)
        origin = table_to_world.Transform(Gf.Vec3d(0.0, 0.0, 0.0))
        x_point = table_to_world.Transform(Gf.Vec3d(1.0, 0.0, 0.0))
        y_point = table_to_world.Transform(Gf.Vec3d(0.0, 1.0, 0.0))
        origin_xy = (float(origin[0]), float(origin[1]))
        x_axis_xy = (float(x_point[0] - origin[0]), float(x_point[1] - origin[1]))
        y_axis_xy = (float(y_point[0] - origin[0]), float(y_point[1] - origin[1]))

        cache = UsdGeom.BBoxCache(
            Usd.TimeCode.Default(),
            [UsdGeom.Tokens.default_, UsdGeom.Tokens.render],
            useExtentsHint=True,
        )
        bounds = cache.ComputeWorldBound(table).ComputeAlignedRange()
        minimum, maximum = bounds.GetMin(), bounds.GetMax()
        if all(math.isfinite(value) for value in (*minimum, *maximum)):
            world_to_table = table_to_world.GetInverse()
            local_corners = [
                world_to_table.Transform(Gf.Vec3d(x, y, z))
                for x in (minimum[0], maximum[0])
                for y in (minimum[1], maximum[1])
                for z in (minimum[2], maximum[2])
            ]
            local_x_min = min(point[0] for point in local_corners)
            local_x_max = max(point[0] for point in local_corners)
            local_y_min = min(point[1] for point in local_corners)
            local_y_max = max(point[1] for point in local_corners)
            if (
                local_x_max - local_x_min > 2.0 * margin
                and local_y_max - local_y_min > 2.0 * margin
            ):
                return SpawnArea(
                    float(local_x_min + margin),
                    float(local_x_max - margin),
                    float(local_y_min + margin),
                    float(local_y_max - margin),
                    float(maximum[2]),
                    origin_xy,
                    x_axis_xy,
                    y_axis_xy,
                )
        print("[run_simulation_obj] Could not resolve table bounds; using fallback area")
        return SpawnArea(
            -0.45, 0.45, -0.55, 0.55, fallback_height,
            origin_xy, x_axis_xy, y_axis_xy,
        )
    print("[run_simulation_obj] Could not resolve table prim; using world-aligned fallback")
    return SpawnArea(-0.45, 0.45, -0.55, 0.55, fallback_height)


def _sample_positions(
    rng: random.Random,
    count: int,
    area: SpawnArea,
    minimum_spacing: float,
) -> list[tuple[float, float]]:
    positions: list[tuple[float, float]] = []
    for _ in range(count):
        for _attempt in range(2_000):
            candidate = (
                rng.uniform(area.x_min, area.x_max),
                rng.uniform(area.y_min, area.y_max),
            )
            if all(
                math.dist(candidate, existing) >= minimum_spacing
                for existing in positions
            ):
                positions.append(candidate)
                break
        else:
            raise RuntimeError(
                f"Cannot place {count} objects with {minimum_spacing:g} m spacing "
                "inside the table area; lower --number, --margin, or --min-spacing"
            )
    return positions


def spawn_ycb_objects(
    stage,
    assets: list[tuple[str, Path]],
    rng: random.Random,
    *,
    area: SpawnArea,
    drop_height: float,
    minimum_spacing: float,
    static_friction: float,
    dynamic_friction: float,
) -> list[tuple[str, str]]:
    """Spawn the given assets and return each object's (class_name, prim_path).

    Both class_name and prim_path are needed by callers: the segmentation
    service only needs prim_path, while SceneObjectListService needs the
    class_name too, since an object's *instance* name (`object_NNN_<class>`,
    derived from spawn order/index) is only assigned here at runtime and is
    not something an external consumer can predict from a fixed name list.
    """
    from isaacsim.core.experimental.utils.semantics import add_labels
    from pxr import Gf, UsdGeom, UsdPhysics, UsdShade

    root = UsdGeom.Xform.Define(stage, "/World/YCBObjects")
    material = UsdShade.Material.Define(
        stage, "/World/YCBObjects/PhysicsMaterials/YCBObjectMaterial"
    )
    physics_material = UsdPhysics.MaterialAPI.Apply(material.GetPrim())
    physics_material.CreateStaticFrictionAttr().Set(static_friction)
    physics_material.CreateDynamicFrictionAttr().Set(dynamic_friction)
    physics_material.CreateRestitutionAttr().Set(0.0)
    positions = _sample_positions(rng, len(assets), area, minimum_spacing)
    spawned: list[tuple[str, str]] = []
    table_yaw = math.degrees(math.atan2(area.x_axis_xy[1], area.x_axis_xy[0]))
    for index, ((model_name, asset), (table_x, table_y)) in enumerate(zip(assets, positions)):
        world_x, world_y = _table_to_world_xy(area, table_x, table_y)
        safe_name = re.sub(r"[^A-Za-z0-9_]", "_", model_name)
        prim_path = f"{root.GetPath()}/object_{index:03d}_{safe_name}"
        prim = stage.DefinePrim(prim_path, "Xform")
        prim.GetReferences().AddReference(str(asset))
        add_labels(prim, labels=[model_name], taxonomy="class")
        add_labels(prim, labels=[Path(prim_path).name], taxonomy="instance")
        UsdShade.MaterialBindingAPI.Apply(prim).Bind(
            material,
            bindingStrength=UsdShade.Tokens.strongerThanDescendants,
            materialPurpose="physics",
        )
        xform = UsdGeom.Xformable(prim)
        xform.AddTranslateOp().Set(
            Gf.Vec3d(world_x, world_y, area.table_top + drop_height)
        )
        xform.AddRotateZOp().Set(table_yaw + rng.uniform(-180.0, 180.0))
        spawned.append((model_name, prim_path))
    return spawned


def _choose_models(rng: random.Random, models: list[Path], number: int) -> list[Path]:
    if number <= len(models):
        return rng.sample(models, number)
    return [rng.choice(models) for _ in range(number)]


def main() -> int:
    bootstrap_repo_path(__file__)
    from env.config import FIXED_JOINTS, JOINT_CONTROL_MODES

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--number", type=int, default=5, help="Number of objects")
    parser.add_argument("--seed", type=int, default=None)
    parser.add_argument(
        "--objects",
        nargs="*",
        default=[],
        help="Restrict selection to long or short YCB model names",
    )
    parser.add_argument("--ycb-folder", type=Path, default=None)
    parser.add_argument("--asset-dir", type=Path, default=None)
    parser.add_argument("--margin", type=float, default=0.15)
    parser.add_argument("--min-spacing", type=float, default=0.14)
    parser.add_argument("--drop-height", type=float, default=0.15)
    parser.add_argument(
        "--static-friction",
        type=float,
        default=0.8,
        help="Static friction coefficient applied to every spawned object",
    )
    parser.add_argument(
        "--dynamic-friction",
        type=float,
        default=0.6,
        help="Dynamic friction coefficient applied to every spawned object",
    )
    parser.add_argument("--fallback-table-height", type=float, default=0.9)
    parser.add_argument(
        "--x-range", type=float, nargs=2, metavar=("MIN", "MAX"), default=None,
        help="Spawn X range in the table's local coordinate frame (meters)",
    )
    parser.add_argument(
        "--y-range", type=float, nargs=2, metavar=("MIN", "MAX"), default=None,
        help="Spawn Y range in the table's local coordinate frame (meters)",
    )
    parser.add_argument(
        "--table-height",
        type=float,
        default=None,
        help="Override the detected tabletop world Z coordinate",
    )
    parser.add_argument(
        "--collision-type",
        choices=("Convex Hull", "Convex Decomposition", "Bounding Sphere", "Bounding Cube"),
        default="Convex Decomposition",
    )
    parser.add_argument("--headless", action="store_true")
    parser.add_argument("--fps", type=float, default=60.0)
    parser.add_argument(
        "--max-steps",
        type=int,
        default=0,
        help="Exit after this many simulation steps; zero runs until closed",
    )
    parser.add_argument(
        "--ros", action=argparse.BooleanOptionalAction, default=True
    )
    parser.add_argument(
        "--control-mode",
        choices=JOINT_CONTROL_MODES,
        default="pd",
        help=(
            "Joint controller used when ROS is enabled: native PD, MIT, or "
            "direct force plus gravity compensation"
        ),
    )
    parser.add_argument(
        "--fixed-joints",
        nargs="*",
        default=list(FIXED_JOINTS),
        metavar="JOINT",
        help="Joint names to hold at their startup positions",
    )
    args, kit_args = parser.parse_known_args()
    if args.number < 0:
        parser.error("--number cannot be negative")
    if args.max_steps < 0:
        parser.error("--max-steps cannot be negative")
    if args.fps <= 0.0:
        parser.error("--fps must be positive")
    if args.margin < 0.0 or args.min_spacing < 0.0 or args.drop_height < 0.0:
        parser.error("--margin, --min-spacing, and --drop-height cannot be negative")
    if args.static_friction < 0.0 or args.dynamic_friction < 0.0:
        parser.error("--static-friction and --dynamic-friction cannot be negative")
    for option, value in (("--x-range", args.x_range), ("--y-range", args.y_range)):
        if value is not None and value[0] >= value[1]:
            parser.error(f"{option} MIN must be less than MAX")

    from env.config import OUTPUT_USDA, PHYSICS_DT, RENDER_DT, YCB_URDF_ASSET_DIR
    from env.create_ycb_urdf import (
        DEFAULT_YCB_FOLDER,
        create_ycb_urdf,
        discover_models,
        select_models,
    )

    ycb_folder = (args.ycb_folder or DEFAULT_YCB_FOLDER).resolve()
    asset_dir = (args.asset_dir or YCB_URDF_ASSET_DIR).resolve()
    try:
        candidates = select_models(discover_models(ycb_folder), args.objects)
    except (FileNotFoundError, ValueError) as error:
        parser.error(str(error))
    if args.number and not candidates:
        parser.error("No usable YCB models found")

    rng = random.Random(args.seed)
    selected = _choose_models(rng, candidates, args.number)
    print(f"[run_simulation_obj] Candidate pool: {[model.name for model in candidates]}")
    print(f"[run_simulation_obj] Selected {len(selected)} model(s): {[model.name for model in selected]}")
    urdfs = [create_ycb_urdf(model) for model in selected]

    prefer_internal_ros_libraries()
    append_internal_ros_libs_from_isaacsim_path()
    simulation_app = create_simulation_app(args.headless)
    stats_overlay = None
    joint_gain_service = None
    camera_tf_publisher = None
    segmentation_service = None
    scene_object_list_service = None

    enable_collision_viewer(simulation_app)

    try:
        if kit_args:
            print(f"[run_simulation_obj] Isaac Kit arguments: {' '.join(kit_args)}")
        if args.ros:
            enable_ros2_bridge(simulation_app)

        imported: dict[Path, Path] = {}
        assets: list[tuple[str, Path]] = []
        for model, urdf in zip(selected, urdfs):
            if urdf not in imported:
                imported[urdf] = import_ycb_urdf(
                    urdf, asset_dir, args.collision_type
                )
            assets.append((model.name, imported[urdf]))

        from isaacsim.core.api import World
        from isaacsim.core.utils.stage import open_stage
        import omni.timeline
        import omni.usd

        open_stage(usd_path=OUTPUT_USDA)
        wait_for_stage_load(simulation_app)
        stage = omni.usd.get_context().get_stage()
        if stage is None:
            raise RuntimeError(f"Failed to open scene: {OUTPUT_USDA}")

        # Spawn objects
        area = _table_bounds(stage, args.margin, args.fallback_table_height)
        if args.x_range is not None:
            area = replace(area, x_min=args.x_range[0], x_max=args.x_range[1])
        if args.y_range is not None:
            area = replace(area, y_min=args.y_range[0], y_max=args.y_range[1])
        if args.table_height is not None:
            area = replace(area, table_top=args.table_height)
        spawned_objects = spawn_ycb_objects(
            stage,
            assets,
            rng,
            area=area,
            drop_height=args.drop_height,
            minimum_spacing=args.min_spacing,
            static_friction=args.static_friction,
            dynamic_friction=args.dynamic_friction,
        )
        object_prim_paths = [prim_path for _, prim_path in spawned_objects]

        # Create simulation world
        world = World(physics_dt=PHYSICS_DT, rendering_dt=RENDER_DT)
        world.reset()
        maximum_substeps = configure_independent_physics_rate(world, PHYSICS_DT, RENDER_DT)

        print(
            f"[run_simulation_obj] Physics: {1.0 / PHYSICS_DT:g} Hz; "
            f"render: {1.0 / RENDER_DT:g} Hz; max substeps: {maximum_substeps}"
        )

        if args.ros:
            try:
                from scripts.simulation_utils import build_ros2_interface
            except ModuleNotFoundError:
                from simulation_utils import build_ros2_interface
            joint_gain_service, camera_tf_publisher = build_ros2_interface(
                control_mode=args.control_mode,
                fixed_joint_names=tuple(args.fixed_joints),
            )
            print(f"[run_simulation_obj] Joint control mode: {args.control_mode}")
            if args.fixed_joints:
                print(f"[run_simulation_obj] Fixed joints: {args.fixed_joints}")

            try:
                from scripts.scene_object_list_service import SceneObjectListService
            except ModuleNotFoundError:
                from scene_object_list_service import SceneObjectListService
            scene_object_list_service = SceneObjectListService(spawned_objects)

        # Run some initial steps
        for _ in range(10):
            simulation_app.update()
        stats_overlay = ViewportStatsOverlay(enabled=not args.headless)
        timeline = omni.timeline.get_timeline_interface()
        timeline.play()
        steps = 0
        target_frame_time = 1.0 / max(args.fps, 1.0)

        # Main loop
        while simulation_app.is_running() and (
            args.max_steps == 0 or steps < args.max_steps
        ):
            frame_start = time.perf_counter()
            if joint_gain_service is not None:
                joint_gain_service.spin_once()  # Update joint Kp, Kd
            if camera_tf_publisher is not None:
                camera_tf_publisher.spin_once()
            if scene_object_list_service is not None:
                scene_object_list_service.spin_once()

            # Object segmentation image ROS service
            if args.ros and segmentation_service is None:
                try:
                    from scripts.segmentation_service import ObjectSegmentationService
                except ModuleNotFoundError:
                    from segmentation_service import ObjectSegmentationService
                segmentation_service = ObjectSegmentationService(object_prim_paths)
            if segmentation_service is not None:
                segmentation_service.update()

            world.step(render=True)
            stats_overlay.update(world.current_time)
            steps += 1
    finally:
        if segmentation_service is not None:
            segmentation_service.close()
        if joint_gain_service is not None:
            joint_gain_service.close()
        if camera_tf_publisher is not None:
            camera_tf_publisher.close()
        if scene_object_list_service is not None:
            scene_object_list_service.close()
        if stats_overlay is not None:
            stats_overlay.destroy()
        try:
            import omni.timeline

            omni.timeline.get_timeline_interface().stop()
        except Exception:
            pass
        simulation_app.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
