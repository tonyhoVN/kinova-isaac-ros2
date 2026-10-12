"""Generate the Gen3+Robotiq URDF, import it into Isaac Sim, and export the scene.

Run with Isaac Sim 6.0.1:

    ~/isaacsim/python.sh env/kinova_table_urdf.py
"""

from __future__ import annotations

import argparse
import hashlib
import math
import xml.etree.ElementTree as ET
from pathlib import Path

from isaacsim.simulation_app import SimulationApp

from env.config import (
    ARM,
    CAMERA_COLOR_FRAME_LINK,
    GEN3_SRDF,
    GEN3_URDF,
    GRID_ENV_USD,
    GRIPPER_DRIVE_JOINT,
    GRIPPER_DRIVE_JOINT_LIMITS_RAD,
    GRIPPER_MIMIC_FOLLOWER_MULTIPLIERS,
    KINOVA_URDF_ASSET_DIR,
    OUTPUT_USDA,
    TABLE_USD,
    WORLD,
)
from env.lighting import create_soft_light_rig
from env.generate_urdf import generate_model

ARM_JOINT_STIFFNESS = 4000.0
ARM_JOINT_DAMPING = 200.0
GRIPPER_JOINT_STIFFNESS = 1000.0
GRIPPER_JOINT_DAMPING = 10.0
GRIPPER_MAX_FORCE = 100.0


def set_translate(prim, xyz: tuple[float, float, float]) -> None:
    from pxr import Gf, UsdGeom

    xform = UsdGeom.Xformable(prim)
    operation = next(
        (
            item
            for item in xform.GetOrderedXformOps()
            if item.GetOpType() == UsdGeom.XformOp.TypeTranslate
        ),
        None,
    )
    if operation is None:
        operation = xform.AddTranslateOp(precision=UsdGeom.XformOp.PrecisionDouble)
    operation.Set(Gf.Vec3d(*xyz))


def set_rotate_xyz(prim, rpy_degrees: tuple[float, float, float]) -> None:
    from pxr import Gf, UsdGeom

    operation = UsdGeom.Xformable(prim).AddRotateXYZOp(
        precision=UsdGeom.XformOp.PrecisionFloat
    )
    operation.Set(Gf.Vec3f(*rpy_degrees))


def add_reference(stage, prim_path: str, asset_path: str):
    prim = stage.DefinePrim(prim_path, "Xform")
    prim.GetReferences().AddReference(asset_path)
    return prim


def find_unique_prim(stage, root_path: str, name: str, schema=None) -> object:
    """Find one prim by name below `root_path`, regardless of import nesting depth."""
    matches = [
        prim
        for prim in stage.Traverse()
        if str(prim.GetPath()).startswith(root_path + "/")
        and prim.GetName() == name
        and (schema is None or schema(prim))
    ]
    if len(matches) != 1:
        raise RuntimeError(
            f"Expected one {name!r} below {root_path}; "
            f"found {[str(prim.GetPath()) for prim in matches]}"
        )
    return matches[0]


def make_handeye_camera(stage, camera_color_frame_prim) -> str:
    """Create the RGB-D hand-eye camera using intrinsics/extrinsics from camera.yaml."""
    import yaml
    from pxr import Gf, Sdf, UsdGeom

    calibration_path = Path(__file__).resolve().parent / "urdf/camera.yaml"
    with calibration_path.open(encoding="utf-8") as stream:
        calibration = yaml.safe_load(stream)
    intrinsics = calibration["intrinsics"]
    extrinsics = calibration["cameras"]["handeye"]["extrinsics"]
    width, height = (int(value) for value in intrinsics["resolution"])
    fx, fy, cx, cy = (float(intrinsics[name]) for name in ("fx", "fy", "cx", "cy"))
    coefficients = [float(value) for value in intrinsics["distortion_coefficients"]]
    if width <= 0 or height <= 0 or fx <= 0.0 or fy <= 0.0:
        raise ValueError("camera.yaml resolution and focal lengths must be positive")
    if intrinsics.get("distortion_model") != "opencvPinhole":
        raise ValueError("camera.yaml distortion_model must be opencvPinhole")
    if len(coefficients) != 12:
        raise ValueError("camera.yaml distortion_coefficients must contain 12 values")

    mount_path = f"{camera_color_frame_prim.GetPath()}/HandEyeCamera"
    mount = UsdGeom.Xform.Define(stage, mount_path).GetPrim()
    set_translate(mount, tuple(float(value) for value in extrinsics["translation_xyz_m"]))
    set_rotate_xyz(mount, tuple(float(value) for value in extrinsics["rotation_rpy_degrees"]))

    camera_path = f"{mount_path}/RGBDCameraHandEye"
    camera = UsdGeom.Camera.Define(stage, camera_path)
    camera.CreateProjectionAttr(UsdGeom.Tokens.perspective)
    focal_length = 20.0
    camera.CreateFocalLengthAttr(focal_length)
    camera.CreateHorizontalApertureAttr(width * focal_length / fx)
    camera.CreateVerticalApertureAttr(height * focal_length / fy)
    camera.CreateClippingRangeAttr(Gf.Vec2f(0.05, 5.0))
    camera.CreateFStopAttr(0.0)
    camera.CreateFocusDistanceAttr(1.0)
    camera.GetVisibilityAttr().Set(UsdGeom.Tokens.invisible)

    camera_prim = camera.GetPrim()
    camera_prim.CreateAttribute("isaac:nameOverride", Sdf.ValueTypeNames.String).Set(
        str(calibration["cameras"]["handeye"]["frame_id"])
    )
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
    return camera_path


def import_kinova_urdf(urdf_path: Path, asset_dir: Path) -> Path:
    """Import the Gen3+Robotiq articulation, with a content-addressed cache."""
    from isaacsim.asset.importer.urdf import URDFImporter, URDFImporterConfig
    from env.config import KORTEX_DESCRIPTION_DIR, ROBOTIQ_DESCRIPTION_DIR

    digest = hashlib.sha256(urdf_path.read_bytes()).hexdigest()[:12]
    import_dir = asset_dir / f"gen3_robotiq_2f_85_{digest}"
    marker = import_dir / ".asset_path"

    if marker.is_file():
        cached = Path(marker.read_text(encoding="utf-8").strip())
        if cached.is_file():
            print(f"Using cached Gen3+Robotiq asset: {cached}")
            return cached.resolve()

    asset_name = urdf_path.stem
    for extension in (".usd", ".usda", ".usdc"):
        cached = import_dir / asset_name / f"{asset_name}{extension}"
        if cached.is_file():
            marker.write_text(str(cached.resolve()), encoding="utf-8")
            print(f"Using cached Gen3+Robotiq asset: {cached}")
            return cached.resolve()

    import_dir.mkdir(parents=True, exist_ok=True)
    config = URDFImporterConfig(
        urdf_path=str(urdf_path.resolve()),
        usd_path=str(import_dir.resolve()),
        # Fixed joints must not be merged away: camera_link/camera_depth_frame/
        # camera_color_frame and the gripper's structural links are all
        # connected via fixed joints and would otherwise disappear.
        merge_fixed_joints=False,
        fix_base=True,
        allow_self_collision=False,
        joint_drive_type="force",
        joint_target_type="position",
        ros_package_paths=[
            {"name": "kortex_description", "path": str(KORTEX_DESCRIPTION_DIR.resolve())},
            {"name": "robotiq_description", "path": str(ROBOTIQ_DESCRIPTION_DIR.resolve())},
        ],
    )
    result = Path(URDFImporter(config).import_urdf()).resolve()
    if not result.is_file():
        raise RuntimeError(f"URDF importer did not create the expected asset: {result}")
    marker.write_text(str(result), encoding="utf-8")
    print(f"Imported Gen3+Robotiq asset: {result}")
    return result


def _load_ready_positions(srdf_path: Path) -> dict[str, float]:
    root = ET.parse(srdf_path).getroot()
    result: dict[str, float] = {}
    for state in root.findall("./group_state[@name='ready']"):
        for joint in state.findall("joint"):
            name, value = joint.get("name"), joint.get("value")
            if name is None or value is None:
                raise RuntimeError(f"Malformed ready group state in {srdf_path}")
            result[name] = float(value)
    if len(result) != 7:
        raise RuntimeError(f"Expected 7 ready arm joints, found {len(result)}")
    return result


def set_initial_revolute_joint_position(
    prim, position_radians: float, stiffness: float, damping: float
) -> None:
    from pxr import PhysxSchema, UsdPhysics

    position_degrees = math.degrees(position_radians)
    state = PhysxSchema.JointStateAPI.Apply(prim, "angular")
    state.CreatePositionAttr().Set(position_degrees)
    state.CreateVelocityAttr().Set(0.0)
    drive = UsdPhysics.DriveAPI.Apply(prim, "angular")
    drive.CreateTypeAttr("force")
    drive.CreateTargetPositionAttr().Set(position_degrees)
    drive.CreateTargetVelocityAttr().Set(0.0)
    drive.CreateStiffnessAttr().Set(stiffness)
    drive.CreateDampingAttr().Set(damping)


def initialize_joints(stage, srdf_path: Path) -> None:
    from pxr import UsdPhysics

    for joint_name, position in _load_ready_positions(srdf_path).items():
        joint = find_unique_prim(stage, ARM, joint_name, UsdPhysics.RevoluteJoint)
        set_initial_revolute_joint_position(
            joint, position, ARM_JOINT_STIFFNESS, ARM_JOINT_DAMPING
        )

    gripper = find_unique_prim(stage, ARM, GRIPPER_DRIVE_JOINT, UsdPhysics.RevoluteJoint)
    set_initial_revolute_joint_position(gripper, 0.0, GRIPPER_JOINT_STIFFNESS, GRIPPER_JOINT_DAMPING)
    UsdPhysics.DriveAPI.Apply(gripper, "angular").CreateMaxForceAttr().Set(GRIPPER_MAX_FORCE)


def fix_gripper_mimic_joint_limits(stage) -> None:
    """Author finite limits on the Robotiq's continuous mimic-follower joints.

    robotiq_85_{left,right}_inner_knuckle_joint and
    robotiq_85_{left,right}_finger_tip_joint are URDF "continuous" mimics of
    GRIPPER_DRIVE_JOINT (see robotiq_2f_85_macro.urdf.xacro), so the URDF
    importer creates them as USD RevoluteJoints with no authored limit.
    Isaac's Newton mimic-joint constraint rejects a revolute follower with no
    finite limit, so each gets one derived from the drive joint's own
    [0, 0.8] rad limit and its mimic multiplier.
    """
    from pxr import UsdPhysics

    lower_rad, upper_rad = GRIPPER_DRIVE_JOINT_LIMITS_RAD
    for joint_name, multiplier in GRIPPER_MIMIC_FOLLOWER_MULTIPLIERS.items():
        joint = find_unique_prim(stage, ARM, joint_name, UsdPhysics.RevoluteJoint)
        bounds = sorted((multiplier * lower_rad, multiplier * upper_rad))
        revolute = UsdPhysics.RevoluteJoint(joint)
        revolute.CreateLowerLimitAttr().Set(math.degrees(bounds[0]))
        revolute.CreateUpperLimitAttr().Set(math.degrees(bounds[1]))


def build_stage(output_path: Path = Path(OUTPUT_USDA), asset_dir: Path = KINOVA_URDF_ASSET_DIR):
    """Generate, import, initialize, and export the single-Kinova table scene."""
    import omni.usd
    from pxr import Gf, UsdGeom, UsdPhysics

    urdf_path = generate_model()
    robot_asset = import_kinova_urdf(urdf_path, asset_dir)

    context = omni.usd.get_context()
    context.new_stage()
    stage = context.get_stage()
    if stage is None:
        raise RuntimeError("Failed to create a new USD stage")

    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    stage.SetTimeCodesPerSecond(60.0)
    stage.SetStartTimeCode(0.0)
    stage.SetEndTimeCode(1_000_000.0)

    world = UsdGeom.Xform.Define(stage, WORLD)
    stage.SetDefaultPrim(world.GetPrim())
    add_reference(stage, f"{WORLD}/GridRoomEnvironment", GRID_ENV_USD)

    physics_scene = UsdPhysics.Scene.Define(stage, f"{WORLD}/physicsScene")
    physics_scene.CreateGravityDirectionAttr(Gf.Vec3f(0.0, 0.0, -1.0))
    physics_scene.CreateGravityMagnitudeAttr(9.81)
    physics_scene.GetPrim().AddAppliedSchema("PhysxSceneAPI")

    create_soft_light_rig(stage)

    UsdGeom.Xform.Define(stage, f"{WORLD}/Environment")
    table = add_reference(stage, f"{WORLD}/Environment/Table", TABLE_USD)
    set_translate(table, (0.3, 0.0, 0.0))
    set_rotate_xyz(table, (0.0, 0.0, 90.0))

    # Mount the arm's fixed base on the tabletop.
    robot = add_reference(stage, ARM, str(robot_asset))
    set_translate(robot, (-0.2, 0.0, 1.0))
    initialize_joints(stage, GEN3_SRDF)
    fix_gripper_mimic_joint_limits(stage)

    articulation_roots = [
        prim
        for prim in stage.Traverse()
        if str(prim.GetPath()).startswith(ARM)
        and prim.HasAPI(UsdPhysics.ArticulationRootAPI)
    ]
    if len(articulation_roots) != 1:
        raise RuntimeError(
            "Imported URDF must contain one articulation; found "
            f"{[str(prim.GetPath()) for prim in articulation_roots]}"
        )

    camera_color_frame = find_unique_prim(stage, ARM, CAMERA_COLOR_FRAME_LINK)
    handeye_camera = make_handeye_camera(stage, camera_color_frame)

    output_path = output_path.resolve()
    output_path.parent.mkdir(parents=True, exist_ok=True)
    if not stage.GetRootLayer().Export(str(output_path)):
        raise RuntimeError(f"Failed to export scene: {output_path}")

    print(f"Generated URDF: {GEN3_URDF}")
    print(f"Exported Isaac Sim scene: {output_path}")
    print("Initial arm group state: ready")
    print(f"Articulation root: {articulation_roots[0].GetPath()}")
    print(f"Hand-eye camera: {handeye_camera}")
    return stage


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=Path(OUTPUT_USDA))
    parser.add_argument("--asset-dir", type=Path, default=KINOVA_URDF_ASSET_DIR)
    parser.add_argument(
        "--headless", action=argparse.BooleanOptionalAction, default=True
    )
    args = parser.parse_args()

    simulation_app = SimulationApp({"headless": args.headless})
    try:
        build_stage(args.output, args.asset_dir)
    finally:
        simulation_app.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
