# -----------------------------------------------------------------------------
# Configuration
# -----------------------------------------------------------------------------
from pathlib import Path

WORLD = "/World"
# Prim under which the imported Kinova USD asset is referenced. The URDF's own
# root link is literally named "world" (a fixed, massless link — see
# gen3.xacro), so this prim IS that link once referenced; every other link
# (base_link, shoulder_link, ..., camera_color_frame, robotiq_85_*) appears as
# a descendant of it. Exact nesting depth is not hardcoded anywhere in this
# project — prims are found by name via stage traversal (see
# `find_unique_prim` in env/kinova_table_urdf.py and scripts/simulation_utils.py)
# so the code does not depend on assumptions about exactly how deep the
# URDF importer nests each link.
ARM = f"{WORLD}/KinovaGen3"

GRID_ENV_USD = (
    "https://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/"
    "Isaac/6.0/Isaac/Environments/Grid/default_environment.usd"
)
TABLE_USD = (
    "https://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/"
    "Isaac/6.0/Isaac/SimReady/Industrial/Warehouse/Equipment/"
    "Heavy_Duty_Packing_Table_C01/sm_table_packing_heavyduty_c01_01.usd"
)

### Scene lighting ###
# A low dome-light level prevents fully black surfaces. Broad sphere lights then
# illuminate the work area from several directions without hard shadows.
SCENE_DOME_LIGHT = {
    "intensity": 350.0,
    "color": (0.95, 0.97, 1.0),
    "specular": 0.35,
}
SCENE_SPHERE_LIGHTS = (
    {
        "name": "OverheadKey",
        "position": (0.3, 0.0, 4.5),
        "intensity": 2.0,
        "radius": 1.5,
        "color": (1.0, 0.96, 0.90),
        "specular": 0.65,
        "cast_shadows": False,
    },
    {
        "name": "LeftFill",
        "position": (0.3, -2.5, 2.8),
        "intensity": 1.0,
        "radius": 1.25,
        "color": (0.94, 0.97, 1.0),
        "specular": 0.35,
        "cast_shadows": False,
    },
    {
        "name": "FrontFill",
        "position": (2.2, 0.0, 2.6),
        "intensity": 1.0,
        "radius": 1.25,
        "color": (1.0, 0.98, 0.94),
        "specular": 0.35,
        "cast_shadows": False,
    },
    {
        "name": "RightFill",
        "position": (0.8, 2.5, 2.8),
        "intensity": 1.0,
        "radius": 1.25,
        "color": (0.94, 0.97, 1.0),
        "specular": 0.35,
        "cast_shadows": False,
    },
)

ENV_FOLDER_DIR = Path(__file__).resolve().parent
USDA_FILE_NAME = "kinova_table_generated.usda"
OUTPUT_USDA = f"{str(ENV_FOLDER_DIR)}/{USDA_FILE_NAME}"

### Xacro-imported single Kinova Gen3 7DOF arm + Robotiq 2F-85 gripper ###
ROS_VERSION = "humble"

# Vendored ROS packages are NOT copied into this project (they are large,
# externally maintained checkouts); xacro resolves `$(find ...)` against them
# directly via a faked ament index (see env/generate_urdf.py::_xacro_environment).
KORTEX_DESCRIPTION_DIR = Path.home() / "Projects/kinova_ws/src/ros2_kortex/kortex_description"
ROBOTIQ_DESCRIPTION_DIR = Path.home() / "Projects/kinova_ws/src/ros2_robotiq_gripper/robotiq_description"

GEN3_URDF_XACRO = KORTEX_DESCRIPTION_DIR / "robots/gen3.xacro"
KINOVA_URDF_DIR = ENV_FOLDER_DIR / "urdf"
GEN3_URDF = KINOVA_URDF_DIR / "gen3.urdf"
# Hand-authored (Kinova ships no parametrized SRDF xacro); mirrors the "ready"
# joint values from kortex_moveit_config's own gen3.srdf "Home" group state.
GEN3_SRDF = KINOVA_URDF_DIR / "gen3.srdf"

KINOVA_URDF_ASSET_DIR = ENV_FOLDER_DIR / "kinova_urdf_assets"
XACRO_EXECUTABLE = Path(f"/opt/ros/{ROS_VERSION}/bin/xacro")
ROS_PYTHON_PATHS = (
    f"/opt/ros/{ROS_VERSION}/local/lib/python3.10/dist-packages",
    f"/opt/ros/{ROS_VERSION}/lib/python3.10/site-packages",
)

# Xacro arguments used to build a fixed-base single Gen3 7DOF arm with the
# Robotiq 2F-85 gripper and the wrist vision module enabled. `gen3.xacro`
# always parents the arm to a `<link name="world"/>` it defines itself, so a
# fixed base requires no extra xacro argument.
GEN3_XACRO_ARGS = (
    "arm:=gen3",
    "dof:=7",
    "vision:=true",
    "gripper:=robotiq_2f_85",
    "sim_isaac:=true",
    "use_fake_hardware:=true",
    "prefix:=",
    "robot_ip:=192.168.11.11",
)

### Arm/gripper joint names (as emitted by the xacro above) ###
CAMERA_COLOR_FRAME_LINK = "camera_color_frame"
# The only actuated (non-mimic) gripper joint; every other robotiq_85_* joint
# mimics it (see robotiq_2f_85_macro.urdf.xacro).
GRIPPER_DRIVE_JOINT = "robotiq_85_left_knuckle_joint"
# Matches GRIPPER_DRIVE_JOINT's own <limit lower="0.0" upper="0.8" .../> in
# robotiq_2f_85_macro.urdf.xacro.
GRIPPER_DRIVE_JOINT_LIMITS_RAD = (0.0, 0.8)
# Prefix shared by every gripper joint, used to keep the whole gripper under
# native PD control regardless of the arm's selected control mode.
GRIPPER_JOINT_PREFIX = "robotiq_85_"
# These four joints are URDF "continuous" mimics of GRIPPER_DRIVE_JOINT (see
# robotiq_2f_85_macro.urdf.xacro), so the importer creates them as USD
# RevoluteJoints with no authored limit. Isaac's Newton mimic-joint
# constraint requires a finite limit on a revolute follower joint, so each
# must get one derived from the drive joint's own limit and mimic multiplier
# (see env/kinova_table_urdf.py::fix_gripper_mimic_joint_limits).
GRIPPER_MIMIC_FOLLOWER_MULTIPLIERS = {
    "robotiq_85_left_inner_knuckle_joint": 1.0,
    "robotiq_85_right_inner_knuckle_joint": -1.0,
    "robotiq_85_left_finger_tip_joint": -1.0,
    "robotiq_85_right_finger_tip_joint": 1.0,
}

### Simulation ###
PHYSICS_DT = 1.0 / 100.0
RENDER_DT = 1.0 / 10.0
JOINT_CONTROL_MODES = ("pd", "mit", "direct_force_g_compensation")
# Joints listed here are stripped from any incoming /isaac_joint_commands
# message by JointNameMapper (see scripts/simulation_utils.py) before it ever
# reaches the articulation controller, in every control mode -- regardless of
# what the publisher sends. They stay attached and held by their native PD
# drive at whatever initialize_joints() set (the gripper starts open), just
# permanently excluded from the ROS command interface: by default the
# commandable joints are joint_1..joint_7 only, no gripper. Pass an empty
# --fixed-joints at launch to command the gripper too.
FIXED_JOINTS: tuple[str, ...] = ("robotiq_85_left_knuckle_joint",)
JOINT_COMMAND_TOPIC = "/isaac_joint_commands"
JOINT_STATE_TOPIC = "/isaac_joint_states"
JOINT_GAIN_SERVICE = "/set_isaac_joint_gains"
SEGMENTATION_SAVE_SERVICE = "/save_object_segmentations"
# Instance names (object_NNN_<class>) are assigned at spawn time by index, so
# external consumers cannot predict them from a fixed name list; this service
# reports the current mapping on demand instead.
SCENE_OBJECT_LIST_SERVICE = "/list_scene_objects"

### YCB objects ###
# Reuses the reference project's already-downloaded YCB model library instead
# of duplicating gigabytes of mesh/texture assets into this project.
YCB_MODELS_DIR = (
    Path.home() / "Projects/dual_manipulation_isaac_sim/env/ycb-tools/models/ycb"
)
YCB_URDF_ASSET_DIR = ENV_FOLDER_DIR / "ycb_urdf_assets"
