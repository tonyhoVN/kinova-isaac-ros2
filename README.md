# Kinova Gen3 Isaac Sim Project

This project adapts the dual-Franka `dual_manipulation_isaac_sim` testbed
(`~/Projects/dual_manipulation_isaac_sim`) into a single-arm manipulation
scene:

- one Kinova Gen3 7DOF arm with a Robotiq 2F-85 gripper, base fixed to a table
- the arm's built-in wrist vision module exposed as one RGB-D hand-eye camera
- a shared table with randomly spawned YCB objects
- ROS 2 joint command/state topics, a runtime gain-tuning service, and an
  object-segmentation-mask save service

The robot description comes from the vendored ROS packages at
`~/Projects/kinova_ws/src/ros2_kortex/kortex_description` and
`~/Projects/kinova_ws/src/ros2_robotiq_gripper/robotiq_description` (read
directly, not copied into this project — see `env/config.py`). YCB meshes are
likewise reused from `~/Projects/dual_manipulation_isaac_sim/env/ycb-tools`
instead of being duplicated here.

## Project layout

- `env/`
  - `config.py` — paths, prim names, joint names, physics rates, xacro args
  - `generate_urdf.py` — expands `gen3.xacro` (arm + gripper + vision) via a
    faked ament index, then validates the resulting URDF
  - `kinova_table_urdf.py` — imports the URDF into Isaac Sim, builds the
    table scene, sets the arm's ready pose, mounts the hand-eye camera, and
    exports the USD stage (run inside Isaac Sim's own Python)
  - `create_ycb_urdf.py` — generates single-link YCB URDFs (output kept
    inside this project; mesh references point at the external model folder)
  - `lighting.py` — shared scene lighting rig
  - `urdf/camera.yaml` — hand-eye camera intrinsics/extrinsics
  - `urdf/gen3.srdf` — hand-authored "ready" joint pose
- `scripts/`
  - `run_simulation.py` — opens the exported scene and runs the ROS 2 bridge
  - `run_simulation_obj.py` — same, plus spawns random YCB objects, the
    segmentation-save service, and the scene-object-list service
  - `simulation_utils.py` — OmniGraph/ROS 2 wiring shared by both runners
  - `segmentation_service.py` — `/save_object_segmentations` service
  - `scene_object_list_service.py` — `/list_scene_objects` service
  - `publish_sine_joint_commands.py` — example joint-command publisher
- `utils/` — shared path and JSON-state helpers

## How to run

1. Build the URDF and the USD scene (skip once already built):

```bash
python3 env/generate_urdf.py
~/isaacsim/python.sh env/kinova_table_urdf.py
```

2. Run standalone. Before running, activate the ROS bridge in your terminal:

```bash
export isaac_sim_package_path=$HOME/isaacsim
export ROS_DISTRO=humble
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$isaac_sim_package_path/exts/isaacsim.ros2.core/humble/lib
```

Then run with Isaac Sim's Python:

```bash
~/isaacsim/python.sh scripts/run_simulation.py
```

The default `pd` mode consumes desired position and velocity. To enable MIT
control (`Kp(q_des-q) + Kd(v_des-v) + torque_ff`):

```bash
~/isaacsim/python.sh scripts/run_simulation.py --control-mode mit
```

The object-spawning runner supports the same options, plus YCB spawning:

```bash
~/isaacsim/python.sh scripts/run_simulation_obj.py --number 5 --control-mode pd
```

To command effort directly while Isaac Sim compensates gravity:

```bash
~/isaacsim/python.sh scripts/run_simulation_obj.py --control-mode direct_force_g_compensation
```

## Use MoveIt

```bash
ros2 launch kinova_gen3_7dof_robotiq_2f_85_moveit_config robot.launch.py \
  robot_ip:=yyy.yyy.yyy.yyy \
  isaac_sim:=true
```

## ROS 2 interface

- `/isaac_joint_commands` (sub, `sensor_msgs/JointState`) — position/velocity
  (or effort, depending on control mode) targets. By default (`FIXED_JOINTS`
  in `env/config.py`) only `joint_1`..`joint_7` are commandable; the gripper
  (`robotiq_85_left_knuckle_joint`, whose mimics follow automatically) stays
  attached and held at its native-PD position (open) but is excluded from
  this topic — pass `--fixed-joints` with no values at launch to make it
  commandable too.
- `/isaac_joint_states` (pub, `sensor_msgs/JointState`) — measured state.
- `/set_isaac_joint_gains` (`rcl_interfaces/SetParametersAtomically`) —
  runtime `<joint_name>.kp` / `<joint_name>.kd` updates.
- `/save_object_segmentations` (`rcl_interfaces/SetParametersAtomically`) —
  params `save_directory`, `camera_frame` (`handeye_camera_color_optical_frame`);
  saves one instance mask per visible YCB object plus a `manifest.json`.
- `/list_scene_objects` (`std_srvs/Trigger`, `run_simulation_obj.py` only) —
  call with an empty request; `response.message` is a JSON array of
  `{class_name, instance_name, prim_path}` for every currently spawned YCB
  object. Each object's `instance_name` (`object_NNN_<class_name>`) is
  assigned by spawn index at runtime, so it — and therefore which files
  correspond to which physical object in a segmentation capture — is not
  predictable from a fixed name list; query this service instead of
  hardcoding names.
- `/handeye/color/image_raw/compressed`, `/handeye/aligned_depth_to_color/image_raw`,
  `/handeye/color/camera_info` — the hand-eye RGB-D camera.
- `/tf_static`: `camera_color_frame` -> `handeye_camera_color_optical_frame` —
  published once at startup (`FixedCameraOpticalTFPublisher` in
  `scripts/simulation_utils.py`), the fixed REP-103 optical-frame offset the
  vendor URDF doesn't define. Assumes something else (e.g. a
  `robot_state_publisher` fed by `/isaac_joint_states`) publishes
  `camera_color_frame`'s own pose in the rest of the TF tree.

## Notes / follow-ups

- `env/urdf/camera.yaml` intrinsics are nominal placeholders (not a real
  device calibration); its mount rotation is analytically derived (see the
  comments in that file) but not yet visually verified inside Isaac Sim —
  check the rendered image orientation and adjust if needed.
- Per-joint PD stiffness/damping (`env/kinova_table_urdf.py`) are generic
  defaults; Kinova ships no per-joint dynamics.yaml equivalent to Franka's.
- `env/urdf/gen3.srdf`'s "ready" pose is a bent working configuration chosen
  for this project, not copied verbatim from kortex_moveit_config's own
  "Home" state (joint_2/joint_4/joint_6 differ slightly).
