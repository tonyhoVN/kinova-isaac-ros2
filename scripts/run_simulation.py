from __future__ import annotations

import argparse

from scripts.simulation_utils import (
    append_internal_ros_libs_from_isaacsim_path,
    build_ros2_interface,
    bootstrap_repo_path,
    configure_independent_physics_rate,
    create_simulation_app,
    enable_ros2_bridge,
    prefer_internal_ros_libraries,
    ViewportStatsOverlay,
    wait_for_stage_load,
)


def main() -> int:
    bootstrap_repo_path(__file__)
    from env.config import FIXED_JOINTS, JOINT_CONTROL_MODES

    parser = argparse.ArgumentParser(
        description="Run the Kinova Gen3 Isaac Sim scene with ROS 2 bridge graphs."
    )
    parser.add_argument(
        "--headless", action="store_true", help="Launch Isaac Sim without a viewport."
    )
    parser.add_argument(
        "--fps", type=float, default=60.0, help="Target wall-clock frame rate."
    )
    parser.add_argument(
        "--control-mode",
        choices=JOINT_CONTROL_MODES,
        default="pd",
        help=(
            "Joint controller: native PD (default), MIT PD plus feed-forward "
            "torque, or direct force plus gravity compensation."
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

    prefer_internal_ros_libraries()
    append_internal_ros_libs_from_isaacsim_path()

    from env.config import OUTPUT_USDA, PHYSICS_DT, RENDER_DT

    simulation_app = create_simulation_app(args.headless)
    joint_gain_service = None
    camera_tf_publisher = None
    stats_overlay = None
    try:
        if kit_args:
            print(f"[run_simulation] Passing through Isaac Kit args: {' '.join(kit_args)}")

        enable_ros2_bridge(simulation_app)

        from isaacsim.core.api import World
        from isaacsim.core.utils.stage import open_stage
        import omni.timeline

        open_stage(usd_path=OUTPUT_USDA)
        wait_for_stage_load(simulation_app)

        world = World(physics_dt=PHYSICS_DT, rendering_dt=RENDER_DT)
        world.reset()
        maximum_substeps = configure_independent_physics_rate(
            world, PHYSICS_DT, RENDER_DT
        )
        print(
            f"[run_simulation] Physics: {1.0 / PHYSICS_DT:g} Hz; "
            f"render: {1.0 / RENDER_DT:g} Hz; max substeps: {maximum_substeps}"
        )

        joint_gain_service, camera_tf_publisher = build_ros2_interface(
            control_mode=args.control_mode,
            fixed_joint_names=tuple(args.fixed_joints),
        )
        print(f"[run_simulation] Joint control mode: {args.control_mode}")
        if args.fixed_joints:
            print(f"[run_simulation] Fixed joints: {args.fixed_joints}")

        for _ in range(10):
            simulation_app.update()
        stats_overlay = ViewportStatsOverlay(enabled=not args.headless)

        timeline = omni.timeline.get_timeline_interface()
        timeline.play()

        while simulation_app.is_running():
            if joint_gain_service is not None:
                joint_gain_service.spin_once()
            if camera_tf_publisher is not None:
                camera_tf_publisher.spin_once()
            world.step(render=True)
            stats_overlay.update(world.current_time)
    finally:
        if joint_gain_service is not None:
            joint_gain_service.close()
        if camera_tf_publisher is not None:
            camera_tf_publisher.close()
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
