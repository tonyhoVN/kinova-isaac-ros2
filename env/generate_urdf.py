"""Generate and validate the fixed-base Gen3 7DOF (no gripper) URDF."""

from __future__ import annotations

import argparse
import os
import subprocess
import tempfile
import xml.etree.ElementTree as ET
from pathlib import Path

try:
    from env.config import (
        GEN3_URDF,
        GEN3_URDF_XACRO,
        GEN3_XACRO_ARGS,
        KORTEX_DESCRIPTION_DIR,
        ROS_PYTHON_PATHS,
        ROS_VERSION,
        XACRO_EXECUTABLE,
    )
except ModuleNotFoundError:
    from config import (  # type: ignore[no-redef]
        GEN3_URDF,
        GEN3_URDF_XACRO,
        GEN3_XACRO_ARGS,
        KORTEX_DESCRIPTION_DIR,
        ROS_PYTHON_PATHS,
        ROS_VERSION,
        XACRO_EXECUTABLE,
    )


def _fake_ament_package(prefix: Path, package_name: str, package_dir: Path) -> None:
    marker = prefix / f"share/ament_index/resource_index/packages/{package_name}"
    package_link = prefix / f"share/{package_name}"
    marker.parent.mkdir(parents=True, exist_ok=True)
    marker.write_text(str(prefix), encoding="utf-8")
    package_link.parent.mkdir(parents=True, exist_ok=True)
    package_link.symlink_to(package_dir.resolve(), target_is_directory=True)


def _xacro_environment(prefix: Path) -> dict[str, str]:
    # With no gripper argument, gen3.xacro only pulls in kortex_description
    # (arm macros) — robotiq_description is never included, so it needs no
    # faked ament entry.
    _fake_ament_package(prefix, "kortex_description", KORTEX_DESCRIPTION_DIR)

    environment = os.environ.copy()
    environment.pop("PYTHONHOME", None)
    existing_prefixes = environment.get("AMENT_PREFIX_PATH", "")
    environment["AMENT_PREFIX_PATH"] = os.pathsep.join(
        value
        for value in (str(prefix), f"/opt/ros/{ROS_VERSION}", existing_prefixes)
        if value
    )
    environment["PYTHONPATH"] = os.pathsep.join(ROS_PYTHON_PATHS)
    return environment


def _expand(source: Path, output: Path, environment: dict[str, str]) -> None:
    if not source.is_file():
        raise FileNotFoundError(f"Xacro source does not exist: {source}")
    output.parent.mkdir(parents=True, exist_ok=True)
    subprocess.run(
        [str(XACRO_EXECUTABLE), str(source.resolve()), *GEN3_XACRO_ARGS, "-o", str(output.resolve())],
        cwd=source.parent,
        check=True,
        env=environment,
    )


def _rewrite_fake_package_paths(urdf_path: Path, prefix: Path) -> None:
    """Replace the temporary faked-package symlink path with its real target.

    xacro's `$(find <pkg>)` substitution resolves to the ament-index prefix
    path itself (`prefix/share/<pkg>`, the symlink), not the symlink's
    target, so mesh `file://` URIs would otherwise point into the
    `tempfile.TemporaryDirectory` used to fake the ament index — a directory
    that is deleted as soon as this function returns, long before Isaac Sim
    ever opens the URDF.
    """
    text = urdf_path.read_text(encoding="utf-8")
    old = str(prefix / "share/kortex_description")
    new = str(KORTEX_DESCRIPTION_DIR.resolve())
    urdf_path.write_text(text.replace(old, new), encoding="utf-8")


def _duplicates(elements: list[ET.Element]) -> set[str]:
    names = [element.get("name", "") for element in elements]
    return {name for name in names if name and names.count(name) > 1}


def validate_urdf(path: Path) -> None:
    root = ET.parse(path).getroot()
    if root.tag != "robot" or root.get("name") != "gen3":
        raise RuntimeError(f"Unexpected URDF root in {path}")

    links = root.findall("link")
    joints = root.findall("joint")
    duplicate_links = _duplicates(links)
    duplicate_joints = _duplicates(joints)
    if duplicate_links or duplicate_joints:
        raise RuntimeError(
            f"Duplicate URDF names: links={duplicate_links}, joints={duplicate_joints}"
        )

    link_names = {link.get("name") for link in links}
    joint_names = {joint.get("name") for joint in joints}
    required_links = {
        "world",
        "base_link",
        "end_effector_link",
        "camera_link",
        "camera_depth_frame",
        "camera_color_frame",
        # With no gripper, kortex_robot.xacro terminates the arm here instead.
        "tool_frame",
    }
    required_joints = {
        *(f"joint_{index}" for index in range(1, 8)),
        "tool_frame_joint",
    }
    missing = (required_links - link_names) | (required_joints - joint_names)
    if missing:
        raise RuntimeError(f"Generated URDF is missing: {sorted(missing)}")

    children = {
        child.get("link")
        for joint in joints
        if (child := joint.find("child")) is not None
    }
    roots = link_names - children
    if roots != {"world"}:
        raise RuntimeError(f"URDF must have only 'world' as its root; found {roots}")


def generate_model(
    urdf_xacro: Path = GEN3_URDF_XACRO,
    urdf_output: Path = GEN3_URDF,
) -> Path:
    if not XACRO_EXECUTABLE.is_file():
        raise FileNotFoundError(f"xacro executable does not exist: {XACRO_EXECUTABLE}")
    with tempfile.TemporaryDirectory(prefix="gen3_xacro_") as directory:
        prefix = Path(directory)
        environment = _xacro_environment(prefix)
        _expand(urdf_xacro, urdf_output, environment)
        _rewrite_fake_package_paths(urdf_output, prefix)
    validate_urdf(urdf_output)
    return urdf_output.resolve()


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--urdf-xacro", type=Path, default=GEN3_URDF_XACRO)
    parser.add_argument("--urdf-output", type=Path, default=GEN3_URDF)
    args = parser.parse_args()
    urdf = generate_model(args.urdf_xacro, args.urdf_output)
    print(f"Generated URDF: {urdf}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
