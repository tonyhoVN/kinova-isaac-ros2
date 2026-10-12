#!/usr/bin/env python3
"""Create Isaac Sim-compatible URDF files for YCB models.

Follows the same single-link-per-model approach as
``ycb-tools/create_ycb_sdf.py``, sourcing meshes from the YCB model library
already downloaded for the dual-manipulation reference project (see
``env.config.YCB_MODELS_DIR``) instead of duplicating that data here.
"""

from __future__ import annotations

import argparse
import math
import re
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from pathlib import Path

try:
    from env.config import ENV_FOLDER_DIR, YCB_MODELS_DIR as DEFAULT_YCB_FOLDER
except ModuleNotFoundError:
    from config import ENV_FOLDER_DIR, YCB_MODELS_DIR as DEFAULT_YCB_FOLDER  # type: ignore[no-redef]

# YCB_MODELS_DIR is an external, read-only checkout (see env/config.py); this
# project must not write into it, so generated URDFs land here instead, with
# mesh references pointing back at the external model folder by absolute path.
DEFAULT_URDF_OUTPUT_DIR = ENV_FOLDER_DIR / "ycb_urdf"

_NUMPY_SCALAR = re.compile(r"np\.float(?:32|64)?\(([^()]*)\)")


@dataclass(frozen=True)
class Inertial:
    mass: float
    xyz: tuple[float, float, float]
    rpy: tuple[float, float, float]
    inertia: tuple[float, float, float, float, float, float]


def _numbers(text: str, count: int) -> tuple[float, ...]:
    cleaned = _NUMPY_SCALAR.sub(r"\1", text)
    values = tuple(float(value) for value in cleaned.split())
    if len(values) != count or not all(math.isfinite(value) for value in values):
        raise ValueError(f"Expected {count} finite values, got {text!r}")
    return values


def _required_text(parent: ET.Element, path: str) -> str:
    element = parent.find(path)
    if element is None or not element.text or not element.text.strip():
        raise ValueError(f"Missing SDF value: {path}")
    return element.text.strip()


def _mesh_paths(
    model_folder: Path,
    use_downsample_mesh: bool = False,
) -> tuple[Path, Path]:
    candidates = (
        model_folder / "google_16k/textured.obj",
        model_folder / "tsdf/textured.obj",
    )
    visual = next((path for path in candidates if path.is_file()), None)
    if visual is None:
        raise FileNotFoundError(f"No textured OBJ mesh in {model_folder}")
    if not use_downsample_mesh:
        return visual, visual

    downsampled = model_folder / "downsampled.obj"
    return visual, downsampled if downsampled.is_file() else visual


def _inertial_from_sdf(model_folder: Path) -> Inertial | None:
    sdf_files = sorted(model_folder.glob("*.sdf"))
    if not sdf_files:
        return None
    link = ET.parse(sdf_files[0]).getroot().find("./model/link")
    if link is None:
        raise ValueError(f"No model/link in {sdf_files[0]}")
    pose = _numbers(_required_text(link, "inertial/pose"), 6)
    inertia_element = link.find("inertial/inertia")
    if inertia_element is None:
        raise ValueError(f"No inertial/inertia in {sdf_files[0]}")
    inertia = tuple(
        float(_required_text(inertia_element, axis))
        for axis in ("ixx", "ixy", "ixz", "iyy", "iyz", "izz")
    )
    return Inertial(
        mass=float(_required_text(link, "inertial/mass")),
        xyz=(pose[0], pose[1], pose[2]),
        rpy=(pose[3], pose[4], pose[5]),
        inertia=inertia,
    )


def _inertial_from_mesh(mesh_path: Path) -> Inertial:
    try:
        import numpy as np
        import trimesh
    except ImportError as error:
        raise RuntimeError(
            f"No SDF inertial data for {mesh_path.parent.parent.name}; "
            "install numpy and trimesh to calculate it"
        ) from error

    loaded = trimesh.load(mesh_path, force="mesh")
    if isinstance(loaded, trimesh.Scene):
        loaded = loaded.dump(concatenate=True)
    transform = loaded.principal_inertia_transform
    tensor = trimesh.inertia.transform_inertia(transform, loaded.moment_inertia)
    rpy = trimesh.transformations.euler_from_matrix(
        np.linalg.inv(transform), axes="sxyz"
    )
    return Inertial(
        mass=float(loaded.mass),
        xyz=tuple(float(value) for value in loaded.center_mass),
        rpy=tuple(float(value) for value in rpy),
        inertia=(
            float(tensor[0, 0]),
            float(tensor[0, 1]),
            float(tensor[0, 2]),
            float(tensor[1, 1]),
            float(tensor[1, 2]),
            float(tensor[2, 2]),
        ),
    )


def _format(values: tuple[float, ...]) -> str:
    return " ".join(f"{value:.12g}" for value in values)


def _mesh_filename(path: Path, output: Path, absolute: bool) -> str:
    if absolute:
        return str(path.resolve())
    return path.relative_to(output.parent).as_posix()


def create_ycb_urdf(
    model_folder: Path,
    *,
    output_dir: Path = DEFAULT_URDF_OUTPUT_DIR,
    overwrite: bool = False,
    absolute_mesh_paths: bool = True,
    use_downsample_mesh: bool = False,
) -> Path:
    """Create one YCB URDF and return its path.

    The URDF is written to `output_dir` (inside this project), never into
    `model_folder`, since the YCB model library is an external, read-only
    checkout shared with the dual-manipulation reference project.
    """
    model_folder = model_folder.resolve()
    model_long = model_folder.name
    model_short = model_long[4:] if re.match(r"^\d{3}[_-]", model_long) else model_long
    output_dir = output_dir.resolve()
    output_dir.mkdir(parents=True, exist_ok=True)
    output = output_dir / f"{model_short}.urdf"
    if output.is_file() and not overwrite:
        return output

    visual_mesh, collision_mesh = _mesh_paths(
        model_folder,
        use_downsample_mesh=use_downsample_mesh,
    )
    inertial = _inertial_from_sdf(model_folder) or _inertial_from_mesh(visual_mesh)
    if inertial.mass <= 0.0:
        raise ValueError(f"Non-positive mass for {model_long}: {inertial.mass}")

    robot = ET.Element("robot", {"name": model_short})
    link = ET.SubElement(robot, "link", {"name": "object_link"})
    inertial_xml = ET.SubElement(link, "inertial")
    ET.SubElement(
        inertial_xml,
        "origin",
        {"xyz": _format(inertial.xyz), "rpy": _format(inertial.rpy)},
    )
    ET.SubElement(inertial_xml, "mass", {"value": f"{inertial.mass:.12g}"})
    ixx, ixy, ixz, iyy, iyz, izz = inertial.inertia
    ET.SubElement(
        inertial_xml,
        "inertia",
        {
            "ixx": f"{ixx:.12g}",
            "ixy": f"{ixy:.12g}",
            "ixz": f"{ixz:.12g}",
            "iyy": f"{iyy:.12g}",
            "iyz": f"{iyz:.12g}",
            "izz": f"{izz:.12g}",
        },
    )

    visual = ET.SubElement(link, "visual", {"name": "visual"})
    ET.SubElement(visual, "origin", {"xyz": "0 0 0", "rpy": "0 0 0"})
    visual_geometry = ET.SubElement(visual, "geometry")
    ET.SubElement(
        visual_geometry,
        "mesh",
        {"filename": _mesh_filename(visual_mesh, output, absolute_mesh_paths)},
    )

    collision = ET.SubElement(link, "collision", {"name": "collision"})
    ET.SubElement(collision, "origin", {"xyz": "0 0 0", "rpy": "0 0 0"})
    collision_geometry = ET.SubElement(collision, "geometry")
    ET.SubElement(
        collision_geometry,
        "mesh",
        {"filename": _mesh_filename(collision_mesh, output, absolute_mesh_paths)},
    )

    ET.indent(robot, space="  ")
    ET.ElementTree(robot).write(output, encoding="utf-8", xml_declaration=True)
    return output


def discover_models(ycb_folder: Path) -> list[Path]:
    if not ycb_folder.is_dir():
        raise FileNotFoundError(f"YCB model folder does not exist: {ycb_folder}")
    return sorted(
        folder
        for folder in ycb_folder.iterdir()
        if folder.is_dir()
        and folder.name != "template"
        and any(path.is_file() for path in (folder / "google_16k/textured.obj", folder / "tsdf/textured.obj"))
    )


def select_models(model_folders: list[Path], requested: list[str]) -> list[Path]:
    """Resolve requested model names to folders, preserving order and repeats.

    A name listed twice (e.g. `--objects banana banana`) yields two entries
    pointing at the same folder, so callers that want to guarantee more than
    one instance of a model spawns can just repeat its name.
    """
    if not requested:
        return model_folders
    by_name: dict[str, Path] = {}
    for folder in model_folders:
        by_name[folder.name] = folder
        by_name[folder.name[4:]] = folder
    missing = [name for name in requested if name not in by_name]
    if missing:
        raise ValueError(f"Unknown/unavailable YCB models: {', '.join(missing)}")
    return [by_name[name] for name in requested]


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--ycb-folder", type=Path, default=DEFAULT_YCB_FOLDER)
    parser.add_argument(
        "--objects",
        nargs="*",
        default=[],
        help="Optional long names (003_cracker_box) or short names (cracker_box)",
    )
    parser.add_argument("--overwrite", action="store_false")
    parser.add_argument(
        "--relative-mesh-paths",
        action="store_true",
        help="Write paths relative to each URDF instead of absolute paths",
    )
    parser.add_argument(
        "--use-downsample-mesh",
        action="store_true",
        help=(
            "Use downsampled.obj for collision when available; by default the "
            "visual textured mesh is also used for collision"
        ),
    )
    args = parser.parse_args()

    try:
        models = select_models(discover_models(args.ycb_folder), args.objects)
        outputs = [
            create_ycb_urdf(
                folder,
                overwrite=args.overwrite,
                absolute_mesh_paths=not args.relative_mesh_paths,
                use_downsample_mesh=args.use_downsample_mesh,
            )
            for folder in models
        ]
    except (FileNotFoundError, RuntimeError, ValueError, ET.ParseError) as error:
        parser.error(str(error))
    print(f"Generated {len(outputs)} YCB URDF file(s) in {args.ycb_folder.resolve()}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
