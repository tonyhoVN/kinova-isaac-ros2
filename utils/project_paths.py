from __future__ import annotations

from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[1]
ENV_DIR = PROJECT_ROOT / "env"
SCRIPTS_DIR = PROJECT_ROOT / "scripts"
README_PATH = PROJECT_ROOT / "README.md"
SCENE_USD_PATH = ENV_DIR / "kinova_table_generated.usda"


def ensure_project_dirs() -> None:
    """Create the standard project folders if they do not exist."""
    ENV_DIR.mkdir(parents=True, exist_ok=True)
    SCRIPTS_DIR.mkdir(parents=True, exist_ok=True)
