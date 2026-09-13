from __future__ import annotations

import json
import os
import tempfile
import time
from pathlib import Path
from typing import Any


STATE_FILE_PATH = Path(os.environ.get("ISAACSIM_STATE_PATH", "/tmp/isaacsim_object_state.json"))


def write_state(
    path: Path,
    frame: int,
    objects: dict[str, dict[str, Any]],
    timestamp: float | None = None,
) -> None:
    """Write a single simulation snapshot atomically."""
    snapshot = {
        "frame": frame,
        "timestamp": time.time() if timestamp is None else timestamp,
        "objects": objects,
    }

    path.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.NamedTemporaryFile("w", delete=False, dir=path.parent, encoding="utf-8") as tmp:
        json.dump(snapshot, tmp, indent=2)
        tmp.write("\n")
        tmp_path = Path(tmp.name)

    tmp_path.replace(path)


def read_state(path: Path) -> dict[str, Any] | None:
    """Read a simulation snapshot if it exists."""
    if not path.exists():
        return None
    try:
        return json.loads(path.read_text(encoding="utf-8"))
    except json.JSONDecodeError:
        return None
