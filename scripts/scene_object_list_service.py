"""ROS 2 service that reports the current YCB object inventory on demand.

Each spawned object's instance name (`object_NNN_<class>`) is assigned at
spawn time by index — it depends on random selection and spawn order, so an
external consumer cannot predict it from a fixed name list the way it can a
model's class name. This service lets such a consumer query the *current*
mapping instead of assuming a name that may be different on the next run.
"""

from __future__ import annotations

import json
from pathlib import Path


class SceneObjectListService:
    """Answer `/list_scene_objects` with the currently spawned object list."""

    def __init__(self, spawned_objects: list[tuple[str, str]]) -> None:
        from scripts.simulation_utils import _append_internal_rclpy_path

        _append_internal_rclpy_path()
        import rclpy
        from std_srvs.srv import Trigger
        from env.config import SCENE_OBJECT_LIST_SERVICE

        self._rclpy = rclpy
        self._objects = [
            {
                "class_name": class_name,
                "instance_name": Path(prim_path).name,
                "prim_path": prim_path,
            }
            for class_name, prim_path in spawned_objects
        ]

        self._owns_context = not rclpy.ok()
        if self._owns_context:
            rclpy.init()
        self._node = rclpy.create_node("isaac_scene_object_list_server")
        self._service = self._node.create_service(
            Trigger,
            SCENE_OBJECT_LIST_SERVICE,
            self._handle_request,
        )
        print(
            f"[scene_object_list_service] Ready: {SCENE_OBJECT_LIST_SERVICE}; "
            f"{len(self._objects)} object(s)"
        )

    def _handle_request(self, request, response):
        response.success = True
        response.message = json.dumps(self._objects)
        return response

    def spin_once(self) -> None:
        """Process pending requests without blocking simulation rendering."""
        self._rclpy.spin_once(self._node, timeout_sec=0.0)

    def close(self) -> None:
        """Release the ROS node and the context it initialized."""
        self._node.destroy_node()
        if self._owns_context and self._rclpy.ok():
            self._rclpy.shutdown()
