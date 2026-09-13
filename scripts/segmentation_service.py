"""ROS 2 service for saving per-object segmentation masks from the hand-eye camera."""

from __future__ import annotations

import json
import queue
import re
import threading
from dataclasses import dataclass, field
from pathlib import Path


def _safe_filename(value: str) -> str:
    """Convert a semantic label or frame ID into a portable filename."""
    sanitized = re.sub(r"[^A-Za-z0-9_.-]+", "_", value).strip("._")
    return sanitized or "unnamed"


def _next_capture_directory(root: Path, camera_frame: str) -> Path:
    """Allocate a monotonically numbered directory without overwriting a view."""
    camera_directory = root / _safe_filename(camera_frame)
    camera_directory.mkdir(parents=True, exist_ok=True)
    for index in range(1_000_000):
        capture_directory = camera_directory / f"capture_{index:06d}"
        try:
            capture_directory.mkdir()
            return capture_directory
        except FileExistsError:
            continue
    raise RuntimeError(f"No free capture index below {camera_directory}")


@dataclass
class _CaptureRequest:
    save_directory: Path
    camera_frame: str
    completed: threading.Event = field(default_factory=threading.Event)
    capture_directory: Path | None = None
    error: str | None = None
    cancelled: bool = False


@dataclass
class _ActiveCapture:
    request: _CaptureRequest
    camera_prim_path: str
    render_product_path: str
    annotator: object
    rendered_frames: int = 0


class ObjectSegmentationService:
    """Save visible YCB instance masks from the hand-eye camera on request."""

    def __init__(self, object_prim_paths: list[str]) -> None:
        from scripts.simulation_utils import (
            _append_internal_rclpy_path,
            _find_camera_prim_path,
            _load_camera_calibration,
        )

        _append_internal_rclpy_path()
        import rclpy
        from rclpy.executors import (
            ExternalShutdownException,
            ShutdownException,
            SingleThreadedExecutor,
        )
        from rcl_interfaces.srv import SetParametersAtomically
        from env.config import ARM, CAMERA_COLOR_FRAME_LINK, SEGMENTATION_SAVE_SERVICE

        calibration = _load_camera_calibration()
        self._rclpy = rclpy
        self._executor_shutdown_exceptions = (
            ExternalShutdownException,
            ShutdownException,
        )
        self._objects = {
            Path(prim_path).name: prim_path for prim_path in object_prim_paths
        }
        camera_frame = str(calibration["cameras"]["handeye"]["frame_id"])
        camera_prim_path = _find_camera_prim_path(
            ARM, CAMERA_COLOR_FRAME_LINK, "RGBDCameraHandEye"
        )
        self._camera_prims: dict[str, str] = {camera_frame: camera_prim_path}
        self._camera_graphs: dict[str, str] = {camera_frame: "/ROS_Camera_HandEye"}
        self._requests: queue.Queue[_CaptureRequest] = queue.Queue()
        self._active_capture: _ActiveCapture | None = None
        self._closed = False

        self._owns_context = not rclpy.ok()
        if self._owns_context:
            rclpy.init()
        self._node = rclpy.create_node("isaac_object_segmentation_server")
        self._service = self._node.create_service(
            SetParametersAtomically,
            SEGMENTATION_SAVE_SERVICE,
            self._handle_request,
        )
        self._executor = SingleThreadedExecutor()
        self._executor.add_node(self._node)
        self._executor_thread = threading.Thread(
            target=self._spin_executor,
            name="isaac-segmentation-ros-service",
            daemon=True,
        )
        self._executor_thread.start()
        print(
            f"[segmentation_service] Ready: {SEGMENTATION_SAVE_SERVICE}; "
            f"camera frames: {sorted(self._camera_prims)}"
        )

    def _spin_executor(self) -> None:
        """Run the service executor and quietly accept normal ROS shutdown."""
        try:
            self._executor.spin()
        except self._executor_shutdown_exceptions:
            # Ctrl+C may stop Isaac's shared ROS context before close() runs.
            pass
        except Exception:
            # Some rclpy builds wrap context shutdown in a generic exception.
            if self._closed or not self._rclpy.ok():
                return
            raise

    @staticmethod
    def _parse_request(parameters) -> tuple[Path, str]:
        """Extract the two required strings from a parameter-list request."""
        from rcl_interfaces.msg import ParameterType

        values: dict[str, str] = {}
        for parameter in parameters:
            if parameter.name not in ("save_directory", "camera_frame"):
                raise ValueError(f"unknown parameter {parameter.name!r}")
            if parameter.name in values:
                raise ValueError(f"duplicate parameter {parameter.name!r}")
            if parameter.value.type != ParameterType.PARAMETER_STRING:
                raise ValueError(f"{parameter.name!r} must be a string")
            value = parameter.value.string_value.strip()
            if not value:
                raise ValueError(f"{parameter.name!r} cannot be empty")
            values[parameter.name] = value
        missing = {"save_directory", "camera_frame"} - set(values)
        if missing:
            raise ValueError(f"missing parameter(s): {sorted(missing)}")
        save_directory = Path(values["save_directory"]).expanduser().resolve()
        return save_directory, values["camera_frame"]

    @staticmethod
    def _id_to_labels(raw_mapping) -> dict[int, dict]:
        """Normalize Replicator's string- or integer-keyed label mapping."""
        if isinstance(raw_mapping, str):
            raw_mapping = json.loads(raw_mapping)
        if not isinstance(raw_mapping, dict):
            raise RuntimeError("Segmentation annotator returned no ID-to-label map")
        return {int(identifier): labels for identifier, labels in raw_mapping.items()}

    def _start_capture(self, request: _CaptureRequest) -> None:
        """Attach an annotator on the simulation thread for one request."""
        import omni.graph.core as og
        import omni.replicator.core as rep

        camera_prim_path = self._camera_prims[request.camera_frame]
        graph_path = self._camera_graphs[request.camera_frame]
        attribute = og.Controller.attribute(
            f"{graph_path}/CreateRenderProduct.outputs:renderProductPath"
        )
        render_product_path = str(og.Controller.get(attribute) or "")
        if not render_product_path:
            raise RuntimeError(
                f"ROS camera render product is unavailable for "
                f"{request.camera_frame!r}"
            )
        annotator = rep.AnnotatorRegistry.get_annotator(
            "semantic_segmentation",
            init_params={"semanticTypes": ["instance"]},
        )
        annotator.attach([render_product_path])
        self._active_capture = _ActiveCapture(
            request=request,
            camera_prim_path=camera_prim_path,
            render_product_path=render_product_path,
            annotator=annotator,
        )

    def _write_capture(self, capture: _ActiveCapture, result: dict) -> Path:
        """Write one binary mask per visible spawned object and a manifest."""
        import numpy as np
        from omni.replicator.core.functional import write_image

        identifiers = np.asarray(result.get("data"))
        if identifiers.ndim == 3 and identifiers.shape[-1] == 1:
            identifiers = identifiers[..., 0]
        if identifiers.ndim != 2 or identifiers.size == 0:
            raise RuntimeError("Segmentation annotator returned an invalid frame")
        labels_by_id = self._id_to_labels(
            result.get("info", {}).get("idToLabels", {})
        )
        request = capture.request
        capture_directory = _next_capture_directory(
            request.save_directory,
            request.camera_frame,
        )

        # Filter the annotator map to this simulation's spawned YCB instances.
        visible_objects = []
        for identifier, labels in labels_by_id.items():
            if not isinstance(labels, dict):
                continue
            instance_name = str(labels.get("instance", ""))
            prim_path = self._objects.get(instance_name)
            if prim_path is None:
                continue
            mask = np.where(identifiers == identifier, 255, 0).astype(np.uint8)
            visible_pixels = int(np.count_nonzero(mask))
            if visible_pixels == 0:
                continue
            mask_name = f"{_safe_filename(instance_name)}.png"
            write_image(path=str(capture_directory / mask_name), data=mask)
            visible_objects.append(
                {
                    "instance": instance_name,
                    "prim_path": prim_path,
                    "segmentation_id": identifier,
                    "visible_pixels": visible_pixels,
                    "mask": mask_name,
                }
            )

        manifest = {
            "camera_frame": request.camera_frame,
            "camera_prim": capture.camera_prim_path,
            "render_product": capture.render_product_path,
            "resolution": [int(identifiers.shape[1]), int(identifiers.shape[0])],
            "visible_object_count": len(visible_objects),
            "objects": visible_objects,
        }
        (capture_directory / "manifest.json").write_text(
            json.dumps(manifest, indent=2) + "\n",
            encoding="utf-8",
        )
        return capture_directory

    def _finish_capture(self, error: Exception | str | None = None) -> None:
        """Release capture resources and wake the waiting ROS callback."""
        capture = self._active_capture
        if capture is None:
            return
        cleanup_error = None
        try:
            capture.annotator.detach()
        except Exception as cleanup_exception:
            cleanup_error = cleanup_exception
        self._active_capture = None
        final_error = error or cleanup_error
        if final_error is not None:
            capture.request.error = str(final_error)
        capture.request.completed.set()

    def update(self) -> None:
        """Advance an active capture after the normal rendered simulation step."""
        if self._closed:
            return

        # Create render resources only after the service callback queues work.
        if self._active_capture is None:
            try:
                request = self._requests.get_nowait()
            except queue.Empty:
                return
            if request.cancelled:
                request.completed.set()
                return
            try:
                self._start_capture(request)
            except Exception as error:
                request.error = str(error)
                request.completed.set()
            return

        capture = self._active_capture
        if capture.request.cancelled:
            self._finish_capture("segmentation request timed out")
            return
        capture.rendered_frames += 1
        result = capture.annotator.get_data()
        data = result.get("data") if isinstance(result, dict) else None
        if data is None or getattr(data, "size", 0) == 0:
            if capture.rendered_frames < 30:
                return
            self._finish_capture("No segmentation frame was produced after 30 render frames")
            return

        try:
            capture.request.capture_directory = self._write_capture(capture, result)
            self._finish_capture()
        except Exception as error:
            self._finish_capture(error)

    def _handle_request(self, request, response):
        """Queue capture work and wait without blocking the simulation thread."""
        try:
            save_directory, camera_frame = self._parse_request(request.parameters)
            if camera_frame not in self._camera_prims:
                raise ValueError(
                    f"unknown camera frame {camera_frame!r}; expected one of "
                    f"{sorted(self._camera_prims)}"
                )
            capture = _CaptureRequest(save_directory, camera_frame)
            self._requests.put(capture)
            if not capture.completed.wait(timeout=30.0):
                capture.cancelled = True
                raise TimeoutError("segmentation capture timed out after 30 seconds")
            if capture.error is not None:
                raise RuntimeError(capture.error)
            capture_directory = capture.capture_directory
            if capture_directory is None:
                raise RuntimeError("segmentation capture completed without output")
            response.result.successful = True
            response.result.reason = f"saved segmentation to {capture_directory}"
            print(f"[segmentation_service] {response.result.reason}")
        except Exception as error:
            response.result.successful = False
            response.result.reason = str(error)
            print(f"[segmentation_service] Rejected request: {error}")
        return response

    def close(self) -> None:
        """Stop pending work, the ROS executor, and any capture resources."""
        self._closed = True
        if self._active_capture is not None:
            self._finish_capture("segmentation service is shutting down")
        while True:
            try:
                request = self._requests.get_nowait()
            except queue.Empty:
                break
            request.error = "segmentation service is shutting down"
            request.completed.set()
        try:
            self._executor.shutdown(timeout_sec=2.0)
        except self._executor_shutdown_exceptions:
            pass
        except Exception:
            if self._rclpy.ok():
                raise
        self._executor_thread.join(timeout=2.0)
        self._executor.remove_node(self._node)
        self._node.destroy_node()
        if self._owns_context and self._rclpy.ok():
            self._rclpy.shutdown()
