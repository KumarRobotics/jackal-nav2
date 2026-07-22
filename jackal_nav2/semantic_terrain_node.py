"""ROS 2 frontend for synchronized SAM3 semantic terrain observations."""

from __future__ import annotations

from dataclasses import dataclass
import threading
import time
from typing import Callable

from cv_bridge import CvBridge
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from jackal_nav2.sam3_backend import UltralyticsSam3Backend
from jackal_nav2.semantic_terrain import (
    DEFAULT_PROMPT_CLASS_IDS,
    DEFAULT_PROMPT_CLASS_NAMES,
    DEFAULT_PROMPTS,
    LatestFrameMailbox,
    organized_pointcloud2,
    PromptClassConfig,
    render_overlay,
    SegmentationBackend,
    SemanticTerrainProcessor,
)
from jackal_nav2.semantic_terrain_sampling import resample_aligned

import message_filters
import numpy as np
from rcl_interfaces.msg import ParameterDescriptor
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image, PointCloud2
from std_msgs.msg import Header
from std_srvs.srv import SetBool
from vision_msgs.msg import LabelInfo, VisionClass


@dataclass(frozen=True)
class SynchronizedFrame:
    """One approximately synchronized RGB-D observation."""

    rgb: Image
    depth: Image
    camera_info: CameraInfo


BackendFactory = Callable[..., SegmentationBackend]


def _copy_header(source: Header, *, frame_id: str | None = None) -> Header:
    header = Header()
    header.stamp.sec = source.stamp.sec
    header.stamp.nanosec = source.stamp.nanosec
    header.frame_id = source.frame_id if frame_id is None else frame_id
    return header


class SemanticTerrainNode(Node):
    """Generate costmap-ready class, confidence, and organized XYZ observations."""

    def __init__(
        self,
        *,
        backend: SegmentationBackend | None = None,
        backend_factory: BackendFactory | None = None,
    ):
        super().__init__("semantic_terrain")

        self.declare_parameter("model_path", "")
        self.declare_parameter("device", "cuda")
        self.declare_parameter("image_size", 640)
        self.declare_parameter("model_confidence", 0.25)
        self.declare_parameter("half_precision", True)
        self.declare_parameter("score_threshold", 0.5)
        class_mapping_descriptor = ParameterDescriptor(
            description="Startup-static semantic prompt-to-class mapping",
            read_only=True,
        )
        self.declare_parameter(
            "prompts",
            list(DEFAULT_PROMPTS),
            descriptor=class_mapping_descriptor,
        )
        self.declare_parameter(
            "prompt_class_ids",
            list(DEFAULT_PROMPT_CLASS_IDS),
            descriptor=class_mapping_descriptor,
        )
        self.declare_parameter(
            "prompt_class_names",
            list(DEFAULT_PROMPT_CLASS_NAMES),
            descriptor=class_mapping_descriptor,
        )
        self.declare_parameter(
            "rgb_topic", "zed/zed_node/left/image_rect_color"
        )
        self.declare_parameter(
            "depth_topic", "zed/zed_node/depth/depth_registered"
        )
        self.declare_parameter(
            "camera_info_topic", "zed/zed_node/depth/camera_info"
        )
        self.declare_parameter("sync_queue_size", 10)
        self.declare_parameter("sync_slop_seconds", 0.05)
        self.declare_parameter("integer_depth_scale", 0.001)
        self.declare_parameter("max_inference_rate_hz", 0.0)
        self.declare_parameter("output_width", 320)
        self.declare_parameter("output_height", 180)
        self.declare_parameter("min_depth_m", 0.5)
        self.declare_parameter("max_depth_m", 8.0)
        self.declare_parameter("start_enabled", True)
        self.declare_parameter("publish_overlay", False)
        self.declare_parameter("overlay_alpha", 0.5)
        self.declare_parameter("publish_diagnostics", True)

        prompts = list(self.get_parameter("prompts").value)
        prompt_class_ids = list(self.get_parameter("prompt_class_ids").value)
        prompt_class_names = list(self.get_parameter("prompt_class_names").value)
        self._class_config = PromptClassConfig.from_sequences(
            prompts, prompt_class_ids, prompt_class_names
        )

        score_threshold = float(self.get_parameter("score_threshold").value)
        integer_depth_scale = float(self.get_parameter("integer_depth_scale").value)
        self._max_inference_rate_hz = float(
            self.get_parameter("max_inference_rate_hz").value
        )
        if self._max_inference_rate_hz < 0.0:
            raise ValueError("max_inference_rate_hz must be non-negative; 0 is uncapped")
        self._overlay_alpha = float(self.get_parameter("overlay_alpha").value)
        if not 0.0 <= self._overlay_alpha <= 1.0:
            raise ValueError("overlay_alpha must be in [0.0, 1.0]")

        sync_queue_size = int(self.get_parameter("sync_queue_size").value)
        sync_slop_seconds = float(self.get_parameter("sync_slop_seconds").value)
        if sync_queue_size <= 0:
            raise ValueError("sync_queue_size must be positive")
        if sync_slop_seconds < 0.0:
            raise ValueError("sync_slop_seconds must be non-negative")

        if backend is None:
            factory = backend_factory or UltralyticsSam3Backend
            backend = factory(
                model_path=str(self.get_parameter("model_path").value),
                device=str(self.get_parameter("device").value),
                image_size=int(self.get_parameter("image_size").value),
                model_confidence=float(
                    self.get_parameter("model_confidence").value
                ),
                half_precision=bool(
                    self.get_parameter("half_precision").value
                ),
            )
        self._backend = backend
        self._processor = SemanticTerrainProcessor(
            backend,
            self._class_config,
            score_threshold=score_threshold,
            integer_depth_scale=integer_depth_scale,
            output_width=int(self.get_parameter("output_width").value),
            output_height=int(self.get_parameter("output_height").value),
            min_depth_m=float(self.get_parameter("min_depth_m").value),
            max_depth_m=float(self.get_parameter("max_depth_m").value),
        )

        sensor_output_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        label_info_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        sensor_input_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=sync_queue_size,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )

        self._label_mask_pub = self.create_publisher(
            Image, "~/label_mask", sensor_output_qos
        )
        self._confidence_pub = self.create_publisher(
            Image, "~/confidence", sensor_output_qos
        )
        self._label_info_pub = self.create_publisher(
            LabelInfo, "~/label_info", label_info_qos
        )
        self._points_pub = self.create_publisher(
            PointCloud2, "~/points", sensor_output_qos
        )
        self._publish_overlay_enabled = bool(
            self.get_parameter("publish_overlay").value
        )
        self._overlay_pub = (
            self.create_publisher(Image, "~/overlay", sensor_output_qos)
            if self._publish_overlay_enabled
            else None
        )
        self._publish_diagnostics_enabled = bool(
            self.get_parameter("publish_diagnostics").value
        )
        self._diagnostics_pub = (
            self.create_publisher(DiagnosticArray, "~/diagnostics", 10)
            if self._publish_diagnostics_enabled
            else None
        )
        self._enable_service = self.create_service(
            SetBool, "~/enable", self._on_enable
        )

        self._bridge = CvBridge()
        self._state_lock = threading.Lock()
        self._enabled = bool(self.get_parameter("start_enabled").value)
        self._processed_frames = 0
        self._processing_errors = 0
        self._last_processing_ms = 0.0
        self._last_instance_count = 0
        self._destroying = False
        self._stop_event = threading.Event()
        self._mailbox: LatestFrameMailbox[SynchronizedFrame] = LatestFrameMailbox()
        self._last_inference_ms = 0.0
        self._worker = threading.Thread(
            target=self._worker_loop,
            name="semantic-terrain-worker",
            daemon=True,
        )
        self._worker.start()

        rgb_topic = str(self.get_parameter("rgb_topic").value)
        depth_topic = str(self.get_parameter("depth_topic").value)
        camera_info_topic = str(self.get_parameter("camera_info_topic").value)
        self._rgb_sub = message_filters.Subscriber(
            self, Image, rgb_topic, qos_profile=sensor_input_qos
        )
        self._depth_sub = message_filters.Subscriber(
            self, Image, depth_topic, qos_profile=sensor_input_qos
        )
        self._camera_info_sub = message_filters.Subscriber(
            self, CameraInfo, camera_info_topic, qos_profile=sensor_input_qos
        )
        self._synchronizer = message_filters.ApproximateTimeSynchronizer(
            [self._rgb_sub, self._depth_sub, self._camera_info_sub],
            queue_size=sync_queue_size,
            slop=sync_slop_seconds,
        )
        self._synchronizer.registerCallback(self._on_synchronized_frame)

        self._publish_label_info()
        class_summary = ", ".join(
            f"{class_id}:{name}"
            for class_id, name in self._class_config.unique_classes
        )
        rate_summary = (
            "uncapped"
            if self._max_inference_rate_hz == 0.0
            else f"{self._max_inference_rate_hz:.2f} Hz"
        )
        self.get_logger().info(
            "Semantic terrain ready: "
            f"RGB={rgb_topic}, depth={depth_topic}, info={camera_info_topic}, "
            f"rate={rate_summary}, classes=[{class_summary}], enabled={self._enabled}"
        )

    def _is_enabled(self) -> bool:
        with self._state_lock:
            return self._enabled and not self._stop_event.is_set()

    def _on_enable(self, request: SetBool.Request, response: SetBool.Response):
        with self._state_lock:
            self._enabled = bool(request.data)
            enabled = self._enabled
        if not enabled:
            self._mailbox.clear()
        state = "enabled" if enabled else "disabled"
        response.success = True
        response.message = f"Semantic terrain inference {state}"
        self.get_logger().info(response.message)
        self._publish_diagnostics(DiagnosticStatus.OK, response.message)
        return response

    def _on_synchronized_frame(
        self, rgb: Image, depth: Image, camera_info: CameraInfo
    ) -> None:
        if not self._is_enabled():
            return
        self._mailbox.put(SynchronizedFrame(rgb, depth, camera_info))

    def _worker_loop(self) -> None:
        last_start = 0.0
        while not self._stop_event.is_set():
            frame = self._mailbox.take()
            if frame is None:
                return

            if self._max_inference_rate_hz > 0.0 and last_start > 0.0:
                period = 1.0 / self._max_inference_rate_hz
                delay = last_start + period - time.monotonic()
                if delay > 0.0 and self._stop_event.wait(delay):
                    return
                newer_frame = self._mailbox.take_nowait()
                if newer_frame is not None:
                    self._mailbox.note_discarded()
                    frame = newer_frame

            if not self._is_enabled():
                continue
            last_start = time.monotonic()
            try:
                self._process_frame(frame)
            except Exception as exc:  # keep the worker alive after a malformed frame
                if self._stop_event.is_set():
                    return
                with self._state_lock:
                    self._processing_errors += 1
                self.get_logger().error(f"Semantic terrain frame failed: {exc}")
                self._publish_diagnostics(
                    DiagnosticStatus.ERROR, f"Frame processing failed: {exc}"
                )

    def _process_frame(self, frame: SynchronizedFrame) -> None:
        started = time.monotonic()
        bgr = self._bridge.imgmsg_to_cv2(frame.rgb, desired_encoding="bgr8")
        depth = self._bridge.imgmsg_to_cv2(frame.depth, desired_encoding="passthrough")
        bgr_array = np.asarray(bgr, dtype=np.uint8)
        depth_array = np.asarray(depth)
        source_height, source_width = bgr_array.shape[:2]
        depth_height, depth_width = depth_array.shape[:2]
        info_width = int(frame.camera_info.width)
        info_height = int(frame.camera_info.height)
        if info_width and (
            info_width != source_width or info_width != depth_width
        ):
            raise ValueError(
                "CameraInfo width does not match the synchronized RGB-D resolution"
            )
        if info_height and (
            info_height != source_height or info_height != depth_height
        ):
            raise ValueError(
                "CameraInfo height does not match the synchronized RGB-D resolution"
            )
        result = self._processor.process(bgr_array, depth_array, frame.camera_info.k)

        if not self._is_enabled():
            return
        common_header = _copy_header(frame.rgb.header)
        point_frame = (
            frame.camera_info.header.frame_id
            or frame.depth.header.frame_id
            or frame.rgb.header.frame_id
        )
        point_header = _copy_header(frame.rgb.header, frame_id=point_frame)

        label_msg = self._bridge.cv2_to_imgmsg(result.label_mask, encoding="mono8")
        label_msg.header = _copy_header(common_header)
        confidence_msg = self._bridge.cv2_to_imgmsg(
            result.confidence, encoding="mono8"
        )
        confidence_msg.header = _copy_header(common_header)
        points_msg = organized_pointcloud2(result.xyz, point_header)

        # All three messages use the RGB stamp so ExactTime synchronization remains
        # available to the semantic costmap even when input synchronization is approximate.
        self._label_mask_pub.publish(label_msg)
        self._confidence_pub.publish(confidence_msg)
        self._points_pub.publish(points_msg)

        if self._overlay_pub is not None and self._overlay_pub.get_subscription_count() > 0:
            (sampled_bgr,) = resample_aligned(
                (bgr_array,),
                output_height=result.label_mask.shape[0],
                output_width=result.label_mask.shape[1],
            )
            overlay = render_overlay(
                sampled_bgr, result.label_mask, alpha=self._overlay_alpha
            )
            overlay_msg = self._bridge.cv2_to_imgmsg(overlay, encoding="bgr8")
            overlay_msg.header = _copy_header(common_header)
            self._overlay_pub.publish(overlay_msg)

        elapsed_ms = (time.monotonic() - started) * 1000.0
        with self._state_lock:
            self._processed_frames += 1
            self._last_processing_ms = elapsed_ms
            self._last_inference_ms = result.inference_seconds * 1000.0
            self._last_instance_count = result.accepted_instance_count
        self._publish_diagnostics(DiagnosticStatus.OK, "Semantic terrain active")

    def _publish_label_info(self) -> None:
        message = LabelInfo()
        message.header.stamp = self.get_clock().now().to_msg()
        message.class_map = []
        for class_id, name in self._class_config.unique_classes:
            vision_class = VisionClass()
            vision_class.class_id = class_id
            vision_class.class_name = name
            message.class_map.append(vision_class)
        message.threshold = self._processor.score_threshold
        self._label_info_pub.publish(message)

    def _publish_diagnostics(self, level: int, message: str) -> None:
        if self._diagnostics_pub is None:
            return
        with self._state_lock:
            enabled = self._enabled
            processed = self._processed_frames
            errors = self._processing_errors
            processing_ms = self._last_processing_ms
            inference_ms = self._last_inference_ms
            instances = self._last_instance_count
        inference_fps = 1000.0 / inference_ms if inference_ms > 0.0 else 0.0
        processing_fps = 1000.0 / processing_ms if processing_ms > 0.0 else 0.0
        status = DiagnosticStatus()
        status.level = level
        status.name = f"{self.get_fully_qualified_name()}: semantic terrain"
        status.hardware_id = str(self.get_parameter("device").value)
        status.message = message
        status.values = [
            KeyValue(key="enabled", value=str(enabled).lower()),
            KeyValue(key="processed_frames", value=str(processed)),
            KeyValue(key="dropped_frames", value=str(self._mailbox.dropped)),
            KeyValue(key="processing_errors", value=str(errors)),
            KeyValue(key="last_processing_ms", value=f"{processing_ms:.3f}"),
            KeyValue(key="last_inference_ms", value=f"{inference_ms:.3f}"),
            KeyValue(key="last_inference_fps", value=f"{inference_fps:.3f}"),
            KeyValue(key="last_processing_fps", value=f"{processing_fps:.3f}"),
            KeyValue(key="last_instance_count", value=str(instances)),
        ]
        array = DiagnosticArray()
        array.header.stamp = self.get_clock().now().to_msg()
        array.status = [status]
        self._diagnostics_pub.publish(array)

    def destroy_node(self):
        """Stop the worker before destroying ROS entities and backend resources."""
        if not self._destroying:
            self._destroying = True
            with self._state_lock:
                self._enabled = False
            self._stop_event.set()
            self._mailbox.close()
            self._worker.join(timeout=5.0)
            if self._worker.is_alive():
                self.get_logger().warn(
                    "Semantic terrain worker did not stop within five seconds"
                )
            else:
                close = getattr(self._backend, "close", None)
                if close is not None:
                    close()
        return super().destroy_node()


def main(args=None):
    """Run the semantic terrain node with concurrent ROS callbacks."""
    rclpy.init(args=args)
    node = None
    try:
        node = SemanticTerrainNode()
        executor = MultiThreadedExecutor(num_threads=2)
        executor.add_node(node)
        executor.spin()
    except KeyboardInterrupt:
        pass
    except Exception as exc:
        rclpy.logging.get_logger("semantic_terrain").fatal(str(exc))
        raise
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
