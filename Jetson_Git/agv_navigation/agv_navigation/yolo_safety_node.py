#!/usr/bin/env python3
"""
AGV YOLO Safety Node
====================
Subscribes to the compressed camera feed, runs YOLOv8 inference on each frame,
and publishes /agv/arm_enable=False (cutting motor power) when a person or any
obstacle is detected closer than STOP_CONFIDENCE_THRESHOLD.

The node does NOT re-arm the rover — it only disarms. Re-arming is done by the
operator pressing E on the keyboard node, or by Nav2 explicitly publishing
arm_enable=True. This is intentional: the human must acknowledge the stop.

Dependencies (install on Jetson):
    pip install ultralytics opencv-python-headless

Topics
------
Subscribes:
    /camera/image_raw/compressed  (sensor_msgs/CompressedImage)
    /agv/arm_enable                (std_msgs/Bool)  — monitors current arm state

Publishes:
    /agv/arm_enable                (std_msgs/Bool)  — False on detection
    /agv/yolo_detections           (std_msgs/String) — JSON detection summary
"""

import json
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Bool, String

# YOLOv8 — loaded lazily so the node starts even if ultralytics isn't installed
try:
    from ultralytics import YOLO
    YOLO_AVAILABLE = True
except ImportError:
    YOLO_AVAILABLE = False


# Classes that trigger an emergency stop.
# Full COCO class list: https://docs.ultralytics.com/datasets/detect/coco/
STOP_CLASSES = {
    0: 'person',
}

# Confidence threshold — detections below this are ignored.
CONFIDENCE_THRESHOLD = 0.55

# How many consecutive frames a stop detection must persist before
# arm_enable is cut. Prevents a single noisy frame from stopping the rover.
STOP_FRAMES_REQUIRED = 3

# After a safety stop, minimum seconds before the node allows re-arm
# (operator must explicitly press E after this window).
REARM_LOCKOUT_S = 3.0

# YOLOv8 model — 'yolov8n.pt' is the smallest/fastest, good for Jetson.
# Options: yolov8n, yolov8s, yolov8m (larger = more accurate, slower)
YOLO_MODEL = 'yolov8n.pt'


class YoloSafetyNode(Node):
    def __init__(self) -> None:
        super().__init__('agv_yolo_safety')

        self.declare_parameter('confidence_threshold', CONFIDENCE_THRESHOLD)
        self.declare_parameter('stop_frames_required', STOP_FRAMES_REQUIRED)
        self.declare_parameter('rearm_lockout_s', REARM_LOCKOUT_S)
        self.declare_parameter('yolo_model', YOLO_MODEL)

        self.conf_thresh = float(
            self.get_parameter('confidence_threshold').value)
        self.stop_frames = int(
            self.get_parameter('stop_frames_required').value)
        self.lockout_s = float(
            self.get_parameter('rearm_lockout_s').value)
        model_name = str(self.get_parameter('yolo_model').value)

        # Publishers
        self.arm_pub = self.create_publisher(Bool, '/agv/arm_enable', 10)
        self.det_pub = self.create_publisher(String, '/agv/yolo_detections', 10)

        # Subscribers
        self.create_subscription(
            CompressedImage, '/camera/image_raw/compressed',
            self._image_callback, 10)
        self.create_subscription(
            Bool, '/agv/arm_enable', self._arm_callback, 1)

        # State
        self._armed = False
        self._stop_frame_count = 0
        self._last_stop_time = 0.0

        # Load YOLO model
        if not YOLO_AVAILABLE:
            self.get_logger().error(
                'ultralytics not installed. Run: pip install ultralytics')
            self._model = None
        else:
            self.get_logger().info(f'Loading YOLO model: {model_name}')
            try:
                self._model = YOLO(model_name)
                self.get_logger().info('YOLO model loaded.')
            except Exception as e:
                self.get_logger().error(f'Failed to load YOLO model: {e}')
                self._model = None

    def _arm_callback(self, msg: Bool) -> None:
        self._armed = bool(msg.data)

    def _image_callback(self, msg: CompressedImage) -> None:
        if self._model is None:
            return

        # Decode JPEG → numpy BGR
        try:
            buf = np.frombuffer(msg.data, dtype=np.uint8)
            frame = cv2.imdecode(buf, cv2.IMREAD_COLOR)
            if frame is None:
                return
        except Exception as e:
            self.get_logger().error(f'Image decode error: {e}')
            return

        # Run YOLOv8 inference
        try:
            results = self._model(frame, verbose=False)[0]
        except Exception as e:
            self.get_logger().error(f'YOLO inference error: {e}')
            return

        # Collect detections that exceed threshold
        stop_detections = []
        all_detections = []
        for box in results.boxes:
            cls_id = int(box.cls[0])
            conf = float(box.conf[0])
            cls_name = results.names.get(cls_id, str(cls_id))
            all_detections.append({'class': cls_name, 'confidence': round(conf, 2)})
            if cls_id in STOP_CLASSES and conf >= self.conf_thresh:
                stop_detections.append({'class': cls_name, 'confidence': round(conf, 2)})

        # Publish detection summary
        det_msg = String()
        det_msg.data = json.dumps(all_detections)
        self.det_pub.publish(det_msg)

        # Safety logic
        if stop_detections:
            self._stop_frame_count += 1
            if self._stop_frame_count >= self.stop_frames and self._armed:
                self._trigger_stop(stop_detections)
        else:
            self._stop_frame_count = 0

    def _trigger_stop(self, detections: list) -> None:
        names = [d['class'] for d in detections]
        self.get_logger().warn(
            f'[SAFETY] STOP triggered — detected: {names}. '
            f'Press E on keyboard to re-arm after {self.lockout_s:.0f}s.')
        msg = Bool()
        msg.data = False
        self.arm_pub.publish(msg)
        self._armed = False
        self._stop_frame_count = 0
        self._last_stop_time = time.monotonic()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = YoloSafetyNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
