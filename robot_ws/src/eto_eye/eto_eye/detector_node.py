import os
import ast
import json
import time
from collections import Counter
from pathlib import Path

import cv2
import numpy as np
import onnxruntime as ort
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import String

PROVIDERS_MAP = {
    "cpu": ["CPUExecutionProvider"],
    "migraphx": ["MIGraphXExecutionProvider"],
}

OUTPUT_FORMATS = {"vision_msgs", "json"}


class DetectorNode(Node):
    def __init__(self):
        super().__init__("eto_eye_detector")

        self.declare_parameter("model_path", os.getenv("MODEL_PATH", "/models/best.onnx"))
        self.declare_parameter("ep", os.getenv("ORT_EP", "cpu"))
        self.declare_parameter(
            "image_topic",
            os.getenv("IMAGE_TOPIC", "/camera/color/image_raw"),
        )
        self.declare_parameter("detection_topic", os.getenv("DETECTION_TOPIC", "/detections"))
        self.declare_parameter("output_format", os.getenv("DETECTION_FORMAT", "vision_msgs"))
        self.declare_parameter(
            "annotated_topic",
            os.getenv("ANNOTATED_TOPIC", "/eto_eye/annotated_image/compressed"),
        )
        self.declare_parameter(
            "enable_annotated_image",
            os.getenv("ENABLE_ANNOTATED_IMAGE", "false").lower() == "true",
        )
        self.declare_parameter("annotated_max_fps", float(os.getenv("ANNOTATED_MAX_FPS", "5.0")))
        self.declare_parameter("annotated_scale", float(os.getenv("ANNOTATED_SCALE", "0.5")))
        self.declare_parameter("annotated_jpeg_quality", int(os.getenv("ANNOTATED_JPEG_QUALITY", "80")))
        self.declare_parameter(
            "enable_detection_logging",
            os.getenv("ENABLE_DETECTION_LOGGING", "false").lower() == "true",
        )
        self.declare_parameter(
            "detection_log_interval",
            float(os.getenv("DETECTION_LOG_INTERVAL", "5.0")),
        )
        self.declare_parameter(
            "enable_depth",
            os.getenv("ENABLE_DEPTH", "false").lower() == "true",
        )
        self.declare_parameter(
            "depth_topic",
            os.getenv("DEPTH_TOPIC", "/depth_to_rgb/image_raw"),
        )
        self.declare_parameter("depth_unit_scale", float(os.getenv("DEPTH_UNIT_SCALE", "0.001")))
        self.declare_parameter("depth_roi_scale", float(os.getenv("DEPTH_ROI_SCALE", "0.5")))
        self.declare_parameter(
            "depth_min_valid_pixels",
            int(os.getenv("DEPTH_MIN_VALID_PIXELS", "20")),
        )
        self.declare_parameter("depth_max_age_sec", float(os.getenv("DEPTH_MAX_AGE_SEC", "0.5")))
        self.declare_parameter("input_size", 640)
        self.declare_parameter("conf_threshold", 0.25)
        self.declare_parameter("iou_threshold", 0.45)

        model_path = Path(self.get_parameter("model_path").value)
        if not model_path.is_file():
            raise FileNotFoundError(f"Model file not found: {model_path}")

        ep = self.get_parameter("ep").value
        if ep not in PROVIDERS_MAP:
            raise ValueError(f"Unknown EP '{ep}'. Choose one of {list(PROVIDERS_MAP)}")

        self.input_size = int(self.get_parameter("input_size").value)
        self.conf_threshold = float(self.get_parameter("conf_threshold").value)
        self.iou_threshold = float(self.get_parameter("iou_threshold").value)
        self.output_format = self.get_parameter("output_format").value
        self.enable_annotated_image = bool(
            self.get_parameter("enable_annotated_image").value
        )
        self.annotated_max_fps = float(self.get_parameter("annotated_max_fps").value)
        self.annotated_scale = float(self.get_parameter("annotated_scale").value)
        self.annotated_jpeg_quality = int(
            self.get_parameter("annotated_jpeg_quality").value
        )
        self.enable_detection_logging = bool(
            self.get_parameter("enable_detection_logging").value
        )
        self.detection_log_interval = float(
            self.get_parameter("detection_log_interval").value
        )
        self.enable_depth = bool(self.get_parameter("enable_depth").value)
        self.depth_unit_scale = float(self.get_parameter("depth_unit_scale").value)
        self.depth_roi_scale = float(self.get_parameter("depth_roi_scale").value)
        self.depth_min_valid_pixels = int(
            self.get_parameter("depth_min_valid_pixels").value
        )
        self.depth_max_age_sec = float(self.get_parameter("depth_max_age_sec").value)
        if self.depth_roi_scale <= 0.0 or self.depth_roi_scale > 1.0:
            raise ValueError("depth_roi_scale must be in the range (0.0, 1.0]")
        self._last_annotated_publish = 0.0
        self._last_detection_log = time.monotonic()
        self._log_frames = 0
        self._log_frames_with_detections = 0
        self._log_total_detections = 0
        self._log_class_counts = Counter()
        self._log_best_by_class = {}
        if self.output_format not in OUTPUT_FORMATS:
            raise ValueError(
                f"Unknown output_format '{self.output_format}'. "
                f"Choose one of {sorted(OUTPUT_FORMATS)}"
            )

        providers = self._resolve_providers(ep)
        self._prepare_migraphx_cache(ep)
        self.get_logger().info(f"Loading {model_path} with EP={ep}")
        self.session = ort.InferenceSession(str(model_path), providers=providers)
        self.class_names = self._load_class_names()
        self.input_name = self.session.get_inputs()[0].name
        active = self.session.get_providers()
        if ep == "migraphx" and "MIGraphXExecutionProvider" not in active:
            raise RuntimeError(f"MIGraphX was requested but active providers are {active}")
        self.get_logger().info(f"Active providers: {active}")
        if self.class_names:
            self.get_logger().info(f"Class names: {self.class_names}")

        self.bridge = CvBridge()
        self.latest_depth = None
        self.latest_depth_stamp_sec = None
        self.latest_depth_encoding = None
        self.latest_depth_frame_id = None

        image_topic = self.get_parameter("image_topic").value
        detection_topic = self.get_parameter("detection_topic").value
        self.subscription = self.create_subscription(
            Image, image_topic, self.image_callback, 10
        )
        if self.enable_depth:
            depth_topic = self.get_parameter("depth_topic").value
            self.depth_subscription = self.create_subscription(
                Image, depth_topic, self.depth_callback, 10
            )
            self.get_logger().info(
                f"Subscribed to depth topic {depth_topic} "
                f"scale={self.depth_unit_scale} roi_scale={self.depth_roi_scale}"
            )
        else:
            self.depth_subscription = None
        if self.output_format == "vision_msgs":
            from vision_msgs.msg import (
                Detection2DArray,
            )

            self.publisher = self.create_publisher(Detection2DArray, detection_topic, 10)
        else:
            self.publisher = self.create_publisher(String, detection_topic, 10)
        if self.enable_annotated_image:
            annotated_topic = self.get_parameter("annotated_topic").value
            self.annotated_publisher = self.create_publisher(
                CompressedImage, annotated_topic, 10
            )
            self.get_logger().info(
                f"Publishing annotated preview to {annotated_topic}"
            )
        else:
            self.annotated_publisher = None
        self.get_logger().info(
            f"Subscribed to {image_topic}, publishing {self.output_format} to {detection_topic}"
        )

    def _prepare_migraphx_cache(self, ep):
        if ep != "migraphx":
            return
        env_name = "ORT_MIGRAPHX_MODEL_CACHE_PATH"
        cache_path = os.getenv(env_name)
        if not cache_path:
            return
        try:
            Path(cache_path).mkdir(parents=True, exist_ok=True)
        except OSError as exc:
            raise RuntimeError(f"Failed to create {env_name}={cache_path}: {exc}") from exc
        self.get_logger().info(f"Using {env_name}={cache_path}")

    def _load_class_names(self):
        names = self.session.get_modelmeta().custom_metadata_map.get("names")
        if not names:
            return {}
        try:
            parsed = ast.literal_eval(names)
        except (SyntaxError, ValueError):
            self.get_logger().warning(f"Failed to parse model class names: {names}")
            return {}
        if isinstance(parsed, dict):
            return {int(class_id): str(name) for class_id, name in parsed.items()}
        if isinstance(parsed, (list, tuple)):
            return {class_id: str(name) for class_id, name in enumerate(parsed)}
        return {}

    def _class_name(self, class_id):
        return self.class_names.get(int(class_id), str(class_id))

    def _resolve_providers(self, ep):
        requested = PROVIDERS_MAP[ep]
        available = ort.get_available_providers()
        missing = [provider for provider in requested if provider not in available]
        if missing:
            raise RuntimeError(
                f"Requested EP '{ep}' needs providers {requested}, "
                f"but ONNX Runtime only has {available}. "
                "For AMD GPU on ROCm 7.1+, use the MIGraphX ONNX Runtime wheel."
            )
        return requested

    def depth_callback(self, msg: Image):
        depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        self.latest_depth = np.asarray(depth)
        self.latest_depth_stamp_sec = self._stamp_to_sec(msg.header.stamp)
        self.latest_depth_encoding = msg.encoding
        self.latest_depth_frame_id = msg.header.frame_id

    def image_callback(self, msg: Image):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        tensor, scale, pad = self._preprocess(frame)
        outputs = self.session.run(None, {self.input_name: tensor})
        detections = self._postprocess(
            outputs, scale, pad, frame.shape, include_masks=self.enable_annotated_image
        )
        if self.enable_depth:
            self._attach_depth(msg, detections, frame.shape)

        if self.output_format == "vision_msgs":
            self.publisher.publish(self._to_vision_msg(msg, detections))
        else:
            self.publisher.publish(self._to_json_msg(msg, detections))
        if self.annotated_publisher is not None:
            self._publish_annotated(msg, frame, detections)
        if self.enable_detection_logging:
            self._log_detection_summary(detections)

    def _preprocess(self, frame):
        h, w = frame.shape[:2]
        size = self.input_size
        scale = min(size / h, size / w)
        new_w, new_h = int(round(w * scale)), int(round(h * scale))
        resized = cv2.resize(frame, (new_w, new_h), interpolation=cv2.INTER_LINEAR)
        pad_w = (size - new_w) // 2
        pad_h = (size - new_h) // 2
        canvas = np.full((size, size, 3), 114, dtype=np.uint8)
        canvas[pad_h : pad_h + new_h, pad_w : pad_w + new_w] = resized
        rgb = cv2.cvtColor(canvas, cv2.COLOR_BGR2RGB)
        tensor = rgb.astype(np.float32) / 255.0
        tensor = tensor.transpose(2, 0, 1)[None, ...]
        return tensor, scale, (pad_w, pad_h)

    def _postprocess(self, outputs, scale, pad, original_shape, include_masks=False):
        if not isinstance(outputs, (list, tuple)):
            outputs = [outputs]

        pred = self._prediction_matrix(outputs[0])
        protos = self._segmentation_protos(outputs)
        mask_dim = protos.shape[0] if protos is not None else 0
        num_classes = pred.shape[1] - 4 - mask_dim
        if num_classes <= 0:
            raise RuntimeError(
                f"Unexpected YOLO output shape {outputs[0].shape}; "
                f"mask_dim={mask_dim}, inferred num_classes={num_classes}"
            )

        boxes_cxcywh = pred[:, :4]
        scores = pred[:, 4 : 4 + num_classes]
        mask_coeffs = pred[:, 4 + num_classes :] if mask_dim else None

        max_scores = scores.max(axis=1)
        class_ids = scores.argmax(axis=1)
        keep = max_scores > self.conf_threshold
        boxes_cxcywh = boxes_cxcywh[keep]
        max_scores = max_scores[keep]
        class_ids = class_ids[keep]
        if mask_coeffs is not None:
            mask_coeffs = mask_coeffs[keep]

        if len(boxes_cxcywh) == 0:
            return []

        x1 = boxes_cxcywh[:, 0] - boxes_cxcywh[:, 2] / 2
        y1 = boxes_cxcywh[:, 1] - boxes_cxcywh[:, 3] / 2
        x2 = boxes_cxcywh[:, 0] + boxes_cxcywh[:, 2] / 2
        y2 = boxes_cxcywh[:, 1] + boxes_cxcywh[:, 3] / 2
        xyxy_model = np.stack([x1, y1, x2, y2], axis=1)
        xyxy = xyxy_model.copy()

        pad_w, pad_h = pad
        xyxy[:, [0, 2]] -= pad_w
        xyxy[:, [1, 3]] -= pad_h
        xyxy /= scale

        h, w = original_shape[:2]
        xyxy[:, [0, 2]] = np.clip(xyxy[:, [0, 2]], 0, w)
        xyxy[:, [1, 3]] = np.clip(xyxy[:, [1, 3]], 0, h)

        # NMS expects [x, y, w, h]
        nms_boxes = np.stack(
            [xyxy[:, 0], xyxy[:, 1], xyxy[:, 2] - xyxy[:, 0], xyxy[:, 3] - xyxy[:, 1]],
            axis=1,
        ).tolist()
        keep_idx = cv2.dnn.NMSBoxes(
            nms_boxes, max_scores.tolist(), self.conf_threshold, self.iou_threshold
        )

        detections = []
        if len(keep_idx) == 0:
            return detections
        for i in np.array(keep_idx).flatten():
            x1_, y1_, x2_, y2_ = xyxy[i]
            detection = {
                "class_id": int(class_ids[i]),
                "class_name": self._class_name(class_ids[i]),
                "score": float(max_scores[i]),
                "bbox": {
                    "center_x": float((x1_ + x2_) / 2),
                    "center_y": float((y1_ + y2_) / 2),
                    "size_x": float(x2_ - x1_),
                    "size_y": float(y2_ - y1_),
                },
            }
            if include_masks and protos is not None:
                detection["_mask"] = self._segmentation_mask(
                    mask_coeffs[i], protos, xyxy_model[i], xyxy[i], scale, pad, original_shape
                )
            detections.append(detection)
        return detections

    def _attach_depth(self, image_msg, detections, frame_shape):
        image_stamp_sec = self._stamp_to_sec(image_msg.header.stamp)
        depth = self.latest_depth
        depth_stamp_sec = self.latest_depth_stamp_sec
        if depth is None or depth_stamp_sec is None:
            for detection in detections:
                detection["depth_valid"] = False
                detection["depth_reason"] = "no_depth_frame"
            return

        depth_age_sec = abs(image_stamp_sec - depth_stamp_sec)
        if self.depth_max_age_sec > 0 and depth_age_sec > self.depth_max_age_sec:
            for detection in detections:
                detection["depth_valid"] = False
                detection["depth_reason"] = "stale_depth_frame"
                detection["depth_age_sec"] = float(depth_age_sec)
            return

        frame_h, frame_w = frame_shape[:2]
        depth_h, depth_w = depth.shape[:2]
        for detection in detections:
            distance_m, sample_count = self._median_depth_for_bbox(
                depth, detection["bbox"], frame_w, frame_h, depth_w, depth_h
            )
            detection["depth_valid"] = distance_m is not None
            detection["depth_sample_count"] = int(sample_count)
            detection["depth_age_sec"] = float(depth_age_sec)
            detection["depth_frame_id"] = self.latest_depth_frame_id
            if distance_m is None:
                detection["depth_reason"] = "not_enough_valid_depth"
            else:
                detection["depth_m"] = float(distance_m)

    def _median_depth_for_bbox(self, depth, bbox, frame_w, frame_h, depth_w, depth_h):
        cx = bbox["center_x"] * depth_w / frame_w
        cy = bbox["center_y"] * depth_h / frame_h
        size_x = max(1.0, bbox["size_x"] * depth_w / frame_w * self.depth_roi_scale)
        size_y = max(1.0, bbox["size_y"] * depth_h / frame_h * self.depth_roi_scale)

        x1 = int(max(0, round(cx - size_x / 2)))
        x2 = int(min(depth_w, round(cx + size_x / 2)))
        y1 = int(max(0, round(cy - size_y / 2)))
        y2 = int(min(depth_h, round(cy + size_y / 2)))
        if x2 <= x1 or y2 <= y1:
            return None, 0

        roi = depth[y1:y2, x1:x2].astype(np.float32)
        if roi.ndim == 3:
            roi = roi[:, :, 0]
        valid = np.isfinite(roi) & (roi > 0)
        values = roi[valid]
        if values.size < self.depth_min_valid_pixels:
            return None, values.size
        return float(np.median(values) * self.depth_unit_scale), values.size

    def _stamp_to_sec(self, stamp):
        return float(stamp.sec) + float(stamp.nanosec) * 1e-9

    def _prediction_matrix(self, output):
        pred = np.squeeze(output, axis=0) if output.ndim == 3 and output.shape[0] == 1 else output
        if pred.ndim != 2:
            raise RuntimeError(f"Unexpected YOLO prediction output shape: {output.shape}")
        if pred.shape[0] < pred.shape[1]:
            pred = pred.T
        return pred

    def _segmentation_protos(self, outputs):
        if len(outputs) < 2:
            return None
        protos = outputs[1]
        if protos.ndim == 4 and protos.shape[0] == 1:
            return protos[0]
        if protos.ndim == 3:
            return protos
        return None

    def _segmentation_mask(self, coeffs, protos, box_model, box_original, scale, pad, original_shape):
        mask = coeffs @ protos.reshape(protos.shape[0], -1)
        mask = 1.0 / (1.0 + np.exp(-mask))
        mask = mask.reshape(protos.shape[1], protos.shape[2])
        mask = cv2.resize(mask, (self.input_size, self.input_size), interpolation=cv2.INTER_LINEAR)

        h, w = original_shape[:2]
        pad_w, pad_h = pad
        scaled_w = int(round(w * scale))
        scaled_h = int(round(h * scale))
        mask = mask[pad_h : pad_h + scaled_h, pad_w : pad_w + scaled_w]
        if mask.size == 0:
            return None
        mask = cv2.resize(mask, (w, h), interpolation=cv2.INTER_LINEAR) > 0.5

        x1, y1, x2, y2 = np.round(box_original).astype(int)
        clipped = np.zeros((h, w), dtype=bool)
        x1, x2 = max(0, x1), min(w, x2)
        y1, y2 = max(0, y1), min(h, y2)
        if x2 > x1 and y2 > y1:
            clipped[y1:y2, x1:x2] = mask[y1:y2, x1:x2]
        return clipped

    def _to_vision_msg(self, image_msg, detections):
        from vision_msgs.msg import (
            Detection2D,
            Detection2DArray,
            ObjectHypothesisWithPose,
        )

        out = Detection2DArray()
        out.header = image_msg.header
        for detection in detections:
            d = Detection2D()
            bbox = detection["bbox"]
            d.bbox.center.position.x = bbox["center_x"]
            d.bbox.center.position.y = bbox["center_y"]
            d.bbox.size_x = bbox["size_x"]
            d.bbox.size_y = bbox["size_y"]

            hyp = ObjectHypothesisWithPose()
            hyp.hypothesis.class_id = detection.get("class_name", str(detection["class_id"]))
            hyp.hypothesis.score = detection["score"]
            d.results.append(hyp)
            out.detections.append(d)
        return out

    def _to_json_msg(self, image_msg, detections):
        out = String()
        out.data = json.dumps(
            {
                "stamp": {
                    "sec": image_msg.header.stamp.sec,
                    "nanosec": image_msg.header.stamp.nanosec,
                },
                "frame_id": image_msg.header.frame_id,
                "detections": [self._json_detection(d) for d in detections],
            }
        )
        return out

    def _json_detection(self, detection):
        return {
            key: value
            for key, value in detection.items()
            if not key.startswith("_")
        }

    def _publish_annotated(self, image_msg, frame, detections):
        now = time.monotonic()
        if self.annotated_max_fps > 0:
            min_period = 1.0 / self.annotated_max_fps
            if now - self._last_annotated_publish < min_period:
                return
        self._last_annotated_publish = now

        annotated = frame.copy()
        for detection in detections:
            bbox = detection["bbox"]
            x1 = int(round(bbox["center_x"] - bbox["size_x"] / 2))
            y1 = int(round(bbox["center_y"] - bbox["size_y"] / 2))
            x2 = int(round(bbox["center_x"] + bbox["size_x"] / 2))
            y2 = int(round(bbox["center_y"] + bbox["size_y"] / 2))
            h, w = annotated.shape[:2]
            x1, x2 = max(0, x1), min(w - 1, x2)
            y1, y2 = max(0, y1), min(h - 1, y2)

            color = self._class_color(detection["class_id"])
            mask = detection.get("_mask")
            if mask is not None:
                mask = mask.astype(bool)
                annotated[mask] = (
                    0.6 * annotated[mask] + 0.4 * np.array(color, dtype=np.uint8)
                ).astype(np.uint8)
                contours, _ = cv2.findContours(
                    mask.astype(np.uint8), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
                )
                cv2.drawContours(annotated, contours, -1, color, 1)

            cv2.rectangle(annotated, (x1, y1), (x2, y2), color, 2)
            label = f"{detection.get('class_name', detection['class_id'])} {detection['score']:.2f}"
            if detection.get("depth_valid"):
                label += f" {detection['depth_m']:.2f}m"
            cv2.putText(
                annotated,
                label,
                (x1, max(20, y1 - 8)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                color,
                2,
                cv2.LINE_AA,
            )

        if self.annotated_scale != 1.0:
            annotated = cv2.resize(
                annotated,
                None,
                fx=self.annotated_scale,
                fy=self.annotated_scale,
                interpolation=cv2.INTER_AREA,
            )

        ok, encoded = cv2.imencode(
            ".jpg",
            annotated,
            [int(cv2.IMWRITE_JPEG_QUALITY), self.annotated_jpeg_quality],
        )
        if not ok:
            self.get_logger().warning("Failed to encode annotated preview")
            return

        out = CompressedImage()
        out.header = image_msg.header
        out.format = "jpeg"
        out.data = encoded.tobytes()
        self.annotated_publisher.publish(out)

    def _class_color(self, class_id):
        colors = [
            (0, 255, 0),
            (0, 165, 255),
            (255, 0, 0),
            (255, 0, 255),
            (0, 255, 255),
            (255, 255, 0),
        ]
        return colors[int(class_id) % len(colors)]

    def _log_detection_summary(self, detections):
        self._log_frames += 1
        if detections:
            self._log_frames_with_detections += 1
            self._log_total_detections += len(detections)
            for detection in detections:
                class_id = detection["class_id"]
                score = detection["score"]
                self._log_class_counts[class_id] += 1
                self._log_best_by_class[class_id] = max(
                    self._log_best_by_class.get(class_id, 0.0), score
                )

        now = time.monotonic()
        if now - self._last_detection_log < self.detection_log_interval:
            return

        class_counts = {
            self._class_name(class_id): count
            for class_id, count in sorted(self._log_class_counts.items())
        }
        best_scores = {
            self._class_name(class_id): round(score, 3)
            for class_id, score in sorted(self._log_best_by_class.items())
        }
        self.get_logger().info(
            "Detection summary "
            f"frames={self._log_frames} "
            f"frames_with_detections={self._log_frames_with_detections} "
            f"total_detections={self._log_total_detections} "
            f"class_counts={class_counts} "
            f"best_scores={best_scores}"
        )

        self._last_detection_log = now
        self._log_frames = 0
        self._log_frames_with_detections = 0
        self._log_total_detections = 0
        self._log_class_counts.clear()
        self._log_best_by_class.clear()


def main():
    rclpy.init()
    node = DetectorNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
