#!/usr/bin/env python3
import math
import sys

import cv2
import message_filters
import numpy as np
import rospy
import tf2_geometry_msgs
import tf2_ros
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Bool, String
from visualization_msgs.msg import Marker

from gen3_lite_hand_follow.filters import VectorOneEuroFilter

try:
    import mediapipe as mp
except ImportError:
    mp = None


class HandDetectorNode:
    def __init__(self):
        if mp is None:
            rospy.logfatal(
                "Python package 'mediapipe' is not installed. Install it in the ROS "
                "Python environment, for example: pip3 install mediapipe"
            )
            sys.exit(1)

        self.base_frame = rospy.get_param("~base_frame", "base_link")
        self.camera_frame = rospy.get_param("~camera_frame", "")
        self.color_topic = rospy.get_param("~color_topic", "/camera/color/image_raw")
        self.depth_topic = rospy.get_param("~depth_topic", "/camera/aligned_depth_to_color/image_raw")
        self.camera_info_topic = rospy.get_param("~camera_info_topic", "/camera/color/camera_info")
        self.depth_scale = float(rospy.get_param("~depth_scale", 0.001))
        self.min_depth_m = float(rospy.get_param("~min_depth_m", 0.20))
        self.max_depth_m = float(rospy.get_param("~max_depth_m", 1.20))
        self.depth_window_px = int(rospy.get_param("~depth_window_px", 9))
        self.depth_fallback_window_px = int(rospy.get_param("~depth_fallback_window_px", 41))
        self.min_valid_depth_samples = int(rospy.get_param("~min_valid_depth_samples", 3))
        self.max_sync_slop_s = float(rospy.get_param("~max_sync_slop_s", 0.06))
        self.publish_debug_image = bool(rospy.get_param("~publish_debug_image", True))
        self.flip_rgb = bool(rospy.get_param("~flip_rgb", False))
        self.use_latest_tf_for_base = bool(rospy.get_param("~use_latest_tf_for_base", True))
        self.enable_gesture = bool(rospy.get_param("~enable_gesture", True))
        self.open_finger_min_count = int(rospy.get_param("~open_finger_min_count", 3))
        self.fist_finger_max_count = int(rospy.get_param("~fist_finger_max_count", 1))

        min_cutoff = float(rospy.get_param("~one_euro_min_cutoff", 1.2))
        beta = float(rospy.get_param("~one_euro_beta", 0.04))
        d_cutoff = float(rospy.get_param("~one_euro_d_cutoff", 1.0))
        self.filter_enabled = bool(rospy.get_param("~enable_filter", True))
        self.position_filter = VectorOneEuroFilter(
            min_cutoff=min_cutoff,
            beta=beta,
            d_cutoff=d_cutoff,
            size=3,
        )

        self.bridge = CvBridge()
        self.camera_info = None
        self.last_valid_stamp = None

        self.tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(5.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.pub_camera = rospy.Publisher("~hand_pose_camera", PoseStamped, queue_size=1)
        self.pub_base = rospy.Publisher("~hand_pose_base", PoseStamped, queue_size=1)
        self.pub_valid = rospy.Publisher("~hand_valid", Bool, queue_size=1)
        self.pub_gesture = rospy.Publisher("~hand_gesture", String, queue_size=1)
        self.pub_marker = rospy.Publisher("~hand_marker", Marker, queue_size=1)
        self.pub_debug = rospy.Publisher("~hand_debug_image", Image, queue_size=1) if self.publish_debug_image else None

        self.info_sub = rospy.Subscriber(self.camera_info_topic, CameraInfo, self._camera_info_cb, queue_size=1)
        color_sub = message_filters.Subscriber(self.color_topic, Image)
        depth_sub = message_filters.Subscriber(self.depth_topic, Image)
        self.sync = message_filters.ApproximateTimeSynchronizer(
            [color_sub, depth_sub],
            queue_size=5,
            slop=self.max_sync_slop_s,
        )
        self.sync.registerCallback(self._image_cb)

        self.mp_hands = mp.solutions.hands
        self.mp_draw = mp.solutions.drawing_utils
        self.hands = self.mp_hands.Hands(
            static_image_mode=False,
            max_num_hands=1,
            model_complexity=int(rospy.get_param("~mediapipe_model_complexity", 0)),
            min_detection_confidence=float(rospy.get_param("~min_detection_confidence", 0.65)),
            min_tracking_confidence=float(rospy.get_param("~min_tracking_confidence", 0.60)),
        )
        self.landmark_indices = list(rospy.get_param("~landmark_indices", [0, 5, 9, 13, 17]))
        rospy.loginfo("hand_detector_node started: %s + %s", self.color_topic, self.depth_topic)

    def _camera_info_cb(self, msg):
        self.camera_info = msg
        if not self.camera_frame:
            self.camera_frame = msg.header.frame_id

    def _image_cb(self, color_msg, depth_msg):
        if self.camera_info is None:
            rospy.logwarn_throttle(2.0, "Waiting for CameraInfo on %s", self.camera_info_topic)
            return

        try:
            color_bgr = self.bridge.imgmsg_to_cv2(color_msg, desired_encoding="bgr8")
            depth = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding="passthrough")
        except Exception as exc:
            rospy.logwarn_throttle(2.0, "cv_bridge conversion failed: %s", exc)
            return

        if self.flip_rgb:
            color_bgr = cv2.flip(color_bgr, 1)
            depth = cv2.flip(depth, 1)

        color_rgb = cv2.cvtColor(color_bgr, cv2.COLOR_BGR2RGB)
        color_rgb.flags.writeable = False
        results = self.hands.process(color_rgb)
        color_rgb.flags.writeable = True

        if not results.multi_hand_landmarks:
            self.pub_valid.publish(Bool(data=False))
            if self.publish_debug_image:
                self._publish_debug(color_bgr, color_msg.header)
            return

        hand_landmarks = results.multi_hand_landmarks[0]
        gesture = self._classify_gesture(hand_landmarks) if self.enable_gesture else "unknown"
        self.pub_gesture.publish(String(data=gesture))
        h, w = color_bgr.shape[:2]
        points_camera = []
        debug_points = []

        for idx in self.landmark_indices:
            lm = hand_landmarks.landmark[idx]
            u = int(round(lm.x * (w - 1)))
            v = int(round(lm.y * (h - 1)))
            z = self._sample_depth(depth, u, v)
            if z is None:
                continue
            xyz = self._deproject(u, v, z)
            points_camera.append(xyz)
            debug_points.append((u, v))

        if not points_camera:
            self.pub_valid.publish(Bool(data=False))
            rospy.logwarn_throttle(
                1.0,
                "Hand detected in RGB, but no valid depth samples in ROI. "
                "Check aligned depth topic, hand distance, and min/max depth filters."
            )
            if self.publish_debug_image:
                self.mp_draw.draw_landmarks(color_bgr, hand_landmarks, self.mp_hands.HAND_CONNECTIONS)
                self._publish_debug(color_bgr, color_msg.header)
            return

        xyz = np.median(np.asarray(points_camera, dtype=np.float64), axis=0).tolist()
        stamp_s = color_msg.header.stamp.to_sec()
        if self.filter_enabled:
            xyz = self.position_filter.filter(xyz, stamp=stamp_s)

        camera_pose = PoseStamped()
        camera_pose.header.stamp = color_msg.header.stamp
        camera_pose.header.frame_id = self.camera_frame or color_msg.header.frame_id
        camera_pose.pose.position.x = xyz[0]
        camera_pose.pose.position.y = xyz[1]
        camera_pose.pose.position.z = xyz[2]
        camera_pose.pose.orientation.w = 1.0

        self.pub_camera.publish(camera_pose)
        self.pub_valid.publish(Bool(data=True))

        try:
            transform_pose = camera_pose
            if self.use_latest_tf_for_base:
                transform_pose = PoseStamped()
                transform_pose.header = camera_pose.header
                transform_pose.header.stamp = rospy.Time(0)
                transform_pose.pose = camera_pose.pose

            base_pose = self.tf_buffer.transform(
                transform_pose,
                self.base_frame,
                timeout=rospy.Duration(0.03),
            )
            base_pose.header.stamp = camera_pose.header.stamp
            self.pub_base.publish(base_pose)
            self._publish_marker(base_pose)
        except Exception as exc:
            rospy.logwarn_throttle(
                1.0,
                "Could not transform hand pose from %s to %s: %s",
                camera_pose.header.frame_id,
                self.base_frame,
                exc,
            )

        if self.publish_debug_image:
            self.mp_draw.draw_landmarks(color_bgr, hand_landmarks, self.mp_hands.HAND_CONNECTIONS)
            for u, v in debug_points:
                cv2.circle(color_bgr, (u, v), 5, (0, 255, 255), -1)
            cv2.putText(
                color_bgr,
                "hand xyz camera: %.3f %.3f %.3f m" % (xyz[0], xyz[1], xyz[2]),
                (20, 32),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (0, 255, 0),
                2,
                cv2.LINE_AA,
            )
            cv2.putText(
                color_bgr,
                "gesture: %s" % gesture,
                (20, 64),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (0, 255, 0),
                2,
                cv2.LINE_AA,
            )
            self._publish_debug(color_bgr, color_msg.header)

    def _classify_gesture(self, hand_landmarks):
        lm = hand_landmarks.landmark
        wrist = lm[0]

        def dist(a, b):
            dx = lm[a].x - lm[b].x
            dy = lm[a].y - lm[b].y
            dz = lm[a].z - lm[b].z
            return math.sqrt(dx * dx + dy * dy + dz * dz)

        palm_scale = max(1.0e-6, dist(0, 9))
        fingers = [
            (8, 6, 5),    # index tip, pip, mcp
            (12, 10, 9),  # middle
            (16, 14, 13), # ring
            (20, 18, 17), # pinky
        ]
        extended = 0
        for tip, pip, mcp in fingers:
            tip_d = dist(0, tip) / palm_scale
            pip_d = dist(0, pip) / palm_scale
            mcp_d = dist(0, mcp) / palm_scale
            if tip_d > pip_d * 1.12 and tip_d > mcp_d * 1.28:
                extended += 1

        # Thumb is less reliable across left/right hands, but it helps open-palm detection.
        thumb_open = dist(4, 17) > dist(3, 17) * 1.10
        if thumb_open:
            extended += 1

        if extended >= self.open_finger_min_count:
            return "open"
        if extended <= self.fist_finger_max_count:
            return "fist"
        return "unknown"

    def _sample_depth(self, depth, u, v):
        if u < 0 or v < 0 or u >= depth.shape[1] or v >= depth.shape[0]:
            return None

        z = self._sample_depth_window(depth, u, v, self.depth_window_px)
        if z is not None:
            return z
        if self.depth_fallback_window_px > self.depth_window_px:
            return self._sample_depth_window(depth, u, v, self.depth_fallback_window_px)
        return None

    def _sample_depth_window(self, depth, u, v, window_px):
        half = max(1, self.depth_window_px // 2)
        half = max(1, int(window_px) // 2)
        u0, u1 = max(0, u - half), min(depth.shape[1], u + half + 1)
        v0, v1 = max(0, v - half), min(depth.shape[0], v + half + 1)
        patch = depth[v0:v1, u0:u1].astype(np.float32)

        if depth.dtype == np.uint16:
            patch *= self.depth_scale

        valid = patch[np.isfinite(patch)]
        valid = valid[(valid >= self.min_depth_m) & (valid <= self.max_depth_m)]
        if valid.size < self.min_valid_depth_samples:
            return None
        return float(np.median(valid))

    def _deproject(self, u, v, z):
        k = self.camera_info.K
        fx, fy = k[0], k[4]
        cx, cy = k[2], k[5]
        if abs(fx) < 1.0e-6 or abs(fy) < 1.0e-6:
            raise RuntimeError("Invalid CameraInfo intrinsics")
        x = (float(u) - cx) * z / fx
        y = (float(v) - cy) * z / fy
        return [x, y, z]

    def _publish_debug(self, image_bgr, header):
        if self.pub_debug is None:
            return
        msg = self.bridge.cv2_to_imgmsg(image_bgr, encoding="bgr8")
        msg.header = header
        self.pub_debug.publish(msg)

    def _publish_marker(self, pose):
        marker = Marker()
        marker.header = pose.header
        marker.ns = "hand_follow"
        marker.id = 1
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose = pose.pose
        marker.scale.x = 0.06
        marker.scale.y = 0.06
        marker.scale.z = 0.06
        marker.color.r = 0.1
        marker.color.g = 0.9
        marker.color.b = 0.2
        marker.color.a = 0.85
        marker.lifetime = rospy.Duration(0.2)
        self.pub_marker.publish(marker)


def main():
    rospy.init_node("hand_detector")
    HandDetectorNode()
    rospy.spin()


if __name__ == "__main__":
    main()
