#!/usr/bin/env python3
import sys

import rospy
import tf2_ros
from geometry_msgs.msg import PoseStamped, TwistStamped

from gen3_lite_hand_follow.filters import LowPassFilter, SlewRateLimiter
from gen3_lite_hand_follow.geometry import clamp_norm, deadband, rotate_vector, transform_to_xyz_quat

try:
    from kortex_driver.msg import CartesianReferenceFrame, TwistCommand
except ImportError:
    CartesianReferenceFrame = None
    TwistCommand = None


class HandFollowControllerNode:
    def __init__(self):
        self.base_frame = rospy.get_param("~base_frame", "base_link")
        self.tool_frame = rospy.get_param("~tool_frame", "tool_frame")
        self.camera_frame = rospy.get_param("~camera_frame", "camera_color_optical_frame")
        self.robot_name = rospy.get_param("~robot_name", "my_gen3_lite")
        self.dry_run = bool(rospy.get_param("~dry_run", True))
        self.kortex_reference_frame = int(rospy.get_param("~kortex_reference_frame", -1))

        self.hand_camera_topic = rospy.get_param("~hand_camera_topic", "/hand_detector/hand_pose_camera")
        self.hand_base_topic = rospy.get_param("~hand_base_topic", "/hand_detector/hand_pose_base")
        self.cartesian_velocity_topic = rospy.get_param(
            "~cartesian_velocity_topic",
            "/%s/in/cartesian_velocity" % self.robot_name,
        )

        self.control_rate_hz = float(rospy.get_param("~control_rate_hz", 30.0))
        self.target_timeout_s = float(rospy.get_param("~target_timeout_s", 0.25))
        self.startup_hold_s = float(rospy.get_param("~startup_hold_s", 1.0))

        self.desired_hand_camera = [
            float(rospy.get_param("~desired_hand_camera_x", 0.0)),
            float(rospy.get_param("~desired_hand_camera_y", 0.0)),
            float(rospy.get_param("~desired_hand_distance_m", 0.45)),
        ]
        self.kp = float(rospy.get_param("~kp", 0.75))
        self.kp_xyz = list(rospy.get_param("~kp_xyz", [self.kp, self.kp, self.kp]))
        self.deadband_xyz = list(rospy.get_param("~deadband_xyz_m", [0.03, 0.03, 0.04]))
        self.invert_command_sign = bool(rospy.get_param("~invert_command_sign", False))
        self.feedforward_gain = float(rospy.get_param("~feedforward_gain", 0.25))
        self.latency_comp_s = float(rospy.get_param("~latency_comp_s", 0.05))
        self.deadband_m = float(rospy.get_param("~deadband_m", 0.018))
        self.max_linear_speed = float(rospy.get_param("~max_linear_speed_mps", 0.18))
        self.max_linear_accel = float(rospy.get_param("~max_linear_accel_mps2", 0.45))
        self.command_lpf_alpha = float(rospy.get_param("~command_lpf_alpha", 0.35))

        self.workspace_min = list(rospy.get_param("~workspace_min_xyz", [-0.45, -0.55, 0.05]))
        self.workspace_max = list(rospy.get_param("~workspace_max_xyz", [0.65, 0.55, 0.75]))
        self.stop_near_workspace_margin = float(rospy.get_param("~stop_near_workspace_margin_m", 0.02))

        if not self.dry_run and TwistCommand is None:
            rospy.logfatal(
                "kortex_driver is not importable, but dry_run is false. "
                "Source the ros_kortex workspace or launch with dry_run:=true."
            )
            sys.exit(1)

        if self.kortex_reference_frame < 0:
            if CartesianReferenceFrame is not None and hasattr(CartesianReferenceFrame, "CARTESIAN_REFERENCE_FRAME_BASE"):
                self.kortex_reference_frame = CartesianReferenceFrame.CARTESIAN_REFERENCE_FRAME_BASE
            else:
                # Kortex CartesianReferenceFrame enum: BASE is normally 3.
                self.kortex_reference_frame = 3

        self.tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(5.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.last_hand_camera = None
        self.last_hand_base = None
        self.last_hand_camera_receive_time = None
        self.last_hand_base_receive_time = None
        self.prev_hand_base = None
        self.prev_hand_time = None
        self.hand_velocity_filter = [LowPassFilter(0.25, 0.0), LowPassFilter(0.25, 0.0), LowPassFilter(0.25, 0.0)]
        self.command_filter = [LowPassFilter(self.command_lpf_alpha, 0.0) for _ in range(3)]
        self.slew = SlewRateLimiter(self.max_linear_accel, size=3)
        self.start_time = rospy.Time.now()

        self.debug_pub = rospy.Publisher("~debug_cmd_vel", TwistStamped, queue_size=1)
        if self.dry_run:
            self.cmd_pub = None
            rospy.logwarn("hand_follow_controller running in dry_run mode; Kinova velocity commands are not sent")
        else:
            self.cmd_pub = rospy.Publisher(self.cartesian_velocity_topic, TwistCommand, queue_size=1)

        rospy.Subscriber(self.hand_camera_topic, PoseStamped, self._hand_camera_cb, queue_size=1)
        rospy.Subscriber(self.hand_base_topic, PoseStamped, self._hand_base_cb, queue_size=1)

        self.timer = rospy.Timer(rospy.Duration(1.0 / self.control_rate_hz), self._control_cb)
        rospy.on_shutdown(self._on_shutdown)
        rospy.loginfo(
            "hand_follow_controller started, target topic: %s, reference_frame: %s",
            self.cartesian_velocity_topic,
            self.kortex_reference_frame,
        )

    def _hand_camera_cb(self, msg):
        self.last_hand_camera = msg
        self.last_hand_camera_receive_time = rospy.Time.now()

    def _hand_base_cb(self, msg):
        self.last_hand_base = msg
        self.last_hand_base_receive_time = rospy.Time.now()
        p = [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]
        t = msg.header.stamp.to_sec()
        if t <= 0.0:
            t = self.last_hand_base_receive_time.to_sec()
        if self.prev_hand_base is not None and self.prev_hand_time is not None:
            dt = max(1.0e-3, t - self.prev_hand_time)
            raw_v = [(p[i] - self.prev_hand_base[i]) / dt for i in range(3)]
            for i in range(3):
                self.hand_velocity_filter[i].filter(raw_v[i])
        self.prev_hand_base = p
        self.prev_hand_time = t

    def _control_cb(self, _event):
        now = rospy.Time.now()
        if (now - self.start_time).to_sec() < self.startup_hold_s:
            self._publish_velocity([0.0, 0.0, 0.0])
            return

        if self.last_hand_camera is None or self.last_hand_base is None:
            self._publish_velocity([0.0, 0.0, 0.0])
            return

        target_time = self.last_hand_camera_receive_time
        if target_time is None:
            target_time = self.last_hand_camera.header.stamp
        age = (now - target_time).to_sec()
        if age > self.target_timeout_s:
            rospy.logwarn_throttle(1.0, "Hand target timeout %.3fs; stopping", age)
            self._publish_velocity([0.0, 0.0, 0.0])
            return

        try:
            cam_to_base = self.tf_buffer.lookup_transform(
                self.base_frame,
                self.last_hand_camera.header.frame_id,
                rospy.Time(0),
                timeout=rospy.Duration(0.02),
            )
        except Exception as exc:
            rospy.logwarn_throttle(1.0, "Missing TF %s <- %s: %s", self.base_frame, self.camera_frame, exc)
            self._publish_velocity([0.0, 0.0, 0.0])
            return

        hand_cam = [
            self.last_hand_camera.pose.position.x,
            self.last_hand_camera.pose.position.y,
            self.last_hand_camera.pose.position.z,
        ]
        error_cam = [hand_cam[i] - self.desired_hand_camera[i] for i in range(3)]
        error_cam = [
            0.0 if abs(error_cam[i]) < self.deadband_xyz[i] else error_cam[i]
            for i in range(3)
        ]

        _, q_base_cam = transform_to_xyz_quat(cam_to_base)
        cmd_cam = [self.kp_xyz[i] * error_cam[i] for i in range(3)]
        if self.invert_command_sign:
            cmd_cam = [-v for v in cmd_cam]
        cmd_base = rotate_vector(q_base_cam, cmd_cam)
        hand_v = [f.value if f.value is not None else 0.0 for f in self.hand_velocity_filter]
        proportional = [
            cmd_base[i] + self.latency_comp_s * self.feedforward_gain * hand_v[i]
            for i in range(3)
        ]
        predicted_ff = [self.feedforward_gain * hand_v[i] for i in range(3)]
        cmd = [proportional[i] + predicted_ff[i] for i in range(3)]
        cmd = clamp_norm(cmd, self.max_linear_speed)
        cmd = self._apply_workspace_guard(cmd)

        filtered = [self.command_filter[i].filter(cmd[i]) for i in range(3)]
        limited = self.slew.limit(filtered, stamp=now.to_sec())
        self._publish_velocity(limited)

    def _apply_workspace_guard(self, velocity):
        try:
            tool_tf = self.tf_buffer.lookup_transform(
                self.base_frame,
                self.tool_frame,
                rospy.Time(0),
                timeout=rospy.Duration(0.01),
            )
        except Exception as exc:
            rospy.logwarn_throttle(2.0, "Workspace guard cannot see tool TF: %s", exc)
            return velocity

        tool_xyz, _ = transform_to_xyz_quat(tool_tf)
        guarded = list(velocity)
        for i in range(3):
            low = self.workspace_min[i] + self.stop_near_workspace_margin
            high = self.workspace_max[i] - self.stop_near_workspace_margin
            if tool_xyz[i] <= low and guarded[i] < 0.0:
                guarded[i] = 0.0
            if tool_xyz[i] >= high and guarded[i] > 0.0:
                guarded[i] = 0.0
        return guarded

    def _publish_velocity(self, linear_xyz):
        self._publish_debug(linear_xyz)
        if self.dry_run:
            return

        cmd = TwistCommand()
        cmd.reference_frame = self.kortex_reference_frame
        cmd.duration = 0
        cmd.twist.linear_x = linear_xyz[0]
        cmd.twist.linear_y = linear_xyz[1]
        cmd.twist.linear_z = linear_xyz[2]
        cmd.twist.angular_x = 0.0
        cmd.twist.angular_y = 0.0
        cmd.twist.angular_z = 0.0
        self.cmd_pub.publish(cmd)

    def _publish_debug(self, linear_xyz):
        msg = TwistStamped()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = self.base_frame
        msg.twist.linear.x = linear_xyz[0]
        msg.twist.linear.y = linear_xyz[1]
        msg.twist.linear.z = linear_xyz[2]
        self.debug_pub.publish(msg)

    def _on_shutdown(self):
        for _ in range(8):
            self._publish_velocity([0.0, 0.0, 0.0])
            rospy.sleep(0.02)


def main():
    rospy.init_node("hand_follow_controller")
    HandFollowControllerNode()
    rospy.spin()


if __name__ == "__main__":
    main()
