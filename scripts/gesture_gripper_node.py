#!/usr/bin/env python3
import sys

import rospy
from std_msgs.msg import Float32, String

try:
    from kortex_driver.msg import Finger, GripperMode
    from kortex_driver.srv import SendGripperCommand, SendGripperCommandRequest
except ImportError:
    Finger = None
    GripperMode = None
    SendGripperCommand = None
    SendGripperCommandRequest = None


class GestureGripperNode:
    def __init__(self):
        self.robot_name = rospy.get_param("~robot_name", "my_gen3")
        self.gesture_topic = rospy.get_param("~gesture_topic", "/hand_detector/hand_gesture")
        self.command_mode = rospy.get_param("~command_mode", "kortex_service")
        self.service_name = rospy.get_param(
            "~service_name",
            "/%s/base/send_gripper_command" % self.robot_name,
        )
        self.position_topic = rospy.get_param("~position_topic", "/%s/gripper_position" % self.robot_name)
        self.open_value = float(rospy.get_param("~open_value", 0.0))
        self.close_value = float(rospy.get_param("~close_value", 0.75))
        self.finger_identifier = int(rospy.get_param("~finger_identifier", 0))
        self.debounce_count = int(rospy.get_param("~debounce_count", 5))
        self.min_command_interval_s = float(rospy.get_param("~min_command_interval_s", 1.0))
        self.dry_run = bool(rospy.get_param("~dry_run", False))

        self.last_raw = None
        self.raw_count = 0
        self.stable_gesture = "unknown"
        self.last_commanded_gesture = None
        self.last_command_time = rospy.Time(0)
        self.service = None
        self.position_pub = None

        if self.command_mode == "kortex_service":
            if SendGripperCommand is None:
                rospy.logfatal("kortex_driver SendGripperCommand is not importable")
                sys.exit(1)
            if not self.dry_run:
                rospy.loginfo("Waiting for gripper service: %s", self.service_name)
                rospy.wait_for_service(self.service_name, timeout=10.0)
                self.service = rospy.ServiceProxy(self.service_name, SendGripperCommand)
        elif self.command_mode == "topic_float":
            self.position_pub = rospy.Publisher(self.position_topic, Float32, queue_size=1)
        else:
            rospy.logfatal("Unknown gripper command_mode: %s", self.command_mode)
            sys.exit(1)

        self.gesture_pub = rospy.Publisher("~stable_gesture", String, queue_size=1)
        rospy.Subscriber(self.gesture_topic, String, self._gesture_cb, queue_size=5)
        rospy.loginfo(
            "gesture_gripper_node started: gesture_topic=%s mode=%s dry_run=%s",
            self.gesture_topic,
            self.command_mode,
            self.dry_run,
        )

    def _gesture_cb(self, msg):
        gesture = msg.data.strip().lower()
        if gesture not in ("open", "fist"):
            return

        if gesture == self.last_raw:
            self.raw_count += 1
        else:
            self.last_raw = gesture
            self.raw_count = 1

        if self.raw_count < self.debounce_count:
            return

        if gesture == self.stable_gesture:
            return

        self.stable_gesture = gesture
        self.gesture_pub.publish(String(data=gesture))
        self._handle_stable_gesture(gesture)

    def _handle_stable_gesture(self, gesture):
        now = rospy.Time.now()
        if gesture == self.last_commanded_gesture:
            return
        if (now - self.last_command_time).to_sec() < self.min_command_interval_s:
            rospy.loginfo_throttle(1.0, "Ignoring gripper gesture during command cooldown")
            return

        value = self.close_value if gesture == "fist" else self.open_value
        action = "close" if gesture == "fist" else "open"
        rospy.loginfo("Gesture %s -> gripper %s value %.3f", gesture, action, value)

        if self.dry_run:
            self.last_commanded_gesture = gesture
            self.last_command_time = now
            return

        try:
            self._send_gripper_position(value)
            self.last_commanded_gesture = gesture
            self.last_command_time = now
        except Exception as exc:
            rospy.logerr("Failed to send gripper command: %s", exc)

    def _send_gripper_position(self, value):
        if self.command_mode == "topic_float":
            self.position_pub.publish(Float32(data=value))
            return

        req = SendGripperCommandRequest()
        finger = Finger()
        finger.finger_identifier = self.finger_identifier
        finger.value = value

        req.input.mode = getattr(GripperMode, "GRIPPER_POSITION", 3)
        req.input.gripper.finger.append(finger)
        self.service(req)


def main():
    rospy.init_node("gesture_gripper")
    GestureGripperNode()
    rospy.spin()


if __name__ == "__main__":
    main()
