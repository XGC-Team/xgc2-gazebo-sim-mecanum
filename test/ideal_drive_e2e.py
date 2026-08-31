#!/usr/bin/env python3
import math
import threading
import time
import unittest

import rospy
import rostest
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import PoseStamped, Twist, TwistStamped
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32
from tf.transformations import euler_from_quaternion


class IdealDriveContractTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rospy.init_node("ideal_drive_contract_test", anonymous=True)
        cls.lock = threading.Lock()
        cls.poses = []
        cls.twists = []
        cls.imus = []
        cls.voltages = []
        cls.model_states = None
        cls.pose_sub = rospy.Subscriber(
            "/ugv1/simulation/ground_truth/pose", PoseStamped, cls._pose_callback, queue_size=500
        )
        cls.twist_sub = rospy.Subscriber(
            "/ugv1/simulation/ground_truth/twist", TwistStamped, cls._twist_callback, queue_size=500
        )
        cls.imu_sub = rospy.Subscriber("/ugv1/imu", Imu, cls._imu_callback, queue_size=500)
        cls.voltage_sub = rospy.Subscriber(
            "/ugv1/PowerVoltage", Float32, cls._voltage_callback, queue_size=50
        )
        cls.model_sub = rospy.Subscriber("/gazebo/model_states", ModelStates, cls._model_callback, queue_size=1)
        cls.command_pub = rospy.Publisher("/ugv1/cmd_vel", Twist, queue_size=10)

        deadline = time.monotonic() + 20.0
        while not rospy.is_shutdown() and time.monotonic() < deadline:
            with cls.lock:
                ready = bool(cls.poses and cls.twists and cls.imus and cls.voltages and cls.model_states)
            if ready and cls.command_pub.get_num_connections() > 0:
                return
            rospy.sleep(0.02)
        raise RuntimeError("Mecanum Gazebo topics did not become ready")

    @classmethod
    def _pose_callback(cls, message):
        with cls.lock:
            cls.poses.append(message)
            del cls.poses[:-1000]

    @classmethod
    def _twist_callback(cls, message):
        with cls.lock:
            cls.twists.append(message)
            del cls.twists[:-1000]

    @classmethod
    def _imu_callback(cls, message):
        with cls.lock:
            cls.imus.append(message)
            del cls.imus[:-1000]

    @classmethod
    def _voltage_callback(cls, message):
        with cls.lock:
            cls.voltages.append((rospy.Time.now(), message))
            del cls.voltages[:-200]

    @classmethod
    def _model_callback(cls, message):
        with cls.lock:
            cls.model_states = message

    def setUp(self):
        self.publish_command(0.0, 0.0, 0.0, 0.12)

    def publish_command(self, x, y, yaw_rate, duration=0.12):
        message = Twist()
        message.linear.x = x
        message.linear.y = y
        message.angular.z = yaw_rate
        deadline = rospy.Time.now() + rospy.Duration(duration)
        rate = rospy.Rate(100)
        published = False
        while not rospy.is_shutdown() and (rospy.Time.now() < deadline or not published):
            self.command_pub.publish(message)
            published = True
            rate.sleep()

    def latest_pose(self):
        with self.lock:
            return self.poses[-1]

    def latest_twist(self):
        with self.lock:
            return self.twists[-1]

    @staticmethod
    def yaw_of(pose):
        q = pose.pose.orientation
        return euler_from_quaternion((q.x, q.y, q.z, q.w))[2]

    def test_01_topics_frames_and_rate(self):
        with self.lock:
            self.poses.clear()
            self.twists.clear()
            self.imus.clear()
            self.voltages.clear()
        deadline = time.monotonic() + 8.0
        while time.monotonic() < deadline:
            with self.lock:
                if (
                    len(self.poses) >= 50
                    and len(self.twists) >= 50
                    and len(self.imus) >= 20
                    and len(self.voltages) >= 8
                ):
                    break
            rospy.sleep(0.02)
        with self.lock:
            poses = list(self.poses)
            twists = list(self.twists)
            imus = list(self.imus)
            voltages = list(self.voltages)
            model_states = self.model_states

        self.assertGreaterEqual(len(poses), 50)
        self.assertEqual(poses[-1].header.frame_id, "map")
        self.assertEqual(twists[-1].header.frame_id, "map")
        self.assertEqual(imus[-1].header.frame_id, "world")
        elapsed = (poses[-1].header.stamp - poses[0].header.stamp).to_sec()
        rate = (len(poses) - 1) / elapsed
        self.assertGreater(rate, 90.0)
        self.assertLess(rate, 110.0)
        self.assertGreaterEqual(len(imus), 20)
        imu_elapsed = (imus[-1].header.stamp - imus[0].header.stamp).to_sec()
        imu_rate = (len(imus) - 1) / imu_elapsed
        self.assertGreater(imu_rate, 18.0)
        self.assertLess(imu_rate, 22.0)
        self.assertIn("ugv1", model_states.name)
        published_topics = dict(rospy.get_published_topics())
        self.assertNotIn("/ugv1/odom", published_topics)
        self.assertNotIn("/ugv1/pose", published_topics)
        self.assertNotIn("/ugv1/twist", published_topics)
        self.assertNotIn("/ugv1/diagnostic/pose", published_topics)
        self.assertNotIn("/ugv1/diagnostic/twist", published_topics)
        self.assertEqual(published_topics.get("/ugv1/simulation/ground_truth/pose"), "geometry_msgs/PoseStamped")
        self.assertEqual(published_topics.get("/ugv1/simulation/ground_truth/twist"), "geometry_msgs/TwistStamped")
        self.assertEqual(published_topics.get("/ugv1/PowerVoltage"), "std_msgs/Float32")
        self.assertGreaterEqual(len(voltages), 8)
        for _, message in voltages:
            self.assertAlmostEqual(message.data, 12.348, places=3)
        voltage_elapsed = (voltages[-1][0] - voltages[0][0]).to_sec()
        voltage_rate = (len(voltages) - 1) / voltage_elapsed
        self.assertGreater(voltage_rate, 1.2)
        self.assertLess(voltage_rate, 2.2)

    def test_02_body_xy_response_and_command_hold(self):
        start = self.latest_pose().pose.position
        self.publish_command(0.4, 0.2, 0.0, 0.15)
        rospy.sleep(0.25)
        twist = self.latest_twist().twist
        self.assertAlmostEqual(twist.linear.x, 0.4, delta=0.04)
        self.assertAlmostEqual(twist.linear.y, 0.2, delta=0.04)
        moved = self.latest_pose().pose.position
        self.assertGreater(moved.x - start.x, 0.08)
        self.assertGreater(moved.y - start.y, 0.035)

        # No watchdog: after publishing stops, Gazebo retains the last command.
        rospy.sleep(0.30)
        held = self.latest_twist().twist
        self.assertAlmostEqual(held.linear.x, 0.4, delta=0.04)
        self.assertAlmostEqual(held.linear.y, 0.2, delta=0.04)

    def test_03_yaw_response_and_limits(self):
        yaw_before = self.yaw_of(self.latest_pose())
        self.publish_command(0.0, 0.0, 0.5, 0.12)
        rospy.sleep(0.25)
        self.assertAlmostEqual(self.latest_twist().twist.angular.z, 0.5, delta=0.04)
        yaw_after = self.yaw_of(self.latest_pose())
        self.assertGreater(abs(math.atan2(math.sin(yaw_after - yaw_before), math.cos(yaw_after - yaw_before))), 0.08)

        self.publish_command(2.0, -2.0, 3.0, 0.15)
        rospy.sleep(0.10)
        pose = self.latest_pose()
        twist = self.latest_twist().twist
        yaw = self.yaw_of(pose)
        body_x = math.cos(yaw) * twist.linear.x + math.sin(yaw) * twist.linear.y
        body_y = -math.sin(yaw) * twist.linear.x + math.cos(yaw) * twist.linear.y
        self.assertAlmostEqual(body_x, 1.5, delta=0.08)
        self.assertAlmostEqual(body_y, -1.5, delta=0.08)
        self.assertAlmostEqual(twist.angular.z, math.pi / 2.0, delta=0.06)

        self.publish_command(0.0, 0.0, 0.0, 0.12)
        rospy.sleep(0.08)
        stopped = self.latest_twist().twist
        self.assertAlmostEqual(stopped.linear.x, 0.0, delta=0.02)
        self.assertAlmostEqual(stopped.linear.y, 0.0, delta=0.02)
        self.assertAlmostEqual(stopped.angular.z, 0.0, delta=0.02)
        pose = self.latest_pose().pose
        roll, pitch, _ = euler_from_quaternion(
            (pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w)
        )
        self.assertAlmostEqual(pose.position.z, 0.0, delta=0.005)
        self.assertAlmostEqual(roll, 0.0, delta=0.005)
        self.assertAlmostEqual(pitch, 0.0, delta=0.005)


if __name__ == "__main__":
    rostest.rosrun("gazebo_sim_mecanum", "ideal_drive_contract", IdealDriveContractTest)
