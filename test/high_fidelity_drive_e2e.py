#!/usr/bin/env python3
import math
import threading
import time
import unittest

import rospy
import rostest
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import DeleteModel, GetJointProperties, SetModelState, SpawnModel
from geometry_msgs.msg import PoseStamped, Twist, TwistStamped
from sensor_msgs.msg import JointState


class HighFidelityDriveContractTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rospy.init_node("high_fidelity_drive_contract_test", anonymous=True)
        cls.lock = threading.Lock()
        cls.pose = None
        cls.twists = []
        cls.joint_state = None
        rospy.Subscriber("/ugv1/pose", PoseStamped, cls._pose_callback, queue_size=1)
        rospy.Subscriber("/ugv1/twist", TwistStamped, cls._twist_callback, queue_size=500)
        rospy.Subscriber("/ugv1/joint_states", JointState, cls._joint_callback, queue_size=1)
        cls.command_pub = rospy.Publisher("/ugv1/cmd_vel", Twist, queue_size=1)
        for service in (
            "/gazebo/get_joint_properties",
            "/gazebo/set_model_state",
            "/gazebo/spawn_sdf_model",
            "/gazebo/delete_model",
        ):
            rospy.wait_for_service(service, timeout=20.0)
        cls.get_joint = rospy.ServiceProxy("/gazebo/get_joint_properties", GetJointProperties)
        cls.set_model = rospy.ServiceProxy("/gazebo/set_model_state", SetModelState)
        cls.spawn_model = rospy.ServiceProxy("/gazebo/spawn_sdf_model", SpawnModel)
        cls.delete_model = rospy.ServiceProxy("/gazebo/delete_model", DeleteModel)

        deadline = time.monotonic() + 20.0
        while time.monotonic() < deadline and not rospy.is_shutdown():
            with cls.lock:
                ready = cls.pose is not None and bool(cls.twists) and cls.joint_state is not None
            if ready and cls.command_pub.get_num_connections() > 0:
                return
            rospy.sleep(0.02)
        raise RuntimeError("high-fidelity Mecanum topics did not become ready")

    @classmethod
    def _pose_callback(cls, message):
        with cls.lock:
            cls.pose = message

    @classmethod
    def _twist_callback(cls, message):
        with cls.lock:
            cls.twists.append(message)
            del cls.twists[:-2000]

    @classmethod
    def _joint_callback(cls, message):
        with cls.lock:
            cls.joint_state = message

    def publish_command(self, x, y, yaw_rate, duration):
        message = Twist()
        message.linear.x = x
        message.linear.y = y
        message.angular.z = yaw_rate
        deadline = rospy.Time.now() + rospy.Duration(duration)
        rate = rospy.Rate(100)
        while not rospy.is_shutdown() and rospy.Time.now() < deadline:
            self.command_pub.publish(message)
            rate.sleep()

    def latest_twist(self):
        with self.lock:
            return self.twists[-1].twist

    def latest_pose(self):
        with self.lock:
            return self.pose.pose

    def clear_twists(self):
        with self.lock:
            self.twists.clear()

    def test_01_first_order_response_and_private_wheel_state(self):
        self.publish_command(0.0, 0.0, 0.0, 0.8)
        self.clear_twists()
        self.publish_command(0.8, 0.0, 0.0, 1.5)
        with self.lock:
            samples = list(self.twists)
            joint_state = self.joint_state

        self.assertGreater(len(samples), 80)
        steady = sum(message.twist.linear.x for message in samples[-20:]) / 20.0
        peak = max(message.twist.linear.x for message in samples)
        self.assertGreater(steady, 0.65)
        self.assertLess(steady, 0.85)
        self.assertLess(peak, 0.95)
        threshold = steady * 0.632
        start = samples[0].header.stamp
        crossing = next(message for message in samples if message.twist.linear.x >= threshold)
        tau = (crossing.header.stamp - start).to_sec()
        self.assertGreater(tau, 0.03)
        self.assertLess(tau, 0.45)

        # The public state remains a low-rate renderer contract. Simulator
        # wheel rates and efforts stay private to the dynamics plugin.
        self.assertEqual(len(joint_state.name), 4)
        self.assertEqual(joint_state.velocity, [])
        self.assertEqual(joint_state.effort, [])
        published_topics = dict(rospy.get_published_topics())
        self.assertFalse(any(topic.startswith("/ugv1/wheel_") for topic in published_topics))

        # The last command is held, matching the original SSS outer contract.
        rospy.sleep(0.4)
        self.assertGreater(self.latest_twist().linear.x, 0.65)

    def test_02_lateral_yaw_and_physical_wheels(self):
        self.publish_command(0.0, 0.0, 0.0, 0.8)
        self.publish_command(0.0, 0.6, 0.0, 1.2)
        lateral = self.latest_twist()
        self.assertGreater(lateral.linear.y, 0.48)
        self.assertLess(lateral.linear.y, 0.70)
        self.assertLess(abs(lateral.linear.x), 0.06)

        self.publish_command(0.0, 0.0, 0.0, 0.8)
        self.publish_command(0.0, 0.0, 0.8, 1.2)
        yaw = self.latest_twist()
        self.assertGreater(yaw.angular.z, 0.62)
        self.assertLess(yaw.angular.z, 0.90)
        self.assertLess(math.hypot(yaw.linear.x, yaw.linear.y), 0.06)

        rates = []
        for name in (
            "upper_left_wheel_joint",
            "upper_right_wheel_joint",
            "lower_left_wheel_joint",
            "lower_right_wheel_joint",
        ):
            response = self.get_joint("ugv1::" + name)
            self.assertTrue(response.success)
            self.assertEqual(len(response.rate), 1)
            rates.append(response.rate[0])
        self.assertTrue(all(abs(rate) > 1.0 for rate in rates))
        self.assertTrue(all(rate < 0.0 for rate in rates))

    def test_03_gravity_and_collision(self):
        self.publish_command(0.0, 0.0, 0.0, 0.8)
        state = ModelState()
        state.model_name = "ugv1"
        state.pose.orientation.w = 1.0
        state.reference_frame = "world"
        self.assertTrue(self.set_model(state).success)
        rospy.sleep(0.5)
        self.assertAlmostEqual(self.latest_pose().position.z, -0.001, delta=0.006)

        wall_name = "mecanum_contract_test_wall"
        try:
            self.delete_model(wall_name)
        except rospy.ServiceException:
            pass
        wall = """<sdf version='1.6'><model name='mecanum_contract_test_wall'><static>true</static>
          <link name='wall'><pose>0.60 0 0.15 0 0 0</pose>
          <collision name='collision'><geometry><box><size>0.10 0.80 0.30</size></box></geometry></collision>
          </link></model></sdf>"""
        spawn = self.spawn_model(wall_name, wall, "", state.pose, "world")
        self.assertTrue(spawn.success)
        try:
            self.publish_command(0.8, 0.0, 0.0, 2.0)
            x = self.latest_pose().position.x
            self.assertGreater(x, 0.25)
            self.assertLess(x, 0.43)
        finally:
            self.publish_command(0.0, 0.0, 0.0, 0.5)
            self.delete_model(wall_name)


if __name__ == "__main__":
    rostest.rosrun(
        "gazebo_sim_mecanum",
        "high_fidelity_drive_contract",
        HighFidelityDriveContractTest,
    )
