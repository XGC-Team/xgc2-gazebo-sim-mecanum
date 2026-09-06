#!/usr/bin/env python3
import math
import socket
import struct
import threading
import time
import unittest

import rospy
import rostest
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import DeleteModel, GetJointProperties, SetModelState, SpawnModel
from geometry_msgs.msg import PoseStamped, Twist, TwistStamped
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32, Float64MultiArray


class HighFidelityDriveContractTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rospy.init_node("high_fidelity_drive_contract_test", anonymous=True)
        cls.lock = threading.Lock()
        cls.pose = None
        cls.twists = []
        cls.joint_state = None
        cls.voltage = None
        cls.traction = []
        rospy.Subscriber("/ugv1/simulation/traction", Float64MultiArray, cls._traction_callback, queue_size=2000)
        rospy.Subscriber("/ugv1/simulation/ground_truth/pose", PoseStamped, cls._pose_callback, queue_size=1)
        rospy.Subscriber("/ugv1/simulation/ground_truth/twist", TwistStamped, cls._twist_callback, queue_size=500)
        rospy.Subscriber("/ugv1/joint_states", JointState, cls._joint_callback, queue_size=1)
        rospy.Subscriber("/ugv1/PowerVoltage", Float32, cls._voltage_callback, queue_size=10)
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
                ready = (
                    cls.pose is not None
                    and bool(cls.twists)
                    and cls.joint_state is not None
                    and cls.voltage is not None
                )
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

    @classmethod
    def _voltage_callback(cls, message):
        with cls.lock:
            cls.voltage = message

    @classmethod
    def _traction_callback(cls, message):
        with cls.lock:
            cls.traction.append(list(message.data))
            del cls.traction[:-4000]

    def test_04_contact_loss_rollover_and_recontact(self):
        self.publish_command(0, 0, 0, 0.5)
        state = ModelState()
        state.model_name = "ugv1"
        state.reference_frame = "world"
        state.pose.orientation.w = 1
        state.pose.position.z = 4
        self.assertTrue(self.set_model(state).success)
        rospy.sleep(0.04)  # Discard contact data from the previous physics step.
        with self.lock:
            self.traction.clear()
        self.publish_command(0.8, 0.3, 0.5, 0.25)
        with self.lock:
            airborne = list(self.traction)
        self.assertGreater(len(airborne), 30)
        self.assertTrue(all(len(row) == 8 for row in airborne))
        self.assertTrue(all(max(abs(v) for v in row) < 1e-9 for row in airborne), airborne[-5:])
        self.assertLess(math.hypot(self.latest_twist().linear.x, self.latest_twist().linear.y), 0.03)

        self.publish_command(0, 0, 0, 0.1)
        pedestal = """<sdf version='1.6'><model name='roof_support'><static>true</static>
          <link name='support'><pose>0 0 0.1 0 0 0</pose><collision name='collision'>
          <geometry><box><size>0.20 0.20 0.20</size></box></geometry>
          </collision></link></model></sdf>"""
        origin = ModelState().pose
        origin.orientation.w = 1
        self.assertTrue(self.spawn_model("roof_support", pedestal, "", origin, "world").success)
        state.pose.position.z = 0.4
        state.pose.orientation.w = 0
        state.pose.orientation.x = 1  # Rest on chassis roof, wheels above ground.
        self.assertTrue(self.set_model(state).success)
        rospy.sleep(0.5)
        with self.lock:
            self.traction.clear()
        self.publish_command(0.8, 0.3, 0.5, 0.3)
        with self.lock:
            inverted = list(self.traction)
        self.assertGreater(len(inverted), 30)
        self.assertTrue(all(max(abs(v) for v in row) < 1e-9 for row in inverted), inverted[-5:])

        self.publish_command(0, 0, 0, 0.1)
        self.assertTrue(self.delete_model("roof_support").success)
        state.pose.position.z = 0
        state.pose.orientation.w = 1
        state.pose.orientation.x = 0
        self.assertTrue(self.set_model(state).success)
        rospy.sleep(0.6)
        with self.lock:
            self.traction.clear()
        self.publish_command(0.8, 0, 0, 1.2)
        with self.lock:
            recovered = list(self.traction)
        self.assertTrue(any(sum(row[:4]) > 10 and sum(abs(v) for v in row[4:]) > 0.1 for row in recovered))
        self.assertGreater(self.latest_twist().linear.x, 0.65)
        self.publish_command(0, 0, 0, 0.5)

    def test_05_partial_contact(self):
        state = ModelState()
        state.model_name = "ugv1"
        state.reference_frame = "world"
        state.pose.position.z = 0.12
        state.pose.orientation.x = math.sin(0.18)
        state.pose.orientation.w = math.cos(0.18)
        self.assertTrue(self.set_model(state).success)
        with self.lock:
            self.traction.clear()
        self.publish_command(0.8, 0, 0, 0.5)
        with self.lock:
            samples = list(self.traction)
        partial = [row for row in samples if 0 < sum(v > 1e-6 for v in row[:4]) < 4]
        self.assertGreater(len(partial), 5, "tilted landing must exercise partially supported wheels")
        for row in partial:
            for index, load in enumerate(row[:4]):
                if load == 0:
                    self.assertEqual(row[index + 4], 0)
                self.assertLessEqual(abs(row[index + 4]), 0.85 * load + 1e-9)
        self.publish_command(0, 0, 0, 0.5)

    def test_06_hold_and_repeated_model_lifetime(self):
        def hold(robot, value, sequence, deletion=None):
            digest = 2166136261
            for byte in robot.encode():
                digest = ((digest ^ byte) * 16777619) & 0xffffffff
            with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as client:
                client.settimeout(2)
                client.sendto(struct.pack("<IBBHI32s", 0x58474348, 1, value, 0,
                                          sequence, robot.encode()),
                              ("127.0.0.1", 20000 + digest % 20000))
                ack, _ = client.recvfrom(1024)
            self.assertEqual(len(ack), 12)
            # Registry removal precedes socket close. A request racing that
            # interval is explicitly rejected, rather than calling a dead Gate.
            status = 1 if deletion is not None and deletion.is_set() and ack[6] == 1 else 0
            self.assertEqual(ack, struct.pack("<IBBBBI", 0x58474348, 1, value, status, 0, sequence))

        source = rospy.get_param("/ugv1/gazebo_model_sdf").replace("ugv1", "ugv2")
        pose = ModelState().pose
        pose.orientation.w = 1
        pose.position.y = 3
        second_command = rospy.Publisher("/ugv2/cmd_vel", Twist, queue_size=1)
        moving = Twist()
        moving.linear.x = 0.8
        for cycle in range(4):
            self.assertTrue(self.spawn_model("ugv2", source, "", pose, "world").success)
            rospy.sleep(0.1)
            hold("ugv2", True, 10 + cycle)
            self.publish_command(0.8, 0, 0, 0.7)
            self.assertGreater(self.latest_twist().linear.x, 0.5)
            hold("ugv1", True, 20 + cycle)
            self.publish_command(0.8, 0, 0, 1.5)
            self.assertLess(math.hypot(self.latest_twist().linear.x, self.latest_twist().linear.y), 0.04)
            hold("ugv1", False, 30 + cycle)
            rospy.sleep(0.15)
            self.assertLess(math.hypot(self.latest_twist().linear.x, self.latest_twist().linear.y), 0.04)
            self.assertGreater(second_command.get_num_connections(), 0)
            stopping = threading.Event()
            deletion = threading.Event()
            errors = []
            def concurrent_hold():
                while not stopping.is_set():
                    try:
                        second_command.publish(moving)
                        hold("ugv2", True, 40 + cycle, deletion)
                    except (socket.timeout, ConnectionRefusedError):
                        pass  # The endpoint is absent during intentional deletion.
                    except Exception as error:
                        errors.append(error)
            sender = threading.Thread(target=concurrent_hold)
            sender.start()
            rospy.sleep(0.1)
            try:
                deletion.set()
                self.assertTrue(self.delete_model("ugv2").success)
            finally:
                stopping.set()
                sender.join(timeout=3)
            self.assertFalse(sender.is_alive())
            self.assertFalse(errors)
        self.publish_command(0, 0, 0, 0.3)

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

    def test_01_first_order_response_and_renderer_wheel_state(self):
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
        self.assertEqual(len(joint_state.position), 4)
        self.assertFalse(joint_state.velocity)
        self.assertFalse(joint_state.effort)
        published_topics = dict(rospy.get_published_topics())
        self.assertFalse(any(topic.startswith("/ugv1/wheel_") for topic in published_topics))
        self.assertNotIn("/ugv1/pose", published_topics)
        self.assertNotIn("/ugv1/twist", published_topics)
        self.assertNotIn("/ugv1/diagnostic/pose", published_topics)
        self.assertNotIn("/ugv1/diagnostic/twist", published_topics)
        self.assertEqual(published_topics.get("/ugv1/simulation/ground_truth/pose"), "geometry_msgs/PoseStamped")
        self.assertEqual(published_topics.get("/ugv1/simulation/ground_truth/twist"), "geometry_msgs/TwistStamped")
        self.assertEqual(published_topics.get("/ugv1/PowerVoltage"), "std_msgs/Float32")
        with self.lock:
            voltage = self.voltage
        self.assertIsNotNone(voltage)
        self.assertAlmostEqual(voltage.data, 12.348, places=3)

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
