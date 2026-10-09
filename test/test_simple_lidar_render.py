#!/usr/bin/env python3
"""Render-only checks for the optional Wheeltec simple lidar mount."""

import os
import math
from pathlib import Path
import shutil
import subprocess
import unittest
import xml.etree.ElementTree as ET


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
MODEL_XACRO = PACKAGE_ROOT / "models/xgc2_mecanum_ugv/model.sdf.xacro"
MODEL_SDF = PACKAGE_ROOT / "models/xgc2_mecanum_ugv/model.sdf"
SIMPLE_LAUNCH = PACKAGE_ROOT / "launch/simple.launch"
SPAWN_LAUNCH = PACKAGE_ROOT / "launch/spawn.launch"
XACRO = shutil.which("xacro")


def installed_simple_lidar_package():
    package = Path(subprocess.check_output(
        ["rospack", "find", "xgc2_simple_lidar"], text=True
    ).strip()).resolve()
    roscpp = Path(subprocess.check_output(
        ["rospack", "find", "roscpp"], text=True
    ).strip()).resolve()
    if package.parent != roscpp.parent:
        raise AssertionError("xgc2_simple_lidar must resolve from the installed ROS share directory")
    if not (package / "models" / "sensor.xacro").is_file():
        raise AssertionError("installed xgc2_simple_lidar has no models/sensor.xacro")
    return package


def render_model(drive_model="high_fidelity", namespace="ugv1", enabled=False,
                 pose="0 0 0.28 0 0 0", extra=None):
    if XACRO is None:
        raise AssertionError("xacro executable is required for render checks")

    env = os.environ.copy()
    if enabled:
        shared_package = installed_simple_lidar_package()
        env["ROS_PACKAGE_PATH"] = str(shared_package.parent)
    else:
        # Keep the public lidar package out of rospack's search path. A default
        # render must succeed without resolving xgc2_simple_lidar at all.
        env["ROS_PACKAGE_PATH"] = str(PACKAGE_ROOT)
    env.pop("CMAKE_PREFIX_PATH", None)

    command = [
        XACRO,
        str(MODEL_XACRO),
        "robot_namespace:={}".format(namespace),
        "chassis_robot_id:={}".format(namespace),
        "drive_model:={}".format(drive_model),
        "enable_simple_lidar:={}".format("true" if enabled else "false"),
        "simple_lidar_pose:={}".format(pose),
    ]
    command.extend(extra or [])
    result = subprocess.run(command, cwd=str(PACKAGE_ROOT), env=env,
                            stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                            universal_newlines=True)
    if result.returncode:
        raise AssertionError("xacro render failed:\n{}".format(result.stderr))
    return parse_xml(result.stdout)


def parse_xml(source):
    parser = ET.XMLParser(target=ET.TreeBuilder(insert_comments=True))
    return ET.fromstring(source, parser=parser)


def normalized_xml(element):
    """Compare XML values while ignoring formatting and the outer xacro banner."""
    if element.tag is ET.Comment:
        return ("#comment", element.text or "")
    return (
        element.tag,
        tuple(sorted(element.attrib.items())),
        (element.text or "").strip(),
        tuple(normalized_xml(child) for child in list(element)),
    )


def model_link(root):
    link = root.find(".//link[@name='base_footprint']")
    if link is None:
        raise AssertionError("rendered model has no base_footprint link")
    return link


def sensor_pose(sensor):
    pose = sensor.findtext("pose")
    if pose is None:
        raise AssertionError("rendered lidar sensor has no pose")
    return tuple(float(value) for value in pose.split())


class SimpleLidarRenderTest(unittest.TestCase):
    def test_cpu_native_scan_keeps_authored_parameters(self):
        root = render_model(namespace='ugv2', enabled=True,
                            extra=['simple_lidar_acceleration:=cpu', 'simple_lidar_rate_hz:=12',
                                   'simple_lidar_range_meters:=8', 'simple_lidar_hfov_deg:=120',
                                   'simple_lidar_vfov_deg:=40', 'simple_lidar_hres:=90',
                                   'simple_lidar_vres:=8'])
        sensor = root.find(".//sensor[@name='simple_lidar']")
        self.assertEqual(sensor.get('type'), 'ray')
        self.assertEqual(sensor.find('plugin').get('filename'), 'libxgc2_simple_lidar_cpu.so')
        self.assertEqual(sensor.findtext('plugin/robotNamespace'), 'ugv2')
        self.assertEqual(float(sensor.findtext('update_rate')), 12)
        self.assertEqual(float(sensor.findtext('ray/range/max')), 8)
        for direction, count, degrees in [('horizontal', 90, 120), ('vertical', 8, 40)]:
            scan = sensor.find('ray/scan/' + direction)
            self.assertEqual(int(scan.findtext('samples')), count)
            self.assertAlmostEqual(float(scan.findtext('max_angle')) - float(scan.findtext('min_angle')),
                                   math.radians(degrees))
    def test_disabled_default_matches_checked_in_model_without_public_package(self):
        rendered = render_model()
        baseline = parse_xml(MODEL_SDF.read_text())
        self.assertEqual(normalized_xml(baseline), normalized_xml(rendered))
        self.assertEqual([], rendered.findall(".//sensor"))

    def test_launch_parameters_reach_the_sdf_xacro(self):
        xacro_ns = {"xacro": "http://www.ros.org/wiki/xacro"}
        xacro = ET.parse(str(MODEL_XACRO)).getroot()
        xacro_args = {arg.get("name"): arg.get("default")
                      for arg in xacro.findall("xacro:arg", xacro_ns)}
        self.assertEqual("false", xacro_args["enable_simple_lidar"])
        self.assertEqual("0 0 0.28 0 0 0", xacro_args["simple_lidar_pose"])

        simple = ET.parse(str(SIMPLE_LAUNCH)).getroot()
        simple_args = {arg.get("name"): arg.get("default")
                       for arg in simple.findall("arg")}
        self.assertEqual("false", simple_args["enable_simple_lidar"])
        self.assertEqual("0 0 0.28 0 0 0", simple_args["simple_lidar_pose"])
        simple_spawn_include = simple.find("include[@file='$(dirname)/spawn.launch']")
        self.assertIsNotNone(simple_spawn_include)
        forwarded = {arg.get("name"): arg.get("value")
                     for arg in simple_spawn_include.findall("arg")}
        self.assertEqual("$(arg enable_simple_lidar)", forwarded["enable_simple_lidar"])
        self.assertEqual("$(arg simple_lidar_pose)", forwarded["simple_lidar_pose"])

        spawn = ET.parse(str(SPAWN_LAUNCH)).getroot()
        spawn_args = {arg.get("name"): arg.get("default")
                      for arg in spawn.findall("arg")}
        self.assertEqual("false", spawn_args["enable_simple_lidar"])
        self.assertEqual("0 0 0.28 0 0 0", spawn_args["simple_lidar_pose"])
        sdf_param = spawn.find("param[@name='/$(arg ns)/gazebo_model_sdf']")
        self.assertIsNotNone(sdf_param)
        command = sdf_param.get("command", "")
        self.assertIn("enable_simple_lidar:=$(arg enable_simple_lidar)", command)
        self.assertIn("simple_lidar_pose:='$(arg simple_lidar_pose)'", command)

    def test_both_drive_models_render_one_namespaced_sensor_at_the_mount_pose(self):
        installed_pose = (0.0, 0.0, 0.28, 0.0, 0.0, 0.0)
        topics = {}

        for drive_model in ("high_fidelity", "ideal"):
            for namespace in ("ugv1", "ugv2"):
                with self.subTest(drive_model=drive_model, namespace=namespace):
                    disabled = render_model(drive_model=drive_model,
                                            namespace=namespace, enabled=False)
                    enabled = render_model(drive_model=drive_model,
                                           namespace=namespace, enabled=True)
                    link = model_link(enabled)
                    sensors = link.findall("sensor")
                    self.assertEqual(1, len(sensors))
                    self.assertEqual(1, len(enabled.findall(".//sensor")))
                    sensor = sensors[0]
                    self.assertEqual("simple_lidar", sensor.get("name"))
                    self.assertEqual("gpu_ray", sensor.get("type"))
                    self.assertEqual(installed_pose, sensor_pose(sensor))

                    plugins = sensor.findall("plugin")
                    self.assertEqual(1, len(plugins))
                    self.assertEqual(namespace,
                                     plugins[0].findtext("robotNamespace", "").strip())
                    topics[namespace] = "/{}/simple_lidar/points".format(namespace)

                    # The optional mount is the sole SDF change in either mode.
                    link.remove(sensor)
                    self.assertEqual(normalized_xml(disabled), normalized_xml(enabled))

        self.assertEqual("/ugv1/simple_lidar/points", topics["ugv1"])
        self.assertEqual("/ugv2/simple_lidar/points", topics["ugv2"])
        self.assertNotEqual(topics["ugv1"], topics["ugv2"])


if __name__ == "__main__":
    unittest.main()
