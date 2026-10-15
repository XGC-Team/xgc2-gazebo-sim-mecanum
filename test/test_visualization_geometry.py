#!/usr/bin/env python3
"""Execute the production visualization calculation with controlled body motion."""
import json
import os
from pathlib import Path
import shlex
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]
SOURCE = Path(os.environ.get("XGC2_MECANUM_PLUGIN_SOURCE", ROOT / "src/mecanum_contract_plugin.cpp"))


class VisualizationGeometryTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        source = SOURCE.read_text()
        signature = "  void PublishVisualizationState(double sim_time) {"
        start = source.index(signature) + len(signature)
        end = source.index("    ros::Time stamp;", start)
        # Compile the actual production transform, wheel rates and angle integration.
        # The model stand-in supplies measured motion; no formula is copied into the fixture.
        calculation = source[start:end]
        cls.directory = tempfile.TemporaryDirectory(prefix="mecanum-geometry-")
        cls.addClassCleanup(cls.directory.cleanup)
        root = Path(cls.directory.name)
        fixture = root / "visualization_geometry.cpp"
        fixture.write_text(r'''
#include <array>
#include <cmath>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <memory>
namespace ignition::math {
struct Vector3d {
  double x, y, z;
  double X() const { return x; }
  double Y() const { return y; }
  double Z() const { return z; }
};
struct Rotation {
  double yaw;
  double Yaw() const { return yaw; }
};
struct Pose3d {
  Rotation rotation;
  Rotation Rot() const { return rotation; }
};
}
struct Model {
  ignition::math::Pose3d pose;
  ignition::math::Vector3d linear, angular;
  auto WorldPose() const { return pose; }
  auto WorldLinearVel() const { return linear; }
  auto WorldAngularVel() const { return angular; }
};
constexpr double kPi = 3.14159265358979323846;
struct Visualization {
  std::shared_ptr<Model> model_ = std::make_shared<Model>();
  double wheel_radius_, wheelbase_sum_, last_visual_update_time_ = 0;
  std::array<double, 4> wheel_positions_{};
  void PublishVisualizationState(double sim_time) {
''' + calculation + r'''
  }
};
int main(int argc, char** argv) {
  if (argc != 7) return 2;
  Visualization state;
  state.wheel_radius_ = std::strtod(argv[1], nullptr);
  state.wheelbase_sum_ = std::strtod(argv[2], nullptr);
  state.model_->linear = {std::strtod(argv[3], nullptr), std::strtod(argv[4], nullptr), 0};
  state.model_->angular = {0, 0, std::strtod(argv[5], nullptr)};
  state.model_->pose = {{std::strtod(argv[6], nullptr)}};
  state.PublishVisualizationState(1.0);
  for (const double angle : state.wheel_positions_) if (angle != 0) return 3;
  state.PublishVisualizationState(1.1);
  std::cout << std::setprecision(17) << '[';
  for (std::size_t i = 0; i < 4; ++i) {
    if (i) std::cout << ',';
    std::cout << state.wheel_positions_[i];
  }
  std::cout << "]\n";
}
''')
        cls.executable = root / "visualization_geometry"
        subprocess.run(shlex.split(os.environ.get("CXX", "c++")) +
                       ["-std=c++17", "-Wall", "-Wextra", "-Werror", str(fixture),
                        "-o", str(cls.executable)], check=True, capture_output=True, text=True, timeout=30)

    def check_angles(self, radius, wheelbase, expected, *, world_x=.6, world_y=.2, yaw=0):
        result = subprocess.run([str(self.executable), str(radius), str(wheelbase),
                                 str(world_x), str(world_y), ".4", str(yaw)],
                                check=True, capture_output=True, text=True, timeout=5)
        angles = json.loads(result.stdout)
        self.assertEqual(len(angles), 4)
        for actual, target in zip(angles, expected):
            self.assertAlmostEqual(actual, target, places=10)

    def test_default_geometry_preserves_animation(self):
        self.check_angles(.05, .30, [.56, -1.84, 1.36, -1.04])

    def test_nondefault_radius(self):
        self.check_angles(.08, .30, [.35, -1.15, .85, -.65])

    def test_nondefault_wheelbase(self):
        self.check_angles(.05, .42, [.464, -1.936, 1.264, -1.136])

    def test_nondefault_radius_and_wheelbase(self):
        self.check_angles(.08, .42, [.29, -1.21, .79, -.71])

    def test_nondefault_geometry_with_rotated_body(self):
        self.check_angles(.08, .42, [.29, -1.21, .79, -.71],
                          world_x=-.2, world_y=.6, yaw=1.5707963267948966)


if __name__ == "__main__":
    unittest.main()
