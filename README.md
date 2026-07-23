# XGC2 Gazebo Sim Mecanum

Gazebo Classic 11 product for the Nexus four-wheel Mecanum UGV. The default
`high_fidelity` drive model uses the original SSS mass, inertia, wheel geometry,
four wheel-speed inner loops, bounded wheel/ground slip forces, gravity, and
collision. Gazebo owns rigid-body integration and contact resolution.

The selectable `ideal` model preserves the lightweight SSS-compatible response:
the plugin applies the clipped planar velocity directly and Gazebo integrates
the pose without gravity or collision. Both modes expose exactly the same ROS
contract and retain the latest command indefinitely.

Reusable vehicle meshes and the visual URDF are owned by the independent
`mecanum_description` package. This simulation product owns its Gazebo model,
dynamics plugin, launch files, tests, and runtime contract.

## Interface

With the default namespace `ugv1`:

| Topic | Type | Rate | Meaning |
| --- | --- | --- | --- |
| `/ugv1/cmd_vel` | `geometry_msgs/Twist` | input | latest body x forward, body y left, +z CCW |
| `/ugv1/pose` | `geometry_msgs/PoseStamped` | 100 Hz | ground-truth pose in `map` |
| `/ugv1/twist` | `geometry_msgs/TwistStamped` | 100 Hz | world-frame velocity in `map` |
| `/ugv1/imu` | `sensor_msgs/Imu` | 100 Hz | orientation and angular rate |
| `/ugv1/joint_states` | `sensor_msgs/JointState` | 20 Hz | renderer-only wheel angles reconstructed from body motion |

Internal physical wheel rates, effort, slip, and controller state are not
published. The public wheel angles are reconstructed from actual body motion,
so RViz and Lichtblick use the same animation path for simulation and physical
robots.

Defaults preserve the original SSS outer contract: x/y limits are 1.5 m/s,
yaw limit is 90 degrees/s, scale factors are 1.0, and there is no command
watchdog. No odometry message is published. `map -> ugv1/base_footprint` is
broadcast at 20 Hz.

## Drive models

- `drive_model:=high_fidelity` (default): approximate first-order body response,
  wheel-speed inner loops, Mecanum traction/slip saturation, gravity, wheel
  contact, and chassis collision.
- `drive_model:=ideal`: collision-free direct velocity response for large,
  inexpensive swarm runs.

The high-fidelity model intentionally retains small damping, steady-state error,
and coupled-command effects instead of forcing exact command tracking.

## Run

```bash
source /opt/ros/noetic/setup.bash
roslaunch gazebo_sim_mecanum simple.launch gui:=true drive_model:=high_fidelity
rostopic pub -r 20 /ugv1/cmd_vel geometry_msgs/Twist \
  '{linear: {x: 0.4, y: 0.2}, angular: {z: 0.3}}'
```

For an existing Gazebo server, use `spawn.launch`. Multiple robots require a
unique `ns` and `model_name` for each instance. Set `drive_model:=ideal` per
robot when a lightweight model is preferred. Packaged process definitions pin
the installed launch file, model meshes, and Gazebo plugin to their canonical
absolute paths. Source development stages a separate immutable release pointing
directly at the checked-out `spawn.launch`, mesh directory, and freshly built
plugin; it never falls back to a stale or missing `/opt` package.
