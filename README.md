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
| `/ugv1/simulation/ground_truth/pose` | `geometry_msgs/PoseStamped` | 100 Hz | simulation ground-truth pose in `map`; not the Experiment canonical pose |
| `/ugv1/simulation/ground_truth/twist` | `geometry_msgs/TwistStamped` | 100 Hz | simulation ground-truth world velocity in `map`; not the Experiment canonical twist |
| `/ugv1/imu` | `sensor_msgs/Imu` | 20 Hz | orientation and angular rate. Matches Wheeltec MCU `/imu` and swarm-ros-bridge send `max_freq=20` (`:3001`). |
| `/ugv1/PowerVoltage` | `std_msgs/Float32` | 1 Hz | fixed chassis voltage in volts; default 12.348 V (88% of Core `mecanum_ugv.3s_lipo` 10.5–12.6 V). Same name/type as the Wheeltec MCU topic. Rate matches swarm-ros-bridge send `max_freq=1` (`:3002`), not the onboard MCU ~1.67 Hz. Simulation does not run swarm-ros-bridge. |
| `/ugv1/joint_states` | `sensor_msgs/JointState` | 20 Hz | renderer-only wheel angles reconstructed from body motion |

Internal physical wheel rates, effort, slip, and controller state are not
published. The public wheel angles are reconstructed from actual body motion,
so RViz and Lichtblick use the same animation path for simulation and physical
robots.

Defaults preserve the original SSS outer contract: x/y limits are 1.5 m/s,
yaw limit is 90 degrees/s, scale factors are 1.0, and there is no command
watchdog. No odometry message is published. The plugin does not advertise
canonical `/<namespace>/pose` or `/<namespace>/twist`; those topics are owned
by `experiment-localization-projection`. `map -> ugv1/base_footprint` is
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

## Contact-bound traction regression

The high-fidelity approximation reads Gazebo's active wheel contact forces.
Each wheel's longitudinal roller effort is bounded by its measured normal load
and `frictionCoefficient`; unsupported wheels contribute zero ground traction.
Forces act along the contact tangent at the actual contact point, with an equal
opposite reaction on a dynamic contacted body. Existing motor inertia, internal
wheel torque, linear/angular drag and authored yaw response scaling remain
separate. The planar roller slip approximation is not a detailed roller/contact
material model. Gazebo contact feedback reflects the preceding physics update.

`/<robot>/simulation/traction` publishes `Float64MultiArray` when subscribed:
first four entries are normal loads (N), last four are bounded roller efforts
(N), ordered upper-left, upper-right, lower-left, lower-right. These are simulator
diagnostics, not measured hardware forces. Contact feedback is enabled globally
in the world's ContactManager so evidence exists without a contact sensor.

In an isolated ROS Noetic/Gazebo 11 catkin workspace containing this package and
`mecanum_description`, build then run:

```bash
catkin_make -DCMAKE_BUILD_TYPE=Release
source devel/setup.bash
rostest gazebo_sim_mecanum high_fidelity_drive.test
```

Six runtime tests cover flat forward/sideways/yaw response, physical wheels,
gravity/wall collision, airborne and roof-supported inverted zero traction,
recontact, tilted partial support, and repeated HOLD/release plus model
spawn/delete. They use disposable simulation models only. HOLD checks allow
1.5 simulation seconds for physical braking; acknowledgment does not claim
instantaneous removal of inertia. The deterministic Gate suite separately
checks serialized target writes and callback lifetime with sanitizers.
