# XGC2 Gazebo Sim Mecanum

Gazebo Classic 11 product for a lightweight Nexus four-wheel Mecanum UGV. The
model is visual-only: it has no collision geometry, gravity, tire friction,
wheel force, PID, or `ros_control` path.

Reusable vehicle meshes and the visual URDF are owned by the independent
`mecanum_description` package. This simulation product owns only its Gazebo
model wrapper, ideal velocity plugin, launch files, and runtime contract.

The model plugin clips body-frame planar commands and sets Gazebo's model
velocity. Gazebo itself integrates the pose and owns simulation time, pause,
and reset. The plugin never computes or writes a pose.

## Interface

With the default namespace `ugv1`:

| Topic | Type | Rate | Meaning |
| --- | --- | --- | --- |
| `/ugv1/cmd_vel` | `geometry_msgs/Twist` | each Gazebo update | latest body x forward, body y left, +z CCW |
| `/ugv1/pose` | `geometry_msgs/PoseStamped` | 100 Hz | ground-truth pose in `map` |
| `/ugv1/twist` | `geometry_msgs/TwistStamped` | 100 Hz | world-frame velocity in `map` |
| `/ugv1/imu` | `sensor_msgs/Imu` | 100 Hz | ideal orientation and yaw rate |
| `/ugv1/joint_states` | `sensor_msgs/JointState` | 20 Hz | compatibility wheel animation state |

Defaults match the original SSS `ugv_sim_single.launch` contract: x/y limits are
1.5 m/s, yaw limit is 90 degrees/s,
scale factors are 1.0, and the last command is retained indefinitely. No
odometry message is published. `map -> ugv1/base_footprint` is broadcast at
20 Hz.

## Run

```bash
source /opt/ros/noetic/setup.bash
roslaunch gazebo_sim_mecanum simple.launch gui:=true
rostopic pub -r 20 /ugv1/cmd_vel geometry_msgs/Twist \
  '{linear: {x: 0.4, y: 0.2}, angular: {z: 0.3}}'
```

For an existing Gazebo server, use `spawn.launch`. Multiple robots require a
unique `ns` and `model_name` for each instance. The spawn process remains
attached and deletes its model when stopped.
