#!/usr/bin/env bash
set -euo pipefail

ROS_DISTRO="${ROS_DISTRO:-noetic}"
source "/opt/ros/${ROS_DISTRO}/setup.bash"

dpkg -s ros-noetic-xgc2-gazebo-sim-mecanum >/dev/null
test "$(rospack find gazebo_sim_mecanum)" = "/opt/ros/${ROS_DISTRO}/share/gazebo_sim_mecanum"
test -f "/opt/ros/${ROS_DISTRO}/lib/libgazebo_sim_mecanum_contract.so"
test -x "/opt/ros/${ROS_DISTRO}/lib/gazebo_sim_mecanum/check_model_ready.py"
test -f "/opt/ros/${ROS_DISTRO}/share/gazebo_sim_mecanum/models/xgc2_mecanum_ugv/model.sdf"
test -f "/opt/ros/${ROS_DISTRO}/share/mecanum_description/meshes/mecanum_wheel_left.STL"
test -f "/usr/share/xgc2/process-definitions/xgc2-gazebo-sim-mecanum.json"
python3 -m json.tool /usr/share/xgc2/process-definitions/xgc2-gazebo-sim-mecanum.json >/dev/null

roslaunch --files gazebo_sim_mecanum simple.launch gui:=false >/tmp/xgc2-mecanum-simple-files.txt
roslaunch --files gazebo_sim_mecanum spawn.launch >/tmp/xgc2-mecanum-spawn-files.txt
GAZEBO_MODEL_PATH="/opt/ros/${ROS_DISTRO}/share:/opt/ros/${ROS_DISTRO}/share/gazebo_sim_mecanum/models:${GAZEBO_MODEL_PATH:-}" \
  gz sdf -k "/opt/ros/${ROS_DISTRO}/share/gazebo_sim_mecanum/models/xgc2_mecanum_ugv/model.sdf"
ldd "/opt/ros/${ROS_DISTRO}/lib/libgazebo_sim_mecanum_contract.so" | \
  awk '/not found/ {missing=1} END {exit missing ? 1 : 0}'

LIBGL_ALWAYS_SOFTWARE=1 timeout 60 rostest gazebo_sim_mecanum ideal_drive.test
echo "Installed package check passed"
