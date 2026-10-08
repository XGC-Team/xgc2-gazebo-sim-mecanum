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

test -f "/opt/ros/${ROS_DISTRO}/share/gazebo_sim_worlds/launch/native_world.launch"
dpkg -s ros-noetic-xgc2-gazebo-scene libxgc2-xrpc1 >/dev/null
for launch in simple spawn; do
  xmllint --noout "/opt/ros/${ROS_DISTRO}/share/gazebo_sim_mecanum/launch/${launch}.launch"
done
GAZEBO_MODEL_PATH="/opt/ros/${ROS_DISTRO}/share:/opt/ros/${ROS_DISTRO}/share/gazebo_sim_mecanum/models:${GAZEBO_MODEL_PATH:-}" \
  gz sdf -k "/opt/ros/${ROS_DISTRO}/share/gazebo_sim_mecanum/models/xgc2_mecanum_ugv/model.sdf"
ldd "/opt/ros/${ROS_DISTRO}/lib/libgazebo_sim_mecanum_contract.so" | \
  awk '/not found/ {missing=1} END {exit missing ? 1 : 0}'

# Native scientific fixtures are explicit process workflows, never installation side effects.
if [[ "${XGC2_RUN_NATIVE_VEHICLE_TESTS:-false}" == "true" ]]; then
  : "${SIMULATION_PREPARED_WORLD:?prepared isolated world required}"
  : "${SIMULATION_SERVICE_REF_JSON:?explicit native ServiceRef required}"
  : "${SIMULATION_TARGET_ID:?explicit native target required}"
  unset DISPLAY WAYLAND_DISPLAY
  for fixture in high_fidelity_drive ideal_drive; do
    LIBGL_ALWAYS_SOFTWARE=1 timeout 100 rostest gazebo_sim_mecanum "${fixture}.test" \
      "world_name:=${SIMULATION_PREPARED_WORLD}" \
      "simulation_service_ref_json:=${SIMULATION_SERVICE_REF_JSON}" "target_id:=${SIMULATION_TARGET_ID}"
  done
fi
echo "Installed package check passed"
