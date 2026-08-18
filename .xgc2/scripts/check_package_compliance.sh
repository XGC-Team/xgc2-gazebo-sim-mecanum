#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"
cd "${REPO_ROOT}"
export PYTHONPYCACHEPREFIX="${PYTHONPYCACHEPREFIX:-/tmp/xgc2-gazebo-sim-mecanum-pycache}"

bash -n .xgc2/scripts/*.sh
python3 -m py_compile scripts/check_model_ready.py scripts/model_lifecycle.py \
  test/ideal_drive_e2e.py test/high_fidelity_drive_e2e.py \
  .xgc2/scripts/xgc2_artifact_manifest.py

required=(
  .github/workflows/ci.yml
  .github/workflows/release.yml
  .xgc2/product.yml
  .xgc2/scripts/build_debs_in_docker.sh
  .xgc2/scripts/check_installed_packages.sh
  .xgc2/scripts/check_package_compliance.sh
  .xgc2/scripts/package_debs.sh
  .xgc2/scripts/xgc2_artifact_manifest.py
  CMakeLists.txt
  LICENSE
  MODEL_ASSET_NOTICE.md
  README.md
  package.xml
  launch/simple.launch
  launch/spawn.launch
  models/xgc2_mecanum_ugv/model.config
  models/xgc2_mecanum_ugv/model.sdf
  models/xgc2_mecanum_ugv/model.sdf.xacro
  scripts/check_model_ready.py
  scripts/model_lifecycle.py
  src/mecanum_contract_plugin.cpp
  test/ideal_drive.test
  test/ideal_drive_e2e.py
  test/high_fidelity_drive.test
  test/high_fidelity_drive_e2e.py
)
for path in "${required[@]}"; do
  test -f "${path}" || { echo "Missing ${path}" >&2; exit 1; }
done

grep -q '^id: xgc2-gazebo-sim-mecanum$' .xgc2/product.yml
grep -q '^version: 0.1.0-11$' .xgc2/product.yml
grep -q '^    focal: 0.1.0-11$' .xgc2/product.yml
grep -q '<name>gazebo_sim_mecanum</name>' package.xml
grep -q 'PACKAGE="ros-noetic-xgc2-gazebo-sim-mecanum"' .xgc2/scripts/package_debs.sh
grep -q 'ros-noetic-rostest' .xgc2/scripts/package_debs.sh
grep -q 'ros-noetic-xgc2-mecanum-description (>= 0.1.0-1)' .xgc2/scripts/package_debs.sh
grep -Eq -- '-Y \$\(arg yaw\).*' launch/spawn.launch
if grep -Eq '<arg name="bond"|(^|[[:space:]])-b([[:space:]]|")' launch/spawn.launch; then
  echo "Spawn lifecycle must have one supervisor-owned delete path" >&2
  exit 1
fi
grep -q 'type="model_lifecycle.py"' launch/spawn.launch
grep -q '$(dirname)/../models/xgc2_mecanum_ugv/model.sdf.xacro' launch/spawn.launch
if grep -q '$(find gazebo_sim_mecanum)' launch/spawn.launch; then
  echo "Spawn launch must resolve its owned model relative to its canonical file" >&2
  exit 1
fi

for xml in package.xml launch/*.launch models/xgc2_mecanum_ugv/model.config \
  models/xgc2_mecanum_ugv/model.sdf models/xgc2_mecanum_ugv/model.sdf.xacro test/*.test; do
  xmllint --noout "${xml}"
done
GAZEBO_MODEL_PATH="/opt/ros/noetic/share:${REPO_ROOT}/models:${GAZEBO_MODEL_PATH:-}" gz sdf -k models/xgc2_mecanum_ugv/model.sdf
expanded_sdf="$(mktemp)"
/opt/ros/noetic/bin/xacro models/xgc2_mecanum_ugv/model.sdf.xacro robot_namespace:=ugv_contract >"${expanded_sdf}"
grep -q '<robotNamespace>ugv_contract</robotNamespace>' "${expanded_sdf}"
grep -q '<driveModel>high_fidelity</driveModel>' "${expanded_sdf}"
grep -q '<gravity>True</gravity>' "${expanded_sdf}"
grep -q '<joint name="upper_left_wheel_joint"' "${expanded_sdf}"
GAZEBO_MODEL_PATH="/opt/ros/noetic/share:${REPO_ROOT}/models:${GAZEBO_MODEL_PATH:-}" gz sdf -k "${expanded_sdf}"
rm -f "${expanded_sdf}"

expanded_source_sdf="$(mktemp)"
source_plugin="/tmp/xgc2-mecanum-contract/libgazebo_sim_mecanum_contract.so"
source_meshes="file:///tmp/xgc2-mecanum-description/meshes"
/opt/ros/noetic/bin/xacro models/xgc2_mecanum_ugv/model.sdf.xacro \
  robot_namespace:=ugv_contract plugin_filename:="${source_plugin}" \
  mesh_prefix:="${source_meshes}" >"${expanded_source_sdf}"
grep -q "filename=\"${source_plugin}\"" "${expanded_source_sdf}"
grep -q "<uri>${source_meshes}/nexus_base_link.STL</uri>" "${expanded_source_sdf}"
rm -f "${expanded_source_sdf}"

expanded_ideal_sdf="$(mktemp)"
/opt/ros/noetic/bin/xacro models/xgc2_mecanum_ugv/model.sdf.xacro \
  robot_namespace:=ugv_contract drive_model:=ideal >"${expanded_ideal_sdf}"
grep -q '<driveModel>ideal</driveModel>' "${expanded_ideal_sdf}"
grep -q '<gravity>False</gravity>' "${expanded_ideal_sdf}"
if grep -Eq '<collision|<joint name=".*wheel_joint"' "${expanded_ideal_sdf}"; then
  echo "Ideal SDF must remain collision-free and wheel-joint-free" >&2
  exit 1
fi
GAZEBO_MODEL_PATH="/opt/ros/noetic/share:${REPO_ROOT}/models:${GAZEBO_MODEL_PATH:-}" gz sdf -k "${expanded_ideal_sdf}"
rm -f "${expanded_ideal_sdf}"

grep -q 'filename="libgazebo_sim_mecanum_contract.so"' models/xgc2_mecanum_ugv/model.sdf
grep -q 'model_->SetLinearVel' src/mecanum_contract_plugin.cpp
grep -q 'model_->SetAngularVel' src/mecanum_contract_plugin.cpp
grep -q 'wheel_joints_\[index\]->SetForce' src/mecanum_contract_plugin.cpp
grep -q 'body_link_->AddRelativeForce' src/mecanum_contract_plugin.cpp
if grep -Rq 'SetWorldPose' src; then
  echo "Plugin must let Gazebo integrate pose" >&2
  exit 1
fi
grep -q '<gravity>True</gravity>' models/xgc2_mecanum_ugv/model.sdf
grep -q '<collision name="chassis">' models/xgc2_mecanum_ugv/model.sdf
grep -q '<joint name="upper_left_wheel_joint"' models/xgc2_mecanum_ugv/model.sdf
if grep -Rq 'libnexus_ros_force_based_move.so' CMakeLists.txt package.xml launch models src test; then
  echo "Legacy force-based plugin reference found" >&2
  exit 1
fi

grep -q 'model://mecanum_description/meshes/nexus_base_link.STL' models/xgc2_mecanum_ugv/model.sdf
if find models -type f -path '*/meshes/*' | grep -q .; then
  echo "Gazebo simulation must consume mecanum_description instead of owning robot meshes" >&2
  exit 1
fi

echo "Package compliance checks passed"
