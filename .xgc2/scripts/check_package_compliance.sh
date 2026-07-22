#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"
cd "${REPO_ROOT}"
export PYTHONPYCACHEPREFIX="${PYTHONPYCACHEPREFIX:-/tmp/xgc2-gazebo-sim-mecanum-pycache}"

bash -n .xgc2/scripts/*.sh
python3 -m py_compile scripts/check_model_ready.py test/ideal_drive_e2e.py \
  .xgc2/scripts/xgc2_artifact_manifest.py
python3 -m json.tool process-definitions/xgc2-gazebo-sim-mecanum.json >/dev/null

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
  process-definitions/xgc2-gazebo-sim-mecanum.json
  scripts/check_model_ready.py
  src/mecanum_contract_plugin.cpp
  test/ideal_drive.test
  test/ideal_drive_e2e.py
)
for path in "${required[@]}"; do
  test -f "${path}" || { echo "Missing ${path}" >&2; exit 1; }
done

grep -q '^id: xgc2-gazebo-sim-mecanum$' .xgc2/product.yml
grep -q '^version: 0.1.0-2$' .xgc2/product.yml
grep -q '^    focal: 0.1.0-2$' .xgc2/product.yml
grep -q '<name>gazebo_sim_mecanum</name>' package.xml
grep -q 'PACKAGE="ros-noetic-xgc2-gazebo-sim-mecanum"' .xgc2/scripts/package_debs.sh
grep -q 'ros-noetic-rostest' .xgc2/scripts/package_debs.sh
grep -q 'ros-noetic-xgc2-mecanum-description (>= 0.1.0-1)' .xgc2/scripts/package_debs.sh
grep -Eq -- '-Y \$\(arg yaw\).*' launch/spawn.launch
grep -q -- '-b' launch/spawn.launch

for xml in package.xml launch/*.launch models/xgc2_mecanum_ugv/model.config \
  models/xgc2_mecanum_ugv/model.sdf models/xgc2_mecanum_ugv/model.sdf.xacro test/*.test; do
  xmllint --noout "${xml}"
done
GAZEBO_MODEL_PATH="/opt/ros/noetic/share:${REPO_ROOT}/models:${GAZEBO_MODEL_PATH:-}" gz sdf -k models/xgc2_mecanum_ugv/model.sdf
expanded_sdf="$(mktemp)"
/opt/ros/noetic/bin/xacro models/xgc2_mecanum_ugv/model.sdf.xacro robot_namespace:=ugv_contract >"${expanded_sdf}"
grep -q '<robotNamespace>ugv_contract</robotNamespace>' "${expanded_sdf}"
GAZEBO_MODEL_PATH="/opt/ros/noetic/share:${REPO_ROOT}/models:${GAZEBO_MODEL_PATH:-}" gz sdf -k "${expanded_sdf}"
rm -f "${expanded_sdf}"

grep -q 'filename="libgazebo_sim_mecanum_contract.so"' models/xgc2_mecanum_ugv/model.sdf
grep -q 'model_->SetLinearVel' src/mecanum_contract_plugin.cpp
grep -q 'model_->SetAngularVel' src/mecanum_contract_plugin.cpp
if grep -Rq 'SetWorldPose' src; then
  echo "Plugin must let Gazebo integrate pose" >&2
  exit 1
fi
grep -q '<gravity>false</gravity>' models/xgc2_mecanum_ugv/model.sdf
if grep -Eq '<collision|force_based|gazebo_ros_control|wheel.*(force|torque)' models/xgc2_mecanum_ugv/model.sdf; then
  echo "SDF contains forbidden collision or low-level dynamics" >&2
  exit 1
fi
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
