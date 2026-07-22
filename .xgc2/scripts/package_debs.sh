#!/usr/bin/env bash
set -euo pipefail

INSTALL_ROOT=""
OUTPUT_DIR=""
ROS_DISTRO="${ROS_DISTRO:-noetic}"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"
PACKAGE="ros-noetic-xgc2-gazebo-sim-mecanum"
ROS_PACKAGE="gazebo_sim_mecanum"

product_version() {
  awk -F': *' '/^version:[[:space:]]*/ {print $2; exit}' "${REPO_ROOT}/.xgc2/product.yml"
}
VERSION="${PACKAGE_VERSION:-$(product_version)}"

while [[ $# -gt 0 ]]; do
  case "$1" in
    --install-root) INSTALL_ROOT="$2"; shift 2 ;;
    --output-dir) OUTPUT_DIR="$2"; shift 2 ;;
    *) echo "unknown argument: $1" >&2; exit 1 ;;
  esac
done
if [[ -z "${INSTALL_ROOT}" || -z "${OUTPUT_DIR}" ]]; then
  echo "--install-root and --output-dir are required" >&2
  exit 1
fi

ARCH="$(dpkg --print-architecture)"
PREFIX="/opt/ros/${ROS_DISTRO}"
PREFIX_ROOT="${INSTALL_ROOT}${PREFIX}"
PKG_ROOT="$(mktemp -d)"
cleanup() { rm -rf "${PKG_ROOT}"; }
trap cleanup EXIT

mkdir -p "${OUTPUT_DIR}" "${PKG_ROOT}/DEBIAN" "${PKG_ROOT}/usr/share/doc/${PACKAGE}"
rm -f "${OUTPUT_DIR}"/*.deb

copy_path() {
  local source="$1"
  if [[ -e "${source}" ]]; then
    mkdir -p "${PKG_ROOT}$(dirname "${source#${INSTALL_ROOT}}")"
    cp -a "${source}" "${PKG_ROOT}${source#${INSTALL_ROOT}}"
  fi
}
copy_path "${PREFIX_ROOT}/share/${ROS_PACKAGE}"
copy_path "${PREFIX_ROOT}/lib/libgazebo_sim_mecanum_contract.so"
copy_path "${PREFIX_ROOT}/lib/${ROS_PACKAGE}"
install -D -m 0644 "${REPO_ROOT}/process-definitions/xgc2-gazebo-sim-mecanum.json" \
  "${PKG_ROOT}/usr/share/xgc2/process-definitions/xgc2-gazebo-sim-mecanum.json"

cat >"${PKG_ROOT}/DEBIAN/control" <<EOF
Package: ${PACKAGE}
Version: ${VERSION}
Section: misc
Priority: optional
Architecture: ${ARCH}
Maintainer: XGC2 <apt@example.com>
Depends: libgazebo11, ros-noetic-gazebo-msgs, ros-noetic-gazebo-ros, ros-noetic-geometry-msgs, ros-noetic-roscpp, ros-noetic-roslaunch, ros-noetic-rospy, ros-noetic-rostest, ros-noetic-sensor-msgs, ros-noetic-tf2, ros-noetic-tf2-ros, ros-noetic-xacro, ros-noetic-xgc2-mecanum-description (>= 0.1.0-1), ros-noetic-xgc2-gazebo-sim-worlds (>= 1.1.0-14)
Recommends: ros-noetic-xgc2-gazebo-sim-vrpn-bridge (>= 1.1.0-13)
Description: XGC2 collision-free ideal Mecanum UGV for Gazebo Classic
EOF

install -m 0644 "${REPO_ROOT}/LICENSE" "${PKG_ROOT}/usr/share/doc/${PACKAGE}/copyright"
find "${PKG_ROOT}" -type d -exec chmod 0755 {} +
find "${PKG_ROOT}" -type f -exec chmod 0644 {} +
chmod 0755 "${PKG_ROOT}/DEBIAN"
chmod 0755 "${PKG_ROOT}${PREFIX}/lib/libgazebo_sim_mecanum_contract.so"
find "${PKG_ROOT}${PREFIX}/lib/${ROS_PACKAGE}" -type f -name '*.py' -exec chmod 0755 {} +
find "${PKG_ROOT}${PREFIX}/share/${ROS_PACKAGE}/test" -type f -name '*.py' -exec chmod 0755 {} +

fakeroot dpkg-deb --build "${PKG_ROOT}" "${OUTPUT_DIR}/${PACKAGE}_${VERSION}_${ARCH}.deb" >/dev/null
find "${OUTPUT_DIR}" -maxdepth 1 -type f -name '*.deb' -print | sort
