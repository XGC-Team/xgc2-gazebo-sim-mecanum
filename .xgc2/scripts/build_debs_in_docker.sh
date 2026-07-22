#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"
DOCKER_IMAGE="${DOCKER_IMAGE:-ros:noetic-ros-base-focal}"
WORK_DIR="${WORK_DIR:-${REPO_ROOT}/.work/docker}"
OUTPUT_DIR="${OUTPUT_DIR:-${REPO_ROOT}/debs}"
INSTALL_CHECK="${INSTALL_CHECK:-true}"

while [[ $# -gt 0 ]]; do
  case "$1" in
    --image) DOCKER_IMAGE="$2"; shift 2 ;;
    --work-dir) WORK_DIR="$2"; shift 2 ;;
    --output-dir) OUTPUT_DIR="$2"; shift 2 ;;
    --skip-install-check) INSTALL_CHECK=false; shift ;;
    *) echo "unknown argument: $1" >&2; exit 1 ;;
  esac
done

mkdir -p "${WORK_DIR}" "${OUTPUT_DIR}"
docker pull "${DOCKER_IMAGE}"
docker run --rm \
  -e DEBIAN_FRONTEND=noninteractive \
  -e INSTALL_CHECK="${INSTALL_CHECK}" \
  -e XGC2_APT_OVERLAY_URL="${XGC2_APT_OVERLAY_URL:-}" \
  -v "${REPO_ROOT}:/workspace/repo:ro" \
  -v "${WORK_DIR}:/workspace/work" \
  -v "${OUTPUT_DIR}:/workspace/out" \
  "${DOCKER_IMAGE}" bash -lc '
    set -euo pipefail
    sed -i \
      -e "s#http://archive.ubuntu.com/ubuntu#https://archive.ubuntu.com/ubuntu#g" \
      -e "s#http://security.ubuntu.com/ubuntu#https://archive.ubuntu.com/ubuntu#g" \
      -e "s#http://ports.ubuntu.com/ubuntu-ports#https://ports.ubuntu.com/ubuntu-ports#g" \
      /etc/apt/sources.list
    printf "%s\n" "Acquire::Retries \"5\";" \
      >/etc/apt/apt.conf.d/99-xgc2-retries
    apt_update() {
      local attempt
      for attempt in 1 2 3; do
        if apt-get update; then
          return 0
        fi
        [[ "${attempt}" -lt 3 ]] || return 1
        sleep "$((attempt * 5))"
      done
    }
    apt_install() {
      local attempt
      for attempt in 1 2 3; do
        if apt-get install "$@"; then
          return 0
        fi
        [[ "${attempt}" -lt 3 ]] || return 1
        sleep "$((attempt * 5))"
        apt_update
      done
    }
    apt_update
    apt_install -y --no-install-recommends ca-certificates
    echo "deb [trusted=yes arch=$(dpkg --print-architecture)] https://xgc2.apt.xiaokang.ink focal main" \
      >/etc/apt/sources.list.d/xgc2.list
    if [[ -n "${XGC2_APT_OVERLAY_URL:-}" ]]; then
      sed "s#https://xgc2.apt.xiaokang.ink#${XGC2_APT_OVERLAY_URL%/}#g" \
        /etc/apt/sources.list.d/xgc2.list >/etc/apt/sources.list.d/00-xgc2-release-train.list
    fi
    apt_update
    apt_install -y --no-install-recommends \
      build-essential cmake dpkg-dev fakeroot file git libxml2-utils rsync \
      gazebo11 libgazebo11-dev \
      ros-noetic-gazebo-msgs ros-noetic-gazebo-ros \
      ros-noetic-geometry-msgs ros-noetic-roscpp ros-noetic-roslaunch \
      ros-noetic-rospack ros-noetic-rospy ros-noetic-rostest \
      ros-noetic-sensor-msgs ros-noetic-tf ros-noetic-tf2 ros-noetic-tf2-ros \
      ros-noetic-xacro \
      ros-noetic-xgc2-mecanum-description \
      ros-noetic-xgc2-gazebo-sim-worlds

    rm -rf /workspace/work/src /workspace/work/build /workspace/work/devel /workspace/work/install-root
    mkdir -p /workspace/work/src/xgc2-gazebo-sim-mecanum
    rsync -a --delete /workspace/repo/ /workspace/work/src/xgc2-gazebo-sim-mecanum/
    cd /workspace/work
    source /opt/ros/noetic/setup.bash
    catkin_make -DCMAKE_BUILD_TYPE=Release
    LIBGL_ALWAYS_SOFTWARE=1 catkin_make run_tests_gazebo_sim_mecanum
    catkin_test_results
    DESTDIR=/workspace/work/install-root catkin_make install \
      -DCMAKE_INSTALL_PREFIX=/opt/ros/noetic -DCATKIN_ENABLE_TESTING=OFF
    /workspace/repo/.xgc2/scripts/package_debs.sh \
      --install-root /workspace/work/install-root --output-dir /workspace/out
    if [[ "${INSTALL_CHECK}" == true ]]; then
      apt_install -y /workspace/out/ros-noetic-xgc2-gazebo-sim-mecanum_*.deb
      /workspace/repo/.xgc2/scripts/check_installed_packages.sh
    fi
  '

find "${OUTPUT_DIR}" -maxdepth 1 -type f -name '*.deb' -print | sort
