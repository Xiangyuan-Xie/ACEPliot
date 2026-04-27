#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"
MAVLINK_ROUTER_DIR="${REPO_ROOT}/third_party/mavlink-router"
BUILD_DIR="${MAVLINK_ROUTER_DIR}/build"

echo "Installing mavlink-router build dependencies"
sudo apt update
sudo apt install -y git meson ninja-build pkg-config gcc g++ systemd

echo "Initializing mavlink-router submodules"
git -C "${REPO_ROOT}" submodule update --init --recursive third_party/mavlink-router

if [[ ! -f "${MAVLINK_ROUTER_DIR}/meson.build" ]]; then
  echo "mavlink-router source was not found at ${MAVLINK_ROUTER_DIR}." >&2
  exit 1
fi

echo "Configuring mavlink-router build"
if [[ -d "${BUILD_DIR}" ]]; then
  meson setup --reconfigure "${BUILD_DIR}" "${MAVLINK_ROUTER_DIR}"
else
  meson setup "${BUILD_DIR}" "${MAVLINK_ROUTER_DIR}"
fi

echo "Building mavlink-router"
ninja -C "${BUILD_DIR}"

echo "Installing mavlink-router"
sudo ninja -C "${BUILD_DIR}" install

echo "Checking mavlink-routerd"
which mavlink-routerd
