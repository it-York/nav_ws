#!/usr/bin/env bash

workspace="${NAV_WS:-$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)}"
export NAV_WS="${workspace}"
# Go2W is wired through USB Ethernet; eno1 is dedicated to the MID360s.
go2w_interface="${GO2W_INTERFACE:-enx00e04c356b10}"

if [[ ! -d "/sys/class/net/${go2w_interface}" ]]; then
  echo "Go2-W network interface not found: ${go2w_interface}" >&2
  return 1 2>/dev/null || exit 1
fi

source /opt/ros/humble/setup.bash
# Reuse the installed Unitree message package without sourcing the old SLAM
# workspace (which contains conflicting FAST-LIO/localization packages).
unitree_api_prefix="${UNITREE_API_PREFIX:-$(dirname "${workspace}")/koala_ws/install/unitree_api}"
if [[ -f "${unitree_api_prefix}/share/unitree_api/local_setup.bash" ]]; then
  source "${unitree_api_prefix}/share/unitree_api/local_setup.bash"
else
  echo "Unitree messages not found. Set UNITREE_API_PREFIX to the unitree_api install prefix." >&2
  return 1 2>/dev/null || exit 1
fi

sdk_prefix="${workspace}/third_party/livox_sdk2_install"
export CMAKE_INCLUDE_PATH="${sdk_prefix}/include${CMAKE_INCLUDE_PATH:+:${CMAKE_INCLUDE_PATH}}"
export CMAKE_LIBRARY_PATH="${sdk_prefix}/lib${CMAKE_LIBRARY_PATH:+:${CMAKE_LIBRARY_PATH}}"
export LD_LIBRARY_PATH="${sdk_prefix}/lib${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}"

if [[ -f "${workspace}/install/setup.bash" ]]; then
  source "${workspace}/install/setup.bash"
fi

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
export ROS_LOCALHOST_ONLY=0
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI="<CycloneDDS><Domain Id=\"${ROS_DOMAIN_ID}\"><General><Interfaces><NetworkInterface name=\"${go2w_interface}\" priority=\"default\" multicast=\"default\" /></Interfaces></General></Domain></CycloneDDS>"

echo "ROS 2 workspace: ${workspace}"
echo "Go2-W DDS interface: ${go2w_interface}"
echo "ROS_DOMAIN_ID: ${ROS_DOMAIN_ID}"
