#!/usr/bin/env bash
# Start navigation with the edited magic2 map. Motion is opt-in.
set -e

nav_script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
nav_motion=false
nav_launch_options=()
for nav_option in "$@"; do
  case "$nav_option" in
    --motion) nav_motion=true ;;
    --show-args) nav_launch_options+=(--show-args) ;;
    -h|--help)
      echo "Usage: $0 [--motion] [--show-args]"
      echo "Default: load magic2.yaml + magic2_loop.pcd, open RViz, disable robot motion."
      echo "--motion: enable the robot's navigation motion bridge."
      echo "--show-args: list launch arguments without starting any nodes."
      exit 0 ;;
    *) echo "Unknown option: $nav_option (use --help)" >&2; exit 2 ;;
  esac
done

export NAV_WS="${NAV_WS:-${nav_script_dir}}"
source "${nav_script_dir}/setup_go2w_navigation.sh"
nav_map_yaml="${NAV_WS}/maps/magic2.yaml"
nav_map_pcd="${NAV_WS}/maps/magic2_loop.pcd"
for nav_map_file in "$nav_map_yaml" "$nav_map_pcd"; do
  if [[ ! -r "$nav_map_file" ]]; then
    echo "Map file not readable: $nav_map_file" >&2
    exit 1
  fi
done

exec ros2 launch go2w_nav2_navigation go2w_navigation.launch.py \
  "map:=${nav_map_yaml}" \
  "pcd_map:=${nav_map_pcd}" \
  "enable_motion:=${nav_motion}" \
  start_rviz:=true "${nav_launch_options[@]}"
