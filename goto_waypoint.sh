#!/usr/bin/env bash
set -e
waypoint_workspace="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source "${waypoint_workspace}/setup_go2w_navigation.sh"
exec python3 "${waypoint_workspace}/scripts/goto_waypoint.py" "$@"
