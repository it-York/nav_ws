#!/usr/bin/env bash
# Physical wiring: USB Ethernet -> Go2W; onboard eno1 -> MID360s.
set -euo pipefail
robot_device=enx00e04c356b10
lidar_device=eno1

if [[ ${EUID} -ne 0 ]]; then
  echo "Run with sudo after exiting navigation." >&2
  exit 1
fi
if pgrep -f '/go2w_nav2_bridge/(cmd_vel_to_sport|go2w_mode_manager)( |$)' >/dev/null; then
  echo "Navigation motion nodes are still running. Exit navigation with Ctrl+C first." >&2
  exit 1
fi
for device in "$robot_device" "$lidar_device"; do
  [[ -d "/sys/class/net/$device" ]] || { echo "Missing interface: $device" >&2; exit 1; }
done
robot_uuid="$(nmcli -g GENERAL.CON-UUID device show "$robot_device")"
lidar_uuid="$(nmcli -g GENERAL.CON-UUID device show "$lidar_device")"
for uuid in "$robot_uuid" "$lidar_uuid"; do
  [[ -n "$uuid" && "$uuid" != -- ]] || { echo "Both wired profiles must be active." >&2; exit 1; }
done
[[ "$robot_uuid" != "$lidar_uuid" ]] || { echo "Profiles must be distinct." >&2; exit 1; }

backup_dir="$(mktemp -d /tmp/nav_ws_network_backup.XXXXXX)"
nmcli connection show uuid "$robot_uuid" > "$backup_dir/robot_profile.txt"
nmcli connection show uuid "$lidar_uuid" > "$backup_dir/lidar_profile.txt"
ip -4 address show > "$backup_dir/addresses.txt"
ip -4 route show > "$backup_dir/routes.txt"
echo "Previous network settings saved to $backup_dir"

# Remove the robot subnet from the radar interface before assigning it to USB.
nmcli connection modify uuid "$lidar_uuid" \
  ipv4.method manual ipv4.addresses '192.168.1.6/24' \
  ipv4.gateway '' ipv4.dns '' ipv4.never-default yes \
  ipv4.routes '192.168.1.195/32' connection.autoconnect yes
nmcli device reapply "$lidar_device"
nmcli connection modify uuid "$robot_uuid" \
  ipv4.method manual ipv4.addresses '192.168.123.99/24' \
  ipv4.gateway '' ipv4.dns '' ipv4.never-default yes \
  ipv4.routes '' connection.autoconnect yes
nmcli device reapply "$robot_device"

ip -br -4 address show dev "$robot_device"
ip -br -4 address show dev "$lidar_device"
ip route get 192.168.123.161
ip route get 192.168.1.195
echo "Network profiles saved. Checking robot and radar reachability..."
result=0
ping -I "$robot_device" -c 3 -W 1 192.168.123.161 || result=1
ping -I "$lidar_device" -c 3 -W 1 192.168.1.195 || result=1
if [[ $result -ne 0 ]]; then
  echo "Settings applied, but a ping failed. Check cabling/device IP before enabling motion." >&2
  exit 2
fi
echo "Robot and radar ping checks passed. Restart navigation in inspection mode first."
