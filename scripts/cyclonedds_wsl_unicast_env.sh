#!/usr/bin/env bash
#
# CycloneDDS "unicast discovery" helper for WSL (mirrored networking) ↔ LAN robot.
#
# Why: WSL mirrored mode often drops/behaves oddly for UDP multicast, which DDS discovery relies on.
# This script configures CycloneDDS to:
#   - bind to the WSL source IP used to reach the robot
#   - disable multicast
#   - add the robot as a static discovery peer (unicast)
#
# Usage (recommended):
#   export ROBOT_IP=192.168.16.210
#   source ~/dev_ws/scripts/cyclonedds_wsl_unicast_env.sh
#
# Or:
#   source ~/dev_ws/scripts/cyclonedds_wsl_unicast_env.sh 192.168.16.210
#
# Keep this file source-safe for ~/.bashrc. Do not enable strict shell options
# here, because they would leak into the user's interactive shell.
_cyclonedds_wsl_done() {
  local rc="$1"
  return "${rc}" 2>/dev/null || exit "${rc}"
}

robot_ip="${1:-${ROBOT_IP:-}}"
if [[ -z "${robot_ip}" ]]; then
  echo "cyclonedds_wsl_unicast_env.sh: missing robot ip"
  echo "  set ROBOT_IP or pass it as the first argument"
  _cyclonedds_wsl_done 2
fi

src_ip="$(
  ip -4 route get "${robot_ip}" 2>/dev/null \
    | awk '{for(i=1;i<=NF;i++) if ($i=="src") {print $(i+1); exit}}'
)"

if [[ -z "${src_ip}" ]]; then
  echo "cyclonedds_wsl_unicast_env.sh: cannot determine source IP to reach ${robot_ip}"
  echo "  check that the robot IP is reachable from WSL and routing is set up"
  _cyclonedds_wsl_done 3
fi

# Configure CycloneDDS via an inline XML string (no external file dependency).
#
# Notes:
# - Discovery/Peers/Peer uses attribute name 'Address' (case-sensitive in CycloneDDS config parsing).
# - General/Interfaces/NetworkInterface attribute name is 'address' (lowercase).
export CYCLONEDDS_URI="<?xml version=\"1.0\" encoding=\"UTF-8\"?>
<CycloneDDS>
  <Domain Id=\"any\">
    <General>
      <Interfaces>
        <NetworkInterface address=\"${src_ip}\" multicast=\"false\" />
      </Interfaces>
      <AllowMulticast>false</AllowMulticast>
    </General>
    <Discovery>
      <Peers>
        <Peer Address=\"${robot_ip}\" />
      </Peers>
    </Discovery>
  </Domain>
</CycloneDDS>"

echo "CycloneDDS configured for unicast discovery:"
echo "  src_ip=${src_ip}"
echo "  peer=${robot_ip}"

unset -f _cyclonedds_wsl_done
