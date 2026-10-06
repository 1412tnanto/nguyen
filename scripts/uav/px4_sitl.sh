#!/bin/bash
# Chạy PX4 SITL không màn hình (headless).
#   scripts/uav/px4_sitl.sh                 # sihsim_quadx: mô phỏng tích hợp trong PX4, nhẹ nhất
#   scripts/uav/px4_sitl.sh sihsim_airplane # các mẫu SIH khác: sihsim_xvert, sihsim_standard_vtol
#   scripts/uav/px4_sitl.sh gz_x500         # Gazebo Harmonic (server headless), các mẫu gz_* khác
# MAVLink: udp://:14540 (offboard API, MAVSDK/pymavlink), udp://:14550 (GCS).
# Dừng: Ctrl+C hoặc kill tiến trình.
set -euo pipefail
MODEL=${1:-sihsim_quadx}
PX4_DIR=${PX4_DIR:-/opt/PX4-Autopilot}
B="$PX4_DIR/build/px4_sitl_default"
[ -x "$B/bin/px4" ] || { echo "Chưa build PX4: chạy scripts/uav/setup.sh" >&2; exit 1; }
WD=$(mktemp -d /tmp/px4-XXXXXX)   # thư mục làm việc riêng (log, tham số)
export PX4_SIM_MODEL="$MODEL"
export HEADLESS=1                 # Gazebo chạy server, không mở GUI
cd "$B"
exec "$B/bin/px4" -w "$WD" "$B/etc"
