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
ln -s "$B/rootfs/gz_env.sh" "$WD/gz_env.sh"   # đường dẫn world/model/plugin Gazebo của PX4
export PX4_SIM_MODEL="$MODEL"
export HEADLESS=1                 # Gazebo chạy server, không mở GUI
# px4 liên kết với gz-transport của conda nhưng không có RPATH tới đó
GZ_ENV=${GZ_ENV:-/opt/gz}
export LD_LIBRARY_PATH="$GZ_ENV/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
export PATH="/usr/local/bin:$PATH"   # lệnh gz (wrapper chạy trong môi trường conda)
cd "$B"
exec "$B/bin/px4" -d -w "$WD" "$B/etc"   # -d: không mở shell tương tác pxh>
