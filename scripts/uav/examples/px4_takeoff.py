"""Ví dụ điều khiển PX4 SITL qua MAVLink: arm → cất cánh → đọc độ cao.

Chạy PX4 trước (terminal khác hoặc nền):  scripts/uav/px4_sitl.sh [sihsim_quadx | gz_x500]
Rồi:  /opt/uav-venv/bin/python scripts/uav/examples/px4_takeoff.py
Mã thoát 0 nếu cất cánh tới MIS_TAKEOFF_ALT (mặc định 2.5 m).
"""
import sys
import time

from pymavlink import mavutil

m = mavutil.mavlink_connection("udpin:0.0.0.0:14540")
hb = m.wait_heartbeat(timeout=90)
if hb is None:
    sys.exit("Không nhận được heartbeat từ PX4 (udp 14540)")
print("heartbeat: type", hb.type, "autopilot", hb.autopilot)  # 2 = quadrotor, 12 = PX4


def gcs_heartbeat():
    # PX4 chỉ cho arm khi thấy trạm mặt đất (GCS) -> script đóng vai GCS
    m.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS, mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)


def command(cmd, *params):
    p = list(params) + [0] * (7 - len(params))
    m.mav.command_long_send(m.target_system, m.target_component, cmd, 0, *p)


# GLOBAL_POSITION_INT (id 33) ở 5 Hz
command(mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 33, 200_000)

# Arm: thử lại tới khi EKF hội tụ và preflight check đạt
t0 = time.time()
armed = False
while time.time() - t0 < 120 and not armed:
    gcs_heartbeat()
    command(mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 1)
    ack = m.recv_match(type="COMMAND_ACK", blocking=True, timeout=3)
    armed = bool(ack and ack.command == mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM and ack.result == 0)
    if not armed:
        time.sleep(2)
print(f"armed: {armed} sau {time.time() - t0:.0f} s")
if not armed:
    sys.exit(1)

# Takeoff: param 7 của PX4 là độ cao AMSL; NaN -> dùng MIS_TAKEOFF_ALT
nan = float("nan")
command(mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, nan, nan, nan, nan)

alt = 0.0
t1 = time.time()
while time.time() - t1 < 40:
    gcs_heartbeat()
    gp = m.recv_match(type="GLOBAL_POSITION_INT", blocking=True, timeout=5)
    if gp:
        alt = gp.relative_alt / 1000  # mm -> m
print(f"độ cao tương đối sau 40 s: {alt:.2f} m")
sys.exit(0 if alt > 2.0 else 1)
