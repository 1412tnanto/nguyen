"""MCP server 'uav-sim': gói các công cụ mô phỏng trên máy ảo thành tool cho Claude.

Khai báo trong .mcp.json ở gốc repo; chạy bằng /opt/uav-venv/bin/python (stdio).
Mỗi tool chỉ gọi lại lệnh đã có (of2412, run_aster, fenicsx-python, px4_sitl.sh, check_env.py).
"""
import os
import re
import subprocess
import sys
import time

from mcp.server.mcpserver import MCPServer

HERE = os.path.dirname(os.path.abspath(__file__))
PY = sys.executable
ARDUCOPTER = "/opt/ardupilot/build/sitl/bin/arducopter"
ENV = {**os.environ, "PATH": f"/usr/local/bin:/opt/uav-venv/bin:/opt/su2/bin:{os.environ.get('PATH', '')}",
       "ACADOS_SOURCE_DIR": "/opt/acados",
       "LD_LIBRARY_PATH": f"/opt/acados/lib:{os.environ.get('LD_LIBRARY_PATH', '')}",
       "MAMBA_ROOT_PREFIX": "/opt/micromamba"}

app = MCPServer("uav-sim", instructions=(
    "Công cụ mô phỏng UAV trên máy ảo: OpenFOAM v2412, Code_Aster, FEniCSx, PX4/ArduPilot SITL. "
    "Đơn vị SI. Trước khi dùng kết quả, chạy case kiểm chứng và kiểm tra hội tụ lưới."))
_sitl: dict[str, subprocess.Popen] = {}   # tiến trình SITL đang chạy, theo tên autopilot


def _run(cmd: list[str], cwd: str | None = None, timeout: int = 3600, tail: int = 60) -> str:
    try:
        r = subprocess.run(cmd, cwd=cwd, env=ENV, capture_output=True, text=True, timeout=timeout)
    except subprocess.TimeoutExpired:
        return f"QUÁ THỜI GIAN ({timeout} s): {' '.join(cmd)}"
    out = (r.stdout + r.stderr).strip().splitlines()
    return f"exit={r.returncode}\n" + "\n".join(out[-tail:])


def _case_dir(path: str) -> str:
    path = os.path.abspath(os.path.expanduser(path))
    if not os.path.isdir(path):
        raise ValueError(f"không có thư mục: {path}")
    return path


@app.tool()
def check_env() -> str:
    """Kiểm tra toàn bộ môi trường mô phỏng (~4 phút): mỗi công cụ chạy một phép thử có đáp án giải tích."""
    return _run([PY, os.path.join(HERE, "check_env.py")], timeout=1800, tail=80)


@app.tool()
def openfoam(case_dir: str, commands: list[str], version: str = "2412") -> str:
    """Chạy lần lượt các lệnh OpenFOAM trong thư mục case, vd ["blockMesh", "checkMesh", "simpleFoam"].

    version: "2412" (mặc định, conda) hoặc "1912" (apt, chỉ cho case cũ).
    Mỗi lệnh ghi log vào log.<lệnh> trong case; dừng ở lệnh đầu tiên lỗi.
    """
    case = _case_dir(case_dir)
    report = []
    for c in commands:
        argv = c.split()
        if version == "2412":
            cmd = ["of2412", *argv]
        else:
            rc = "source /usr/share/openfoam/etc/bashrc >/dev/null 2>&1; exec \"$@\""
            cmd = ["bash", "-c", rc, "_", *argv]
        res = _run(cmd, cwd=case, tail=15)
        open(os.path.join(case, f"log.{argv[0]}"), "w").write(res)
        report.append(f"## {c}\n{res}")
        if not res.startswith("exit=0"):
            break
    return "\n\n".join(report)


@app.tool()
def code_aster(script: str) -> str:
    """Chạy Code_Aster với file lệnh Python (.py/.comm).

    File mở đầu bằng `from code_aster import CA; CA.init()` và kết thúc bằng `FIN()`.
    In kết quả dạng `TÊN=giá trị` (vd print("DY_TIP=%g" % dy)) để tool trích ra.
    Log đầy đủ ghi vào <script>.log (Aster in >400 dòng, kết quả nằm giữa log).
    """
    path = os.path.abspath(os.path.expanduser(script))
    full = _run(["run_aster", path], cwd=os.path.dirname(path), tail=10**6)
    open(path + ".log", "w").write(full)
    lines = full.splitlines()
    keep = [ln for ln in lines[1:-20] if re.match(r"[A-Za-z_][\w.]*=\S", ln) or re.search(r"<[FEA]>", ln)]
    return "\n".join([lines[0], *keep, "...", *lines[-20:], f"(log đầy đủ: {path}.log)"])


@app.tool()
def fenicsx(script: str) -> str:
    """Chạy script Python FEniCSx 0.11 (dolfinx, ufl, mpi4py) trong env riêng /opt/fenicsx."""
    path = os.path.abspath(os.path.expanduser(script))
    return _run(["fenicsx-python", path], cwd=os.path.dirname(path), tail=80)


@app.tool()
def sitl_start(autopilot: str = "px4", model: str = "sihsim_quadx") -> str:
    """Bật SITL chạy nền (headless).

    autopilot="px4": model sihsim_quadx | sihsim_airplane | gz_x500 | gz_standard_vtol ...; MAVLink udp:14540.
    autopilot="ardupilot": model quad | plane ...; MAVLink tcp:127.0.0.1:5760.
    """
    if autopilot in _sitl and _sitl[autopilot].poll() is None:
        return f"{autopilot} đang chạy (pid {_sitl[autopilot].pid}); gọi sitl_stop trước."
    if autopilot == "px4":
        cmd = [os.path.join(HERE, "px4_sitl.sh"), model]
    elif autopilot == "ardupilot":
        defaults = "copter.parm" if model == "quad" else f"{model}.parm"
        cmd = [ARDUCOPTER if model == "quad" else ARDUCOPTER.replace("arducopter", "arduplane"),
               "--model", model, "-I0",
               "--defaults", f"/opt/ardupilot/Tools/autotest/default_params/{defaults}"]
    else:
        raise ValueError("autopilot phải là 'px4' hoặc 'ardupilot'")
    log = open(f"/tmp/sitl_{autopilot}.log", "w")
    _sitl[autopilot] = subprocess.Popen(cmd, cwd="/tmp", env=ENV, stdout=log, stderr=subprocess.STDOUT,
                                        start_new_session=True)
    time.sleep(5)
    if _sitl[autopilot].poll() is not None:
        return f"SITL thoát sớm (exit {_sitl[autopilot].returncode}), xem /tmp/sitl_{autopilot}.log"
    return f"{autopilot} {model} đang chạy (pid {_sitl[autopilot].pid}), log /tmp/sitl_{autopilot}.log"


@app.tool()
def sitl_stop() -> str:
    """Dừng mọi SITL do server này bật (kể cả Gazebo con)."""
    stopped = []
    for name, p in list(_sitl.items()):
        if p.poll() is None:
            os.killpg(p.pid, 15)
            p.wait(timeout=20)
            stopped.append(name)
        del _sitl[name]
    subprocess.run(["pkill", "-x", "ruby"])   # server gz (Gazebo) là tiến trình ruby
    return "đã dừng: " + (", ".join(stopped) or "không có gì")


@app.tool()
def px4_takeoff() -> str:
    """Arm + cất cánh PX4 tới MIS_TAKEOFF_ALT (2.5 m) qua MAVLink, trả về độ cao đạt được.

    Cần gọi sitl_start('px4') trước.
    """
    return _run([PY, os.path.join(HERE, "examples", "px4_takeoff.py")], timeout=300)


@app.tool()
def mavlink_status(autopilot: str = "px4", seconds: float = 3.0) -> dict:
    """Đọc nhanh trạng thái qua MAVLink: heartbeat, vị trí, độ cao, tư thế, pin."""
    from pymavlink import mavutil
    url = "udpin:0.0.0.0:14540" if autopilot == "px4" else "tcp:127.0.0.1:5760"
    m = mavutil.mavlink_connection(url)
    hb = m.wait_heartbeat(timeout=15)
    if hb is None:
        return {"lỗi": f"không có heartbeat trên {url}"}
    out = {"type": hb.type, "autopilot": hb.autopilot,
           "armed": bool(hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)}
    t_end = time.time() + seconds
    while time.time() < t_end:
        msg = m.recv_match(type=["GLOBAL_POSITION_INT", "ATTITUDE", "SYS_STATUS"], blocking=True, timeout=1)
        if msg is None:
            continue
        t = msg.get_type()
        if t == "GLOBAL_POSITION_INT":
            out.update(lat=msg.lat / 1e7, lon=msg.lon / 1e7, rel_alt_m=msg.relative_alt / 1000)
        elif t == "ATTITUDE":
            out.update(roll_deg=round(msg.roll * 57.2958, 2), pitch_deg=round(msg.pitch * 57.2958, 2),
                       yaw_deg=round(msg.yaw * 57.2958, 2))
        elif t == "SYS_STATUS":
            out.update(battery_V=msg.voltage_battery / 1000)
    m.close()
    return out


if __name__ == "__main__":
    app.run()
