---
name: uav-sim-lab
description: Bộ công cụ mô phỏng UAV cài sẵn trên máy ảo cloud (venv /opt/uav-venv) — khí động học (AeroSandbox, NeuralFoil, XFOIL, OpenFOAM, SU2), động lực học bay và điều khiển (python-control, CasADi, MuJoCo, PyBullet/gym-pybullet-drones, ArduPilot SITL, MAVLink), kiểm bền kết cấu (CalculiX, PyNite, scikit-fem, sectionproperties, gmsh), hậu xử lý (ParaView, PyVista). Dùng skill này BẤT CỨ KHI NÀO cần tính/mô phỏng UAV, cánh, profil, lực nâng/cản, ổn định, PID/LQR/MPC, quỹ đạo, SITL, ứng suất/chuyển vị/dao động khung, chia lưới, hoặc khi cần cài thêm, kiểm tra, nâng cấp công cụ mô phỏng.
---

# UAV Sim Lab

Môi trường nằm ngoài repo (máy ảo bị xóa sau mỗi phiên) và được dựng lại bằng
`scripts/uav/setup.sh` (SessionStart hook tự gọi). Mọi lệnh Python dùng
`/opt/uav-venv/bin/python`.

## 0. Trước khi làm bất cứ việc gì
1. `source /etc/profile.d/uav-env.sh` (PATH, SU2, MUJOCO_GL=egl).
2. Chạy `/opt/uav-venv/bin/python scripts/uav/check_env.py`. Mục nào ❌ thì sửa (mục 4) trước khi dùng.
3. Máy ảo: 4 CPU, 15 GB RAM, **không GPU, không màn hình**. Không đề xuất Isaac Sim/Isaac Lab,
   Gazebo GUI, Fluent/ANSYS (bản quyền, Windows). Kết quả hình xuất ra PNG/MP4/VTK rồi gửi người dùng.

## 1. Chọn công cụ theo bài toán (từ nhanh → chính xác)

| Bài toán | Công cụ | Ghi chú |
|---|---|---|
| Khí quyển chuẩn | `ambiance` | ISA tới 81 km |
| Profil cánh 2D | `neuralfoil` → `xfoil` | NeuralFoil nhanh, ổn định; XFOIL để đối chiếu, chạy qua stdin |
| Cánh/máy bay 3D thế (inviscid) | `aerosandbox`: AeroBuildup, LiftingLine, VortexLatticeMethod | Đối chiếu Helmbold/Prandtl |
| Tối ưu thiết kế (sizing, MDO) | `aerosandbox.Opti` (CasADi), `openmdao` | |
| CFD nhớt, không nén | OpenFOAM (`simpleFoam`, `pimpleFoam`) | gõ `of` để nạp môi trường; lưới `blockMesh`/`snappyHexMesh`/gmsh |
| CFD nén được, adjoint | SU2 (`SU2_CFD`, `SU2_CFD_AD`) | |
| Động lực học bay 6DOF, điều khiển | `numpy`/`scipy.integrate`, `control`, `casadi` (MPC), `filterpy` (EKF) | |
| Mô phỏng vật lý đa vật | `mujoco` (chính xác, nhanh), `pybullet` | |
| Quadrotor + RL | `gym_pybullet_drones` + `stable_baselines3` (torch CPU) | |
| Autopilot thật (SITL) | ArduPilot `/opt/ardupilot` (`sim_vehicle.py -v ArduCopter --no-mavproxy` hoặc `build/sitl/bin/arducopter`), điều khiển bằng `pymavlink`/`mavsdk` | PX4 chỉ có khi cài `--with-px4` |
| Log bay | `pyulog` (PX4 .ulg), `pymavlink.mavutil` (ArduPilot .bin/.tlog) | |
| Khung/dầm 3D | `Pynite` (FEModel3D) | |
| Đặc trưng mặt cắt | `sectionproperties` | |
| FEM khối/vỏ, dao động riêng, buckling | CalculiX `ccx` (input kiểu Abaqus), `scikit-fem` | lưới bằng `gmsh` → `meshio` |
| Hậu xử lý | `pyvista` (off-screen), `pvbatch` (script ParaView headless), `matplotlib` | |

## 1b. Bẫy đã gặp
- `ambiance.Atmosphere(h)` dùng độ cao **hình học**; bảng ISA dùng độ cao **địa thế** H
  (h = r0·H/(r0−H), r0 = 6 356 766 m). Ở 11 km sai khác 0.12 K.
- Mạng máy ảo chặn `download.pytorch.org` và `api.github.com`: torch lấy từ PyPI (bản CUDA, vẫn chạy CPU),
  phiên bản GitHub lấy bằng `git ls-remote --tags`.
- MuJoCo dùng Euler bán ẩn: rơi tự do n bước cho z = −g·dt²·n(n+1)/2 (−4.910 m sau 1 s, dt = 1 ms),
  không phải −4.905 m — đây là sai số tích phân, giảm dt để hội tụ.
- OpenFOAM từ apt là ESI v1912 (cũ); cú pháp tutorial của bản mới có thể khác.
- SITL: muốn nhận vị trí/attitude phải gửi `REQUEST_DATA_STREAM`/`SET_MESSAGE_INTERVAL` trước.

## 2. Quy trình bắt buộc khi mô phỏng
1. Ghi dữ kiện, đơn vị (SI: mm-N-MPa cho kết cấu, m-kg-s cho khí động/bay), giả thiết.
2. Chạy **case kiểm chứng** có lời giải giải tích trước (xem `check_env.py`: Helmbold, rơi tự do,
   dầm công-xôn, F/A) — cùng công cụ, cùng thiết lập.
3. FEM/CFD: **hội tụ lưới** tối thiểu 3 mức lưới, sai khác đại lượng quan tâm < 2–5 %.
   CFD: kiểm residual, y+ phù hợp mô hình rối, cân bằng lưu lượng.
4. Đối chiếu chéo khi có thể (NeuralFoil ↔ XFOIL, VLM ↔ LiftingLine, PyNite ↔ CalculiX).
5. Kiểm tra vật lý: dấu, bậc độ lớn, trường hợp giới hạn. Báo rõ điểm chưa chắc.
6. Code/case lưu trong repo (vd. `sims/<tên>/`), kết quả nặng (lưới, VTK) không commit.

## 3. Mẫu nhanh
```bash
of                                   # nạp OpenFOAM
cp -r $FOAM_TUTORIALS/incompressible/simpleFoam/airFoil2D /tmp/af && cd /tmp/af && ./Allrun
```
```python
# SITL + pymavlink: khởi động arducopter rồi kết nối tcp:127.0.0.1:5760
import subprocess, time
from pymavlink import mavutil
p = subprocess.Popen(["/opt/ardupilot/build/sitl/bin/arducopter", "--model", "quad",
                      "--defaults", "/opt/ardupilot/Tools/autotest/default_params/copter.parm",
                      "-I0"], cwd="/tmp")
m = mavutil.mavlink_connection("tcp:127.0.0.1:5760"); m.wait_heartbeat(timeout=60)
```

## 4. Cài thêm / sửa / nâng cấp (tự làm, không chờ nhắc)
- **Thiếu gói Python**: thêm vào `scripts/uav/requirements.txt` → chạy `scripts/uav/setup.sh`
  → `check_env.py`. Gói hệ thống: thêm vào `APT_PKGS` trong `setup.sh`. Công cụ build từ nguồn: thêm
  khối mới trong `setup.sh` theo mẫu SU2/ArduPilot (idempotent, kiểm tra file đích trước khi cài).
- **Nâng cấp định kỳ**: `scripts/uav/setup.sh --upgrade` rồi `check_env.py`; nếu mục nào hỏng
  sau nâng cấp thì ghim phiên bản (`pkg==x.y`) trong requirements và ghi lý do bằng comment.
- **Thêm phép thử** cho mọi công cụ mới vào `check_env.py` (có đáp án giải tích).
- Sau khi sửa: commit + push để phiên sau dùng được; cập nhật bảng ở mục 1 của file này.
- PX4: `scripts/uav/setup.sh --with-px4` (~8 GB, ~30 phút).
