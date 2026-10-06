#!/bin/bash
# Cài môi trường mô phỏng UAV (khí động, động lực học bay, kiểm bền, SITL).
# Idempotent: chạy lại chỉ cài phần còn thiếu.
#   scripts/uav/setup.sh              cài phần thiếu
#   scripts/uav/setup.sh --upgrade    nâng cấp gói Python, SU2, ArduPilot
#   scripts/uav/setup.sh --with-px4   cài thêm PX4 SITL (nặng, ~8 GB)
#   scripts/uav/setup.sh --skip-sitl  bỏ qua ArduPilot (cài nhanh)
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
VENV=/opt/uav-venv
SU2_DIR=/opt/su2
ARDUPILOT_DIR=/opt/ardupilot
PX4_DIR=/opt/PX4-Autopilot
UPGRADE=0; WITH_PX4=0; SKIP_SITL=0
for a in "$@"; do
  case "$a" in
    --upgrade) UPGRADE=1 ;;
    --with-px4) WITH_PX4=1 ;;
    --skip-sitl) SKIP_SITL=1 ;;
    *) echo "Tham số không hợp lệ: $a" >&2; exit 2 ;;
  esac
done

log() { echo "[uav-setup] $*" >&2; }
export DEBIAN_FRONTEND=noninteractive

# ---------- 1. Gói hệ thống ----------
APT_PKGS=(
  build-essential gfortran git cmake ninja-build pkg-config curl unzip rsync
  python3.12-dev python3.12-venv
  openfoam            # CFD thể tích hữu hạn (ESI OpenFOAM)
  calculix-ccx        # FEM kết cấu (tương thích cú pháp Abaqus)
  gmsh                # chia lưới
  xfoil               # phân tích profil cánh
  paraview            # hậu xử lý (pvpython chạy headless)
  xvfb libosmesa6 libgl1 libglu1-mesa libegl1 ffmpeg   # render không màn hình
  libxml2-dev libxslt1-dev
)
missing=()
for p in "${APT_PKGS[@]}"; do dpkg -s "$p" >/dev/null 2>&1 || missing+=("$p"); done
if [ ${#missing[@]} -gt 0 ]; then
  log "apt cài: ${missing[*]}"
  apt-get update -qq
  apt-get install -y -qq --no-install-recommends "${missing[@]}" >/dev/null
fi

# ---------- 2. Python venv ----------
if ! command -v uv >/dev/null; then pip3 install -q uv; fi
if [ ! -x "$VENV/bin/python" ]; then
  log "tạo venv $VENV (Python 3.12)"
  uv venv -q --python /usr/bin/python3.12 "$VENV"
fi
UVPIP=(uv pip install -q --python "$VENV/bin/python")
[ $UPGRADE -eq 1 ] && UVPIP+=(--upgrade)
if ! "$VENV/bin/python" -c "import torch" 2>/dev/null || [ $UPGRADE -eq 1 ]; then
  log "cài torch (ưu tiên bản CPU; nếu mạng chặn download.pytorch.org thì lấy từ PyPI)"
  "${UVPIP[@]}" torch --index-url https://download.pytorch.org/whl/cpu 2>/dev/null \
    || "${UVPIP[@]}" torch
fi
log "cài gói Python từ requirements.txt"
"${UVPIP[@]}" -r "$HERE/requirements.txt"
if ! "$VENV/bin/python" -c "import gym_pybullet_drones" 2>/dev/null || [ $UPGRADE -eq 1 ]; then
  log "cài gym-pybullet-drones"
  "${UVPIP[@]}" "gym-pybullet-drones @ git+https://github.com/utiasDSL/gym-pybullet-drones.git"
fi

# ---------- 3. SU2 (CFD nén được, adjoint) ----------
if [ ! -x "$SU2_DIR/bin/SU2_CFD" ] || [ $UPGRADE -eq 1 ]; then
  # api.github.com có thể bị chặn; lấy tag mới nhất bằng git
  tag=$(git ls-remote --tags --refs https://github.com/su2code/SU2.git 'v*' \
        | awk -F/ '{print $NF}' | grep -E '^v[0-9]+\.[0-9]+\.[0-9]+$' | sort -V | tail -1 || true)
  url=""
  [ -n "$tag" ] && url="https://github.com/su2code/SU2/releases/download/$tag/SU2-$tag-linux64.zip"
  if [ -n "$url" ]; then
    log "tải SU2: $url"
    tmp=$(mktemp -d)
    if curl -fsSL "$url" -o "$tmp/su2.zip" && unzip -q "$tmp/su2.zip" -d "$tmp/x"; then
      bin=$(find "$tmp/x" -name SU2_CFD -type f | head -1)
      rm -rf "$SU2_DIR"; mkdir -p "$SU2_DIR"
      cp -r "$(dirname "$(dirname "$bin")")"/. "$SU2_DIR"/
      chmod +x "$SU2_DIR"/bin/* || true
    else
      log "CẢNH BÁO: tải SU2 thất bại, bỏ qua"
    fi
    rm -rf "$tmp"
  else
    log "CẢNH BÁO: không tìm được bản SU2 linux64, bỏ qua"
  fi
fi

# ---------- 4. ArduPilot SITL ----------
if [ $SKIP_SITL -eq 0 ]; then
  if [ ! -d "$ARDUPILOT_DIR/.git" ]; then
    log "clone ArduPilot"
    git clone -q --depth 1 --recurse-submodules --shallow-submodules \
      https://github.com/ArduPilot/ardupilot.git "$ARDUPILOT_DIR"
  elif [ $UPGRADE -eq 1 ]; then
    git -C "$ARDUPILOT_DIR" pull -q --depth 1 && git -C "$ARDUPILOT_DIR" submodule update -q --init --recursive --depth 1
    rm -f "$ARDUPILOT_DIR/build/sitl/bin/arducopter"
  fi
  if [ ! -x "$ARDUPILOT_DIR/build/sitl/bin/arducopter" ]; then
    log "build ArduPilot SITL (copter, plane) — lần đầu ~10–15 phút"
    ( cd "$ARDUPILOT_DIR" && PATH="$VENV/bin:$PATH" ./waf configure --board sitl >/dev/null \
      && PATH="$VENV/bin:$PATH" ./waf copter plane >/dev/null )
  fi
fi

# ---------- 5. PX4 SITL (tùy chọn) ----------
if [ $WITH_PX4 -eq 1 ] && [ ! -x "$PX4_DIR/build/px4_sitl_default/bin/px4" ]; then
  [ -d "$PX4_DIR/.git" ] || git clone -q --recursive --depth 1 https://github.com/PX4/PX4-Autopilot.git "$PX4_DIR"
  "${UVPIP[@]}" -r "$PX4_DIR/Tools/setup/requirements.txt"
  ( cd "$PX4_DIR" && PATH="$VENV/bin:$PATH" make px4_sitl_default >/dev/null )
fi

# ---------- 6. Biến môi trường ----------
ENV_SH=/etc/profile.d/uav-env.sh
cat > "$ENV_SH" <<EOF
export UAV_VENV=$VENV
export PATH="$VENV/bin:$SU2_DIR/bin:$ARDUPILOT_DIR/Tools/autotest:\$PATH"
export SU2_RUN="$SU2_DIR/bin"
export PYTHONPATH="$SU2_DIR/bin\${PYTHONPATH:+:\$PYTHONPATH}"
export MUJOCO_GL=egl
export PYOPENGL_PLATFORM=egl
EOF
# OpenFOAM: nạp bashrc nếu có
of_rc=$(ls /usr/share/openfoam/etc/bashrc /usr/lib/openfoam/*/etc/bashrc 2>/dev/null | head -1 || true)
[ -n "$of_rc" ] && echo "alias of='source $of_rc'   # gõ 'of' để nạp OpenFOAM" >> "$ENV_SH"
if [ -n "${CLAUDE_ENV_FILE:-}" ]; then grep '^export' "$ENV_SH" >> "$CLAUDE_ENV_FILE"; fi

log "xong. Kiểm tra: $VENV/bin/python $HERE/check_env.py"
