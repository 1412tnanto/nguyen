#!/bin/bash
# Cài môi trường mô phỏng UAV (khí động, động lực học bay, kiểm bền, SITL).
# Idempotent: chạy lại chỉ cài phần còn thiếu.
#   scripts/uav/setup.sh              cài phần thiếu
#   scripts/uav/setup.sh --upgrade    nâng cấp gói Python, SU2, ArduPilot, Gazebo
#   scripts/uav/setup.sh --skip-px4   bỏ qua PX4 + Gazebo
#   scripts/uav/setup.sh --skip-sitl  bỏ qua ArduPilot, PX4, Gazebo (cài nhanh)
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
VENV=/opt/uav-venv
SU2_DIR=/opt/su2
ARDUPILOT_DIR=/opt/ardupilot
PX4_DIR=/opt/PX4-Autopilot
PX4_TAG=v1.16.2                 # bản ổn định; đổi tag ở đây để nâng cấp PX4
GZ_ENV=/opt/gz                  # Gazebo Harmonic từ conda-forge (repo OSRF bị chặn)
MAMBA=/opt/micromamba/bin/micromamba
export MAMBA_ROOT_PREFIX=/opt/micromamba
UPGRADE=0; SKIP_PX4=0; SKIP_SITL=0
for a in "$@"; do
  case "$a" in
    --upgrade) UPGRADE=1 ;;
    --skip-px4) SKIP_PX4=1 ;;
    --with-px4) ;;  # giữ tương thích: PX4 nay cài mặc định
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
  paraview python3-paraview   # hậu xử lý (pvbatch chạy headless)
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
  # Tên file đổi theo phiên bản: ưu tiên bản OpenMP (song song, không cần MPI)
  for suffix in linux64-omp linux64; do
    u="https://github.com/su2code/SU2/releases/download/$tag/SU2-$tag-$suffix.zip"
    if [ -n "$tag" ] && curl -fsL -r 0-10 -o /dev/null "$u"; then url=$u; break; fi
  done
  if [ -n "$url" ]; then
    log "tải SU2: $url"
    tmp=$(mktemp -d)
    if curl -fsSL "$url" -o "$tmp/su2.zip" && unzip -q "$tmp/su2.zip" -d "$tmp/x"; then
      # Một số bản đóng gói zip lồng trong zip
      find "$tmp/x" -name '*.zip' -exec unzip -q -o {} -d "$tmp/x" \;
      bin=$(find "$tmp/x" -name SU2_CFD -type f | head -1)
      if [ -n "$bin" ]; then
        rm -rf "$SU2_DIR"; mkdir -p "$SU2_DIR"
        cp -r "$(dirname "$(dirname "$bin")")"/. "$SU2_DIR"/
        chmod +x "$SU2_DIR"/bin/* || true
      else
        log "CẢNH BÁO: không thấy SU2_CFD trong gói tải về, bỏ qua"
      fi
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

# ---------- 5. Gazebo Harmonic + PX4 SITL ----------
if [ $SKIP_SITL -eq 0 ] && [ $SKIP_PX4 -eq 0 ]; then
  if [ ! -x "$MAMBA" ]; then
    log "tải micromamba"
    mkdir -p "$(dirname "$MAMBA")"
    curl -fsSL -o "$MAMBA" https://github.com/mamba-org/micromamba-releases/releases/latest/download/micromamba-linux-64
    chmod +x "$MAMBA"
  fi
  if [ ! -x "$GZ_ENV/bin/gz" ]; then
    log "cài Gazebo Harmonic (gz-sim8) + OpenCV 4 vào $GZ_ENV"
    "$MAMBA" create -y -q -p "$GZ_ENV" -c conda-forge gz-sim8 gz-tools2 "libopencv=4.*" >/dev/null
  elif [ $UPGRADE -eq 1 ]; then
    "$MAMBA" update -y -q -p "$GZ_ENV" -c conda-forge --all >/dev/null
  fi
  # Lệnh gz chạy trong môi trường conda, không làm bẩn PATH hệ thống
  printf '#!/bin/bash\nexec %s run -p %s gz "$@"\n' "$MAMBA" "$GZ_ENV" > /usr/local/bin/gz
  chmod +x /usr/local/bin/gz

  if [ ! -d "$PX4_DIR/.git" ]; then
    log "clone PX4 $PX4_TAG"
    git clone -q --recursive --depth 1 --shallow-submodules -b "$PX4_TAG" \
      https://github.com/PX4/PX4-Autopilot.git "$PX4_DIR"
  elif [ "$(git -C "$PX4_DIR" describe --tags 2>/dev/null || true)" != "$PX4_TAG" ]; then
    log "đổi PX4 sang $PX4_TAG"
    git -C "$PX4_DIR" fetch -q --depth 1 origin tag "$PX4_TAG"
    git -C "$PX4_DIR" checkout -q "$PX4_TAG"
    git -C "$PX4_DIR" submodule update -q --init --recursive --depth 1
    rm -rf "${PX4_DIR:?}/build"
  fi
  if [ ! -x "$PX4_DIR/build/px4_sitl_default/bin/px4" ]; then
    "${UVPIP[@]}" -r "$PX4_DIR/Tools/setup/requirements.txt"
    # Clone nông thiếu tag NuttX -> script sinh header phiên bản lỗi; chỉ tải tag (depth 1)
    nuttx="$PX4_DIR/platforms/nuttx/NuttX/nuttx"
    if [ -z "$(git -C "$nuttx" tag -l 'nuttx-*' | head -1)" ]; then
      git -C "$nuttx" fetch -q --depth 1 origin 'refs/tags/nuttx-*:refs/tags/nuttx-*'
    fi
    # protobuf/abseil của conda-forge yêu cầu C++17; PX4 v1.16 đặt C++14 cứng trong CMakeLists
    sed -i 's/^set(CMAKE_CXX_STANDARD 14)/set(CMAKE_CXX_STANDARD 17)/' "$PX4_DIR/CMakeLists.txt"
    log "build PX4 SITL (kèm cầu nối Gazebo) — lần đầu ~15–20 phút"
    # CMAKE_PREFIX_PATH để tìm gz-transport/OpenCV; ép Python của venv (không lấy Python của conda)
    ( cd "$PX4_DIR" && PATH="$VENV/bin:$PATH" CMAKE_PREFIX_PATH="$GZ_ENV" make px4_sitl_default \
        CMAKE_ARGS="-DPYTHON_EXECUTABLE=$VENV/bin/python -DPython3_EXECUTABLE=$VENV/bin/python \
-DCMAKE_SHARED_LINKER_FLAGS=-L$GZ_ENV/lib -DCMAKE_EXE_LINKER_FLAGS=-L$GZ_ENV/lib \
-DCMAKE_CXX_FLAGS=-Wno-error=deprecated-declarations" >/dev/null )
    # (-Wno-error=deprecated-declarations: protobuf mới đánh dấu RepeatedField::Resize là deprecated)
  fi
fi

# ---------- 6. Biến môi trường ----------
ENV_SH=/etc/profile.d/uav-env.sh
cat > "$ENV_SH" <<EOF
export UAV_VENV=$VENV
export PATH="$VENV/bin:$SU2_DIR/bin:$ARDUPILOT_DIR/Tools/autotest:\$PATH"
export SU2_RUN="$SU2_DIR/bin"
export PX4_DIR=$PX4_DIR
export GZ_ENV=$GZ_ENV
export MAMBA_ROOT_PREFIX=/opt/micromamba
export PYTHONPATH="$SU2_DIR/bin\${PYTHONPATH:+:\$PYTHONPATH}"
export MUJOCO_GL=egl
export PYOPENGL_PLATFORM=egl
EOF
# OpenFOAM: nạp bashrc nếu có
of_rc=$(ls /usr/share/openfoam/etc/bashrc /usr/lib/openfoam/*/etc/bashrc 2>/dev/null | head -1 || true)
[ -n "$of_rc" ] && echo "alias of='source $of_rc'   # gõ 'of' để nạp OpenFOAM" >> "$ENV_SH"
if [ -n "${CLAUDE_ENV_FILE:-}" ]; then grep '^export' "$ENV_SH" >> "$CLAUDE_ENV_FILE"; fi

log "xong. Kiểm tra: $VENV/bin/python $HERE/check_env.py"
