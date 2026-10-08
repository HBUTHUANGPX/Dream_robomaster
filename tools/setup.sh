#!/usr/bin/env bash
set -euo pipefail
root=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
source "$root/tools/doctor.sh"
with_tests=false
if [[ $# == 1 && "$1" == --with-tests ]]; then with_tests=true;
elif [[ $# != 0 ]]; then rm_fail '用法：./rm setup [--with-tests]。'; fi
rm_prepare_path
rm_check_platform
mkdir -p "$root/.cache/tmp" "$root/.deps/downloads"
export TMPDIR="$root/.cache/tmp"
exec 9>"$root/.cache/setup.lock"
flock -n 9 || rm_fail '另一个准备命令正在运行。请等待该命令结束。'

# 只安装缺少的系统包。日常入口不会运行此步骤。
packages=(build-essential cmake ninja-build pkg-config curl ca-certificates tar xz-utils
  libeigen3-dev nlohmann-json3-dev libxml2-dev libglfw3-dev libegl1-mesa-dev
  libgl1-mesa-dev libjpeg-dev libpng-dev ffmpeg)
missing=()
for package in "${packages[@]}"; do
  status=$(dpkg-query -W -f='${Status}' "$package" 2>/dev/null || true)
  if [[ "$status" != 'install ok installed' ]]; then missing+=("$package"); fi
done
rust_missing=false
if ! command -v rustc >/dev/null 2>&1 || ! command -v cargo >/dev/null 2>&1; then
  rust_missing=true
elif ! rm_version_at_least "$(rustc --version 2>/dev/null | awk '{print $2}' || true)" 1.85.0 ||
     ! rm_version_at_least "$(cargo --version 2>/dev/null | awk '{print $2}' || true)" 1.85.0; then
  rust_missing=true
fi
node_missing=false
if [[ "$with_tests" == true ]]; then
  if ! command -v node >/dev/null 2>&1; then node_missing=true
  elif ! rm_version_at_least "$(node --version 2>/dev/null | sed 's/^v//' || true)" 18.0.0; then node_missing=true; fi
fi
# 此阈值覆盖常见构建缓存。下载依赖时增加空间要求。
required_kb=1048576
if [[ ! -r "$root/.deps/opencv/lib/cmake/opencv4/OpenCVConfig.cmake" ]]; then required_kb=$((required_kb + 3145728)); fi
if [[ ! -r "$root/.deps/mujoco/include/mujoco/mujoco.h" ]]; then required_kb=$((required_kb + 262144)); fi
if [[ "$rust_missing" == true ]]; then required_kb=$((required_kb + 1572864)); fi
if [[ "$node_missing" == true ]]; then required_kb=$((required_kb + 262144)); fi
available_kb=$(df -Pk "$root" | awk 'END {print $4}')
[[ "$available_kb" =~ ^[0-9]+$ && "$available_kb" -ge "$required_kb" ]] ||
  rm_fail "仓库所在分区空间不足。至少需要 $((required_kb / 1024)) MiB 可用空间。"
printf '准备平台：Ubuntu 22.04 x86_64。\n构建缓存和本地 SDK 写入：%s。\n' "$root"
if [[ ${#missing[@]} -gt 0 ]]; then
  printf '需要安装系统包：%s\n' "${missing[*]}"
  if [[ $(id -u) == 0 ]]; then installer=(apt-get)
  elif command -v sudo >/dev/null 2>&1; then installer=(sudo apt-get)
  else rm_fail '缺少 sudo。请让管理员安装上列系统包，然后重试 ./rm setup。'; fi
  "${installer[@]}" update
  "${installer[@]}" install -y --no-install-recommends "${missing[@]}"
else
  printf '系统包已齐全。不执行 apt 安装。\n'
fi
if [[ "$rust_missing" == true ]]; then
  printf '正在将 Rust stable 最小工具链安装到仓库 .deps。不会修改用户的 shell 配置。\n'
  curl --proto '=https' --tlsv1.2 -fL --retry 2 --connect-timeout 15 --max-time 240 \
    https://sh.rustup.rs -o "$root/.deps/downloads/rustup-init.sh"
  CARGO_HOME="$root/.deps/cargo" RUSTUP_HOME="$root/.deps/rustup" RUSTUP_INIT_SKIP_PATH_CHECK=yes \
    sh "$root/.deps/downloads/rustup-init.sh" -y --profile minimal --default-toolchain stable --no-modify-path
  rm_prepare_path
fi
if [[ "$node_missing" == true ]]; then
  node_name=node-v22.15.0-linux-x64
  node_archive="$root/.deps/downloads/$node_name.tar.xz"
  if [[ -e "$root/.deps/node" || -L "$root/.deps/node" ]]; then
    rm_fail '仓库 .deps/node 已存在，但版本不可用。请先备份并移走此目录，然后重试 ./rm setup --with-tests。'
  fi
  printf '正在下载 Node.js 22.15.0。该版本仅用于测试。\n'
  curl --proto '=https' --tlsv1.2 -fL --retry 2 --connect-timeout 15 --max-time 240 \
    https://nodejs.org/dist/v22.15.0/SHASUMS256.txt -o "$root/.deps/downloads/node-SHASUMS256.txt"
  checksum=$(awk -v name="$node_name.tar.xz" '$2 == name {print $1}' "$root/.deps/downloads/node-SHASUMS256.txt")
  [[ "$checksum" =~ ^[0-9a-f]{64}$ ]] || rm_fail 'Node.js 官方校验清单缺少目标文件。'
  curl --proto '=https' --tlsv1.2 -fL --retry 2 --connect-timeout 15 --max-time 240 \
    "https://nodejs.org/dist/v22.15.0/$node_name.tar.xz" -o "$node_archive.part"
  printf '%s  %s\n' "$checksum" "$node_archive.part" | sha256sum --check --status || rm_fail 'Node.js 文件校验失败。'
  mv -- "$node_archive.part" "$node_archive"
  stage=$(mktemp -d "$root/.deps/.node-stage.XXXXXX")
  trap 'rm -rf -- "$stage"' EXIT
  tar -xJf "$node_archive" -C "$stage" --strip-components=1
  mv -- "$stage" "$root/.deps/node"
  trap - EXIT
  rm_prepare_path
fi
bash "$root/tools/bootstrap-native.sh"
rm_check_build
if [[ "$with_tests" == true ]]; then rm_check_node; fi
printf '依赖准备完成。正在构建程序。\n'
bash "$root/rm" build
printf '准备完成。执行 ./rm 启动导航和自瞄对战服务。\n'
if [[ "$with_tests" == false ]]; then printf '运行测试前可执行 ./rm setup --with-tests。\n'; fi
