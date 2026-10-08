#!/usr/bin/env bash
# 该脚本也供仓库入口加载。加载时不执行检查。
set -euo pipefail
rm_root=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)

rm_fail() {
  printf '错误：%s\n' "$*" >&2
  return 1
}
rm_dependency_error() {
  printf '错误：%s\n请在仓库根目录执行 ./rm setup。\n' "$*" >&2
  return 1
}
rm_prepare_path() {
  local rust_version='' cargo_version='' node_version=''
  if command -v rustc >/dev/null 2>&1; then rust_version=$(rustc --version 2>/dev/null | awk '{print $2}' || true); fi
  if command -v cargo >/dev/null 2>&1; then cargo_version=$(cargo --version 2>/dev/null | awk '{print $2}' || true); fi
  if [[ -x "$rm_root/.deps/cargo/bin/rustc" ]]; then
    export CARGO_HOME="$rm_root/.deps/cargo"
    export RUSTUP_HOME="$rm_root/.deps/rustup"
    export PATH="$CARGO_HOME/bin:$PATH"
  elif [[ -x "${HOME:-}/.cargo/bin/rustc" && -x "${HOME:-}/.cargo/bin/cargo" ]] &&
       { ! rm_version_at_least "$rust_version" 1.85.0 || ! rm_version_at_least "$cargo_version" 1.85.0; }; then
    rust_version=$("$HOME/.cargo/bin/rustc" --version 2>/dev/null | awk '{print $2}' || true)
    if rm_version_at_least "$rust_version" 1.85.0; then export PATH="$HOME/.cargo/bin:$PATH"; fi
  fi
  if command -v node >/dev/null 2>&1; then node_version=$(node --version 2>/dev/null || true); fi
  if [[ -x "$rm_root/.deps/node/bin/node" ]]; then
    export PATH="$rm_root/.deps/node/bin:$PATH"
  elif [[ -x "${HOME:-}/.local/bin/node" ]] && ! rm_version_at_least "${node_version#v}" 18.0.0; then
    node_version=$("$HOME/.local/bin/node" --version 2>/dev/null || true)
    if rm_version_at_least "${node_version#v}" 18.0.0; then export PATH="$HOME/.local/bin:$PATH"; fi
  fi
}
rm_version_at_least() {
  local version=$1 required=$2
  [[ "$version" =~ ^[0-9]+\.[0-9]+(\.[0-9]+)?$ ]] || return 1
  [[ $(printf '%s\n%s\n' "$required" "$version" | sort -V | head -n 1) == "$required" ]]
}
rm_check_platform() {
  [[ $(uname -s) == Linux && $(uname -m) == x86_64 ]] ||
    rm_fail '本入口仅支持已验证的 Ubuntu 22.04 x86_64。'
  local ID='' VERSION_ID=''
  [[ -r /etc/os-release ]] || rm_fail '无法读取 /etc/os-release。'
  source /etc/os-release
  [[ "$ID" == ubuntu && "$VERSION_ID" == 22.04 ]] ||
    rm_fail "当前系统为 $ID $VERSION_ID。本入口仅支持已验证的 Ubuntu 22.04 x86_64。"
}
rm_check_tools() {
  local tool version
  for tool in cmake ninja c++ cargo rustc pkg-config; do
    command -v "$tool" >/dev/null 2>&1 || rm_dependency_error "缺少工具：$tool。"
  done
  version=$(cmake --version 2>/dev/null | sed -n '1s/^cmake version //p' || true)
  rm_version_at_least "$version" 3.22.0 || rm_dependency_error "CMake 需要 3.22 或更高版本。当前版本：$version。"
  version=$(rustc --version 2>/dev/null | awk '{print $2}' || true)
  rm_version_at_least "$version" 1.85.0 || rm_dependency_error "Rust 需要 1.85 或更高版本。当前版本：$version。"
  version=$(cargo --version 2>/dev/null | awk '{print $2}' || true)
  rm_version_at_least "$version" 1.85.0 || rm_dependency_error "Cargo 需要 1.85 或更高版本。当前版本：$version。"
  printf '%s\n' '#include <concepts>' 'template<class T> requires std::integral<T> constexpr T value(T x) { return x; }' 'static_assert(value(1) == 1);' |
    c++ -std=c++20 -x c++ -fsyntax-only - >/dev/null 2>&1 ||
    rm_dependency_error 'C++ 编译器不能编译 C++20 程序。'
}
rm_check_packages() {
  local library
  for library in eigen3 nlohmann_json libxml-2.0 glfw3 egl gl libjpeg libpng; do
    pkg-config --exists "$library" || rm_dependency_error "缺少开发库：$library。"
  done
  pkg-config --atleast-version=3.3 eigen3 || rm_dependency_error 'Eigen 需要 3.3 或更高版本。'
  pkg-config --atleast-version=3.9 nlohmann_json || rm_dependency_error 'nlohmann JSON 需要 3.9 或更高版本。'
}
rm_check_sdks() {
  local mujoco=${MUJOCO_ROOT:-"$rm_root/.deps/mujoco"}
  local opencv=${OpenCV_DIR:-"$rm_root/.deps/opencv/lib/cmake/opencv4"}
  local version
  [[ -r "$mujoco/include/mujoco/mujoco.h" ]] || rm_dependency_error "缺少 MuJoCo 头文件：$mujoco/include/mujoco/mujoco.h。"
  grep -Eq '^#define mjVERSION_HEADER +3015000([[:space:]]|$)' "$mujoco/include/mujoco/mujoco.h" ||
    rm_dependency_error '本仓库需要 MuJoCo 3.15.0。'
  [[ -r "$mujoco/libmujoco.so" || -r "$mujoco/lib/libmujoco.so" ]] ||
    rm_dependency_error "缺少 MuJoCo 共享库：$mujoco。"
  [[ -r "$opencv/OpenCVConfig.cmake" && -r "$opencv/OpenCVConfig-version.cmake" ]] ||
    rm_dependency_error "缺少 OpenCV C++ SDK：$opencv。"
  version=$(sed -n 's/^set(OpenCV_VERSION[[:space:]]\+"\?\([0-9.]*\)"\?).*/\1/p' "$opencv/OpenCVConfig-version.cmake" | head -n 1)
  rm_version_at_least "$version" 4.10.0 || rm_dependency_error "OpenCV 需要 4.10 或更高版本。当前版本：$version。"
  local component
  for component in core imgproc imgcodecs calib3d dnn; do
    [[ -r "$opencv/../../libopencv_$component.so" || -r "$opencv/../../libopencv_$component.a" ]] ||
      rm_dependency_error "缺少 OpenCV 库：libopencv_$component。"
  done
}
rm_check_node() {
  local version
  command -v ctest >/dev/null 2>&1 || rm_dependency_error '缺少 CTest。请安装完整的 CMake 工具。'
  if ! command -v node >/dev/null 2>&1; then
    printf '错误：测试需要 Node.js 18 或更高版本。\n请执行 ./rm setup --with-tests。日常启动不需要 Node.js。\n' >&2
    return 1
  fi
  version=$(node --version 2>/dev/null || true)
  if ! rm_version_at_least "${version#v}" 18.0.0; then
    printf '错误：测试需要 Node.js 18 或更高版本。当前版本：%s。\n请执行 ./rm setup --with-tests。\n' "$version" >&2
    return 1
  fi
}
rm_check_build() {
  rm_check_platform
  rm_check_tools
  rm_check_packages
  rm_check_sdks
}
rm_require_files() {
  local file
  for file in "$@"; do
    [[ -s "$rm_root/$file" ]] || rm_fail "缺少仓库文件：$file。请恢复此文件后重试。"
  done
}
rm_check_assets() {
  local module=$1
  [[ -d "$rm_root/assets" ]] || rm_fail '缺少仓库根目录：assets。请恢复此目录后重试。'
  case "$module" in
    navigation)
      rm_require_files assets/navigation.xml assets/arena/rmuc2023.stl assets/meshes/armor_am02.obj assets/meshes/armor_frame_a.obj web/navigation/index.html ;;
    duel)
      rm_require_files assets/robot.xml assets/meshes/armor_am02.obj assets/meshes/armor_frame_a.obj assets/armor_labels/3.png assets/rm_auto_aim/mlp.onnx assets/rm_auto_aim/label.txt web/duel/index.html web/duel/app.js ;;
    robot) rm_require_files assets/robot.xml assets/meshes/armor_am02.obj assets/meshes/armor_frame_a.obj ;;
    cube) ;;
    cube-dual)
      rm_require_files assets/robotiq_2f85/2f85.xml
      local mesh
      for mesh in base base_mount driver coupler follower spring_link pad silicone_pad; do
        rm_require_files "assets/robotiq_2f85/assets/$mesh.stl"
      done ;;
    all) rm_check_assets navigation; rm_check_assets duel; rm_check_assets robot; rm_check_assets cube-dual ;;
    *) rm_fail "未知模块：$module。" ;;
  esac
}
rm_check_display() {
  [[ -n "${DISPLAY:-}" ]] ||
    rm_fail '当前未设置 DISPLAY。请在 Linux 图形桌面中运行窗口命令。无窗口运行时请显式添加 --headless。'
}

if [[ "${BASH_SOURCE[0]}" == "$0" ]]; then
  mode=${1:-}
  [[ $# -le 1 && ( -z "$mode" || "$mode" == --test ) ]] || rm_fail '用法：./rm doctor [--test]。'
  rm_prepare_path
  rm_check_build
  rm_check_assets all
  if [[ "$mode" == --test ]]; then rm_check_node; fi
  printf '检查通过：Ubuntu 22.04 x86_64、C++20、CMake、Rust、系统开发库、MuJoCo、OpenCV 和运行资产。\n'
  if [[ "$mode" == --test ]]; then
    printf '检查通过：Node.js 测试依赖。\n'
  else
    printf 'Node.js 仅用于测试。使用 ./rm doctor --test 检查测试依赖。\n'
  fi
  if [[ -z "${DISPLAY:-}" ]]; then printf '当前未设置 DISPLAY。浏览器服务和无窗口程序仍可运行。\n'; fi
fi
