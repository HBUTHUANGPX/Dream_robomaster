#!/usr/bin/env bash
set -euo pipefail
root=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
[[ $# == 0 ]] || { printf '错误：SDK 准备脚本不接受参数。\n' >&2; exit 2; }
cd "$root"
mkdir -p .deps/downloads .deps/sources .cache/tmp output/build
export TMPDIR="$root/.cache/tmp"
for tool in curl tar sha256sum cmake ninja c++ flock; do
  command -v "$tool" >/dev/null || { printf '错误：缺少工具 %s。请执行 ./rm setup。\n' "$tool" >&2; exit 1; }
done
[[ $(uname -s) == Linux && $(uname -m) == x86_64 ]] || {
  printf '错误：此 SDK 准备脚本仅支持 Linux x86_64。\n' >&2; exit 1;
}
exec 8>"$root/.cache/sdk.lock"
flock -n 8 || { printf '错误：另一个 SDK 准备命令正在运行。\n' >&2; exit 1; }
download() {
  local url=$1 file=$2 expected=$3
  if [[ ! -f "$file" ]] || ! printf '%s  %s\n' "$expected" "$file" | sha256sum --check --status; then
    printf '正在下载：%s\n' "${file##*/}"
    curl --proto '=https' --tlsv1.2 -fL --retry 2 --connect-timeout 15 --max-time 240 "$url" -o "$file.part"
    if ! printf '%s  %s\n' "$expected" "$file.part" | sha256sum --check --status; then
      printf '错误：文件 SHA256 校验失败：%s。\n' "$file.part" >&2
      return 1
    fi
    mv -- "$file.part" "$file"
  fi
}
# 保留已验证的版本和 SHA256。禁止解压覆盖已安装的共享库。
if [[ -r .deps/mujoco/include/mujoco/mujoco.h ]] &&
   grep -Eq '^#define mjVERSION_HEADER +3015000([[:space:]]|$)' .deps/mujoco/include/mujoco/mujoco.h &&
   [[ -r .deps/mujoco/lib/libmujoco.so || -r .deps/mujoco/libmujoco.so ]]; then
  printf 'MuJoCo 3.15.0 已就绪。不改写已安装文件。\n'
else
  if [[ -e .deps/mujoco || -L .deps/mujoco ]]; then
    printf '错误：.deps/mujoco 已存在，但不是完整的 MuJoCo 3.15.0。\n请停止使用该 SDK 的程序，再备份并移走此目录。然后执行 ./rm setup。\n' >&2
    exit 1
  fi
  download 'https://github.com/google-deepmind/mujoco/releases/download/3.15.0/mujoco-3.15.0-linux-x86_64.tar.gz' \
    .deps/downloads/mujoco-3.15.0-linux-x86_64.tar.gz \
    319943b332fd84c968f655f695d8f4b03cc6a68f658bacd0591f0a552744e497
  stage=$(mktemp -d "$root/.deps/.mujoco-stage.XXXXXX")
  trap 'rm -rf -- "$stage"' EXIT
  tar -xzf .deps/downloads/mujoco-3.15.0-linux-x86_64.tar.gz -C "$stage" --strip-components=1
  mv -- "$stage" "$root/.deps/mujoco"
  trap - EXIT
fi
opencv_ready=true
if [[ ! -r .deps/opencv/lib/cmake/opencv4/OpenCVConfig.cmake ]] ||
   ! grep -Eq '^set\(OpenCV_VERSION[[:space:]]+"?4\.12\.0"?\)' .deps/opencv/lib/cmake/opencv4/OpenCVConfig-version.cmake 2>/dev/null; then
  opencv_ready=false
fi
for component in core imgproc imgcodecs calib3d dnn; do
  if [[ ! -r ".deps/opencv/lib/libopencv_$component.so" ]]; then opencv_ready=false; fi
done
if [[ "$opencv_ready" == true ]]; then
  printf 'OpenCV 4.12.0 已就绪。不改写已安装文件。\n'
else
  if [[ -e .deps/opencv || -L .deps/opencv ]]; then
    printf '错误：.deps/opencv 已存在，但不是完整的 OpenCV 4.12.0。\n请停止使用该 SDK 的程序，再备份并移走此目录。然后执行 ./rm setup。\n' >&2
    exit 1
  fi
  download 'https://github.com/opencv/opencv/archive/refs/tags/4.12.0.tar.gz' \
    .deps/downloads/opencv-4.12.0.tar.gz \
    44c106d5bb47efec04e531fd93008b3fcd1d27138985c5baf4eafac0e1ec9e9d
  if [[ ! -r .deps/sources/opencv-4.12.0/CMakeLists.txt ]]; then
    tar -xzf .deps/downloads/opencv-4.12.0.tar.gz -C .deps/sources
  fi
  jobs=${RM_BUILD_JOBS:-4}
  [[ "$jobs" =~ ^[1-9][0-9]*$ ]] || { printf '错误：RM_BUILD_JOBS 必须是正整数。\n' >&2; exit 2; }
  cmake -S .deps/sources/opencv-4.12.0 -B .deps/opencv-build -G Ninja \
    -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX="$root/.deps/opencv" \
    -DBUILD_LIST=core,imgproc,imgcodecs,calib3d,dnn -DBUILD_TESTS=OFF \
    -DBUILD_PERF_TESTS=OFF -DBUILD_EXAMPLES=OFF -DBUILD_opencv_apps=OFF \
    -DBUILD_opencv_python2=OFF -DBUILD_opencv_python3=OFF -DBUILD_JAVA=OFF \
    -DWITH_IPP=OFF -DWITH_TBB=OFF -DWITH_OPENCL=OFF -DWITH_GDAL=OFF \
    -DWITH_GDCM=OFF -DWITH_OPENEXR=OFF -DWITH_FFMPEG=OFF -DWITH_GSTREAMER=OFF \
    -DWITH_QT=OFF -DWITH_GTK=OFF -DWITH_VTK=OFF -DWITH_1394=OFF \
    -DWITH_OPENJPEG=OFF -DWITH_TIFF=OFF -DWITH_WEBP=OFF \
    -DBUILD_PROTOBUF=ON -DCPU_DISPATCH= -DOPENCV_DNN_OPENCL=OFF
  cmake --build .deps/opencv-build --parallel "$jobs"
  stage=$(mktemp -d "$root/.deps/.opencv-stage.XXXXXX")
  trap 'rm -rf -- "$stage"' EXIT
  cmake --install .deps/opencv-build --prefix "$stage"
  mv -- "$stage" "$root/.deps/opencv"
  trap - EXIT
fi
printf '本地 SDK 准备完成。\n'
