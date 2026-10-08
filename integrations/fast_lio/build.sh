#!/usr/bin/env bash
set -euo pipefail
cd -- "$(dirname -- "${BASH_SOURCE[0]}")"
mkdir -p logs
exec > >(tee logs/build.log) 2>&1
free -h
docker_root=$(docker info --format '{{.DockerRootDir}}')
echo "Docker 存储目录：$docker_root"
df -h "$docker_root"
available_kib=$(df -Pk "$docker_root" | awk 'NR == 2 {print $4}')
if (( available_kib < 12 * 1024 * 1024 )); then
  echo "构建未开始：Docker 分区只有 ${available_kib} KiB 可用空间；至少需要 12 GiB。"
  echo '请让管理员提供足够存储空间，然后重新执行此命令。现有镜像、容器和存储设置未改变。'
  exit 3
fi
# 构建时自动获取依赖。编译使用两个并行任务。
docker build --progress=plain -t robomaster-fast-lio:7cc4175 .
