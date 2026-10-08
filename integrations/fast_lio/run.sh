#!/usr/bin/env bash
set -euo pipefail
if [[ $# -ne 1 ]]; then
  echo "用法：bash $0 数据目录。目录内须有 input.bag；结果写入 lio_output.bag。" >&2
  exit 2
fi
data_dir=$(realpath -- "$1")
[[ -f "$data_dir/input.bag" ]] || { echo '数据目录缺少 input.bag。请先准备输入数据包。' >&2; exit 2; }
[[ ! -e "$data_dir/lio_output.bag" && ! -e "$data_dir/lio_output.bag.active" ]] || {
  echo '输出已存在。请选择新的数据目录，防止覆盖结果。' >&2; exit 2;
}
# 只移除本次新建的临时容器，保留已有容器。
exec docker run --rm --init --network none --cap-drop ALL \
  --security-opt no-new-privileges --user "$(id -u):$(id -g)" \
  --mount "type=bind,src=$data_dir,dst=/data" \
  --mount "type=bind,src=$data_dir/input.bag,dst=/data/input.bag,readonly" \
  robomaster-fast-lio:7cc4175
