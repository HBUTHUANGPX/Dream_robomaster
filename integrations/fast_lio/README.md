# FAST-LIO 离线回放集成

## 用途与当前状态

本集成用于离线验证官方 FAST-LIO2。
它不参与默认网页启动，也不替代当前导航的 ICP 定位。

**本机尚未完成 ROS 构建、容器回放和估计轨迹验收。**
已通过的原生进程管理测试不能证明估计器可运行。
下列步骤是完整环境中的复现说明，不能视为本机成功记录。

容器使用 ROS1 Noetic 和 catkin。
FAST-LIO 使用 MARSIM 模式，配置值为 `lidar_type: 4`。
容器编译 Livox 驱动与 SDK，以满足上游消息类型依赖。
回放时不启动硬件驱动。

## 操作前的条件

所有下列命令在仓库根目录执行。

| 条件 | 要求 |
| --- | --- |
| 主机 | Linux x86_64；本仓库原生准备流程验证于 Ubuntu 22.04 |
| 原生程序 | 已成功执行 `./rm setup` |
| Docker | 已安装，服务已启动，当前用户执行 `docker info` 成功 |
| 网络 | 构建时能下载 ROS 基础镜像、软件包和固定提交源码 |
| 容器存储 | Docker 存储所在分区至少有 12 GiB 余量；这是构建脚本的预检阈值 |
| 数据目录 | 当前用户可读写；有符合下文格式的 `input.bag` |
| 输出路径 | 不存在 `lio_output.bag` 和 `lio_output.bag.active` |

如果尚未安装 Docker，按[Docker 官方 Ubuntu 安装说明](https://docs.docker.com/engine/install/ubuntu/)安装并完成其验证步骤。
当前仓库脚本不安装 Docker，也不修改 Docker 存储位置。
如果当前用户无法执行 `docker info`，先让管理员处理 Docker 访问权限。

不需要在主机安装 ROS。
把仓库移动到另一分区不会自动移动 Docker 镜像层。

## 构建镜像

执行：

~~~bash
bash integrations/fast_lio/build.sh
~~~

脚本先检查 Docker 存储，再构建 `robomaster-fast-lio:7cc4175`。
日志写入 `integrations/fast_lio/logs/build.log`。
脚本会自动获取固定提交，不要求用户初始化根 Git 子模块。

构建成功后，才进行回放。
存储检查失败时，脚本返回状态码 3，并且不开始构建。
让管理员提供足够 Docker 存储后，再执行同一条命令。
不要通过删除不明用途的镜像或数据来绕过容量检查。

## 准备输入并回放

如果已有符合格式的 ROS1 数据包，将它复制为一个新目录中的 `input.bag`。
每次回放使用不同的数据目录。
以下命令从原生导航生成一份新数据，不依赖历史输出文件。

1. 确认 `output/fast-lio-example/` 尚不存在。
2. 执行 `mkdir -p output/fast-lio-example`。
3. 执行原生录制命令：

~~~bash
build/bin/rm_record_lio --root "$PWD" --case 0 --seconds 60 --output "$PWD/output/fast-lio-example/input.bag"
~~~

录制成功后，目录中应有 `input.bag`。
程序还会单独记录参考真值。
如果录制未到达目标，程序返回状态码 2。
此时先检查录制报告，再决定是否使用该数据包。

执行回放：

~~~bash
bash integrations/fast_lio/run.sh output/fast-lio-example
~~~

回放成功时，输出文件为 `output/fast-lio-example/lio_output.bag`。
它只包含 `/Odometry`。
终端显示进程与回放状态。
已有输出会导致命令失败，防止覆盖结果。
要再次回放，准备另一个数据目录。

容器使用 `--network none`。
ROS 主节点和相关进程通过容器内回环地址通信。
容器不使用主机网络，不映射设备，不发布端口。
退出时只移除本次新建的临时容器。

## 输入数据约定

| 数据 | 类型与含义 |
| --- | --- |
| `/sim/lidar` | `sensor_msgs/PointCloud2`；上方雷达的瞬时点云；10 Hz |
| 点字段 | float32 类型的 `x,y,z,intensity`；坐标属于上雷达光学坐标系 |
| `/sim/imu` | `sensor_msgs/Imu`；200 Hz 原始角速度与比力 |
| 角速度 | 单位 rad/s，属于 IMU 坐标系 |
| 比力 | 单位 m/s²，包含重力响应 |
| 时间戳 | 使用同步且单调的仿真时间；数据包记录按时间排序 |
| 启动段 | 包含静止段，用于重力和偏置初始化 |
| 末尾数据 | IMU 采样至少覆盖最后一帧点云时间 |

数据不能输入估计位姿或仿真真值。
FAST-LIO 不把 IMU 消息中的姿态作为外部姿态解。
回放程序不补造缺少的 IMU 数据，也不重新采样。

外参约定为 `p_IMU = R * p_LiDAR + t`。
`R` 为单位矩阵。`t = [0, 0, 0.438]`，单位为 m。
关闭在线外参估计。

点云不含人工扫描线时间、逐点时间戳或雷达与 IMU 的时间偏移。
上游 `Preprocess::sim_handler` 将曲率设为零。
`sync_packages` 用消息头时间作为扫描结束时间。
MARSIM 路径跳过点云去畸变。
因此，这里模拟瞬时点云，不模拟真实 MID-360 的逐点扫描过程。

## 回放管理行为

原生 `fast_lio_replay` 先验证输入消息类型与点云布局。
然后启动 ROS、等待订阅就绪、启动里程计录制，并以 0.5 倍速度播放输入。
输入时间戳保持原样，回放同时发布 `/clock`。

回放结束后，程序等待 5 s 墙钟时间，让回调继续处理。
这个等待时间不能保证所有扫描均已处理。
评估结果时，检查里程计数量、时间覆盖和日志。

退出顺序是播放器、录制器、建图进程。
程序向各进程组发送 SIGINT。
每个进程有 15 s 退出时间，之后才记录并执行 SIGKILL 后备处理。
发生强制终止或数据包未完成封装时，任务不能报告成功。
SIGTERM 和中途失败也执行清理。

管理器检查位姿为有限值、时间戳不倒退、输出不为空。
这些是数据传输检查，不是定位精度评价。
估计位姿属于 FAST-LIO 初始化的 `camera_init` 局部坐标系。
精度对比前，必须单独对齐真值坐标系，且不能将真值输入估计器。
噪声配置来自上游初始值，不是传感器标定结果。

## 固定来源

| 组件 | 固定版本或提交 |
| --- | --- |
| ROS 基础镜像 | `ros:noetic-ros-base-focal@sha256:72b8bc59035dc0a5b8e07aae28c16caa84192971d72d207c72ed734fb1d5e97d` |
| FAST-LIO 源码 | `7cc4175de6f8ba2edf34bab02a42195b141027e9` |
| ikd-Tree 子模块 | `e2e3f4e9d3b95a9e66b1ba83dc98d4a05ed8a3c4` |
| Livox 驱动 | `3d240d5666129e1a3052e78ee8487a04b08fdda3` |
| Livox 开发包 | `9306596a2bf15c1343bc023b497465ed0a32909d` |

容器固定检出提交，并初始化上游递归子模块。
SDK 与 catkin 使用两个并行编译任务。
先编译 Livox 消息，再编译 FAST-LIO。估计器源码不打补丁。
版本和软件包清单写入镜像中的 `/opt/build-manifest.txt`。
Ubuntu 软件包源没有固定到快照，因此不能保证逐字节相同的镜像。

管理器使用 C++17，以兼容 Noetic 生成的旧分配器接口。
当前集成入口不调用自有 Python 脚本。
上游 ROS 工具仍保留自身的 Python 依赖。
FAST-LIO 的原始许可证随上游源码保留。

## 本机验证与历史证据

2026-10-08 的原生进程管理测试通过。
测试使用真实子进程，覆盖非零退出、执行失败、进程组信号、录制器先退出、
强制终止后备处理、继续清理、等待就绪超时和取消。
该测试不编译 ROS 消息与数据包部分。

如果只验证进程管理器，在仓库根目录执行：

~~~bash
cmake -S integrations/fast_lio -B build-replay -G Ninja -DRM_REPLAY_BUILD_ROS=OFF -DCMAKE_BUILD_TYPE=Release
cmake --build build-replay -j2
TMPDIR="$PWD/build-replay" ctest --test-dir build-replay --output-on-failure
~~~

完整 ROS 配置曾因缺少 `catkinConfig.cmake` 停止。
Docker 存储检查曾返回状态码 3。
当时的基础镜像拉取耗尽系统分区，但没有完成估计器构建或回放。
这些历史记录位于本机 `integrations/fast_lio/logs/`，新仓库不一定包含它们。

经用户授权，原任务把新增的 ROS 基础镜像保存到主机目录后移除。
归档文件为 `integrations/fast_lio/ros-noetic-ros-base-focal.tar`。
其大小为 491,606,016 字节。
SHA-256 为 `961170b9eac34ba06d621d418e587aaee27cc1154a6f68d86df1e8bdd5a675ec`。
没有删除此前已有的镜像、容器、卷或构建缓存。
清理后的系统空间仍不足以完成构建。
这个本机归档不是新用户的前提；构建脚本可下载固定基础镜像。

后续仍需完成镜像构建、无网络回放、非空里程计和时间覆盖检查。
轨迹精度需要单独评价。
