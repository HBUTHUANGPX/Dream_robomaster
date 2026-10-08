# 底盘与导航模块

技术名称和缩写见[术语表](../glossary.md)。

`rm_navigation` 使用 C++20 实现导航。`rm_robot` 提供由轮地接触驱动的自由驾驶演示。原始赛场、装甲网格、模型 XML 和来源记录保存在 `assets/` 中。

## 启动与检查

首次使用时，请先阅读[根目录说明](../../README.md)和[入门说明](../getting-started.md)。下列命令均在仓库根目录运行。先完成 `./rm setup`。

```bash
./rm navigation
```

该命令在前台启动导航服务。使用终端中的 Ctrl+C 停止服务。可通过 `./rm navigation --port 8765` 指定端口。需要同时启动导航与对战时，运行 `./rm` 或 `./rm start`。需要自由驾驶窗口时，运行 `./rm robot`。

以下原生命令供无窗口运行、录制和验证使用：

```bash
build/bin/rm_navigation --root "$PWD" --headless --duration 8 --goal 7.5 5
build/bin/rm_navigation --root "$PWD" --rpc
build/bin/rm_navigation --root "$PWD" --localization slam --headless --duration 2
build/bin/rm_robot --root "$PWD" --headless --duration 3 --velocity 0 .4 0
build/bin/rm_robot --root "$PWD" --viewer --duration 120
build/bin/rm_navigation_build_models --root "$PWD" --output "$PWD/output/generated-models"
build/bin/rm_validate_navigation --root "$PWD" --seconds 180 --output "$PWD/output/navigation-validation.json"
build/bin/rm_record_navigation --root "$PWD" --case 0 --seconds 15 --output "$PWD/output/navigation.mp4"
ctest --test-dir build -R navigation --output-on-failure
```

## 物理、感知与控制

机器人通过四个轮轴速度执行器和 48 个被动滚子接触移动。导航不会把期望位置直接写入 MuJoCo 状态。

两颗参考 MID-360 的传感器使用 MuJoCo 射线相交。模型保留自身遮挡、旧版简化射线分布、量程、视场和测距噪声。原始数据包包含传感器坐标、安装外参和时间戳。

轮编码器和陀螺仪为重力对齐的 ICP 配准提供预测。`prior` 模式使用 CAD 采样法向，执行稳健的点到平面配准。`slam` 模式对累计扫描地图执行有界的点到点配准。仿真位姿仅用于生成测量、渲染和诊断误差。

启动时，模块直接从二进制 STL 重建地形。它保留自由空间支撑面选择、顶部净空、28° 坡度过渡限制和 0.38 m 边界膨胀。原生代码不依赖旧版 NumPy 地形缓存。

A* 保留高度与净空代价，并禁止从对角障碍缝隙穿过。全向 DWA 检查加速度、轮端功率、扫描障碍、跟踪余量、扫掠线段，以及常规预测时域以外的完整制动段。平移与偏航分别接收指令。

轨迹 PID 和速度反馈保留积分抗饱和。执行器力矩分配包含不回收能量的电功率、转速损耗和堵转损耗。这些系数仍是未标定的模型假设，不代表实测电机效率或竞赛官方功率限值。

估计器仍是局部重力对齐 ICP。它不提供完整惯性状态估计、全局重定位或回环。FAST-LIO 回放属于独立集成，不属于导航进程。

## 接口与模型重建

逐行 JSON 的 RPC 接受 `state`、`tick`、`reset`、`pause`、`frame`、`map`、`goal` 和 `yaw`。状态和地图字段与导航网页一致。图像输出为带全局路径和局部路径叠加的 JPEG。Rust 服务处理 HTTP，并按顺序访问原生进程。

`rm_navigation_build_models` 根据物理参数和原始 CAD 输入，重新生成两个 XML 模型。其中包括向内挤出 3 mm 的三角棱柱地形碰撞面。命令要求显式指定输出目录，并使用 MuJoCo 加载器检查输出。

生成模型使用绝对网格路径，因此可在 `assets` 以外的目录验证。移动仓库后应重新生成这些模型。原始来源仍记录在 `assets/arena/source.json` 中。重新生成 XML 不会改变模型来源。

## 录制与驾驶窗口

以下命令记录原始传感器数据：

```bash
build/bin/rm_navigation --root "$PWD" --headless --duration 5 --goal 7.5 5 \
  --record "$PWD/output/raw.jsonl"
```

JSONL 包含元数据、200 Hz 陀螺仪与比力记录，以及两颗雷达各 10 Hz 的数据包。原始 IMU 记录不含理想姿态或真值位姿。

需要 ROS1 数据包时，先选择尚不存在的输出文件，再运行：

```bash
build/bin/rm_record_lio --root "$PWD" --case 0 --seconds 15 \
  --output "$PWD/output/native-lio.bag"
```

原生程序写入未压缩的 ROS bag v2。消息类型为 `sensor_msgs/Imu` 和 `sensor_msgs/PointCloud2`，话题为 `/sim/imu` 和 `/sim/lidar`。数据包只包含上雷达，并将 IMU 姿态标记为不可用。程序保留原有的 2 s 静止初始化，并把真值单独写入旁边的 JSON 文件。

已有数据包不会被覆盖。若指定时长不足以到达目标，程序保留录制结果，并以状态码 2 退出。

`rm_record_navigation` 通过 OpenCV 合成画面，并用进程管道调用 FFmpeg。它输出 H.264 MP4、最终 PNG 和验证数据。运行前须能在命令搜索路径中找到 `ffmpeg`。`--case`、`--yaw-rate`、`--playback` 和 `--seconds` 选择物理场景及录制参数。只有实际到达误差小于 0.2 m，且没有 MuJoCo 警告时，录制任务才成功。

GLFW 驾驶窗口使用保持式按键。W/S 前进或后退，A/D 侧移，Q/E 旋转。松开按键不会自动停止。按空格停车，按 P 启动演示。鼠标控制相机旋转和缩放。原生命令的 `--duration` 限制窗口运行时间。窗口需要可用的桌面显示环境。

## 测试范围

算法测试覆盖 PID 抗饱和、全向平移与偏航、方向相关功率、执行器预算分配、制动尾段、扫掠拐角、脱困净空、扫描障碍绕行、地图边界和超速窗口。定位测试覆盖独立场景 ICP、平面约束下的打滑拒绝、在线建图、被拒绝或稀疏的扫描，以及原始 CAD 地形路径。

物理测试检查双雷达返回与外参、通过接触产生的侧移及移动旋转、定位误差、功率和故障停车。模型测试比较重新生成模型与原模型的拓扑、质量、关节轴和静置接触。

数据包测试覆盖二进制格式样本和不能表示的数据拒绝。现有 `rosbags` 测试依赖也独立解码过输出。原生随机数生成器与 NumPy 不同，因此不要求每个采样点完全相同。

## 迁移验证记录（2026-10-08）

七条旧版预设路线均通过原生接触物理和先验定位完成。全部路线没有 MuJoCo 警告。120 W 执行器预算仅存在浮点舍入量级的偏差。

| 预设目标 | 仿真耗时（s） | 最终真值水平误差（m） |
| --- | ---: | ---: |
| 平地 | 4.1 | 0.1242 |
| 红方高地 | 14.9 | 0.1196 |
| 红方公路高地 | 23.9 | 0.1153 |
| 中央起伏区 | 19.0 | 0.1089 |
| 蓝方高地 | 52.0 | 0.1251 |
| 跨场路线 | 36.6 | 0.1258 |
| 坡道停驻 | 19.7 | 0.1093 |

最大定位误差为 0.02874 m。各路线均不需要原地调头。另一次无先验地图的平地运行也已到达，最终真值水平误差为 0.1414 m，最大定位误差为 0.06131 m。七条路线的完整物理验证使用 `prior` 模式，不能据此推断 `slam` 模式已通过全部路线。

原生 STL 栅格在全部有限高度单元上与旧版地形缓存完全一致。五项原生 CTest 通过，其中包括重新生成模型的命名点和静置测试。RPC 验证覆盖状态、地图、非法命令与数值、暂停、推进，以及可解码的 960×600 JPEG。

一次原生平地录制实际到达目标，并生成 71 帧 H.264 视频，分辨率为 1280×720，帧率为 10 帧/s。一次完整 ROS1 目标录制生成 1,242 条 IMU 消息和 63 条上雷达消息，且通过独立解码。GLFW 窗口在 Xvfb 下完成物理移动测试，警告数为零。
