# C++ 与 Rust 迁移记录

## 当前结构

当前目录是统一源码仓库。
三个仿真模块使用 C++。
Rust 提供网页接口、命令队列、图像快照和原生子进程管理。
当前程序不启动 Python 解释器。

首次启动使用[首页](../README.md)中的 `./rm setup` 和 `./rm`。
本文记录迁移对应关系和验收边界，不替代首次使用指南。

## 文件与入口对应关系

原 Python 文件位于 `legacy/python/`。
下表左列路径以该目录为起点。右列路径以仓库根目录为起点。

| 历史源码 | 当前实现或入口 |
| --- | --- |
| `simulate.py` | `modules/navigation/src/robot.cpp`；程序 `rm_robot` |
| `navigation/{terrain,sensors,slam,control,dwa,power,task}.py` | `modules/navigation/`；程序 `rm_navigation` |
| `navigation/server.py`、`duel/server.py` | `crates/robomaster/`；根入口 `./rm` |
| `scripts/build_model.py`、`scripts/build_navigation.py` | 原生模型生成器 `rm_navigation_build_models` |
| `scripts/record_lio.py`、`navigation/rosbag_export.py` | `rm_record_lio` 导出 ROS1 数据包；导航 `--record` 导出 JSONL |
| `scripts/record_navigation.py`、`scripts/validate_navigation.py` | `rm_record_navigation` 和 `rm_validate_navigation` |
| `duel/{model,physics,vision,classifier,rotor_tracking,task}.py` | `modules/duel/`；程序 `rm_duel` |
| `rubiks_cube.py`、`cube_solver.py` | `modules/cube/`；程序 `rm_cube` |
| `gripper_plan.py`、`dual_gripper_{model,cube}.py` | 原生魔方规划、模型组装和摩擦控制 |
| `record_rubiks_cube.py`、`record_dual_gripper_cube.py` | `rm_cube --record`；保留物理执行，录像排版有所简化 |
| `scripts/convert_cad.py` | 可选 `tools/cad/`；程序 `rm_convert_cad` |
| `integrations/fast_lio/replay.py` | 原生 ROS1 回放管理器 `integrations/fast_lio/replay.cpp` |

旧版单装甲跟踪器已由旋转跟踪器替代。
当前实现保留迁移时最新的图像检测、数字分类和旋转估计流程。
历史测试仍保留其当时的接口。

## 构建与通信

用户构建入口是 `./rm build`。统一验收入口是 `./rm test`。
底层仍使用 CMake、Cargo 和 Node.js 测试工具。
CAD 需要额外 SDK，并使用 `RM_BUILD_CAD=ON` 启用。

原生进程逐行接收 JSON。
请求包含 `command` 和 `args`。
响应包含 `ok`，以及 `result` 或 `error`。
标准输出只传协议。诊断信息写入标准错误。

C++ 线程独占其 MuJoCo 和 EGL 对象。
HTTP 请求体上限为 4 KiB。命令队列容量为 64。
Rust 丢弃过期底盘命令，并传递剩余的 350 ms 有效期。
服务退出时，Rust 终止并回收对应进程。
当前统一入口可同时管理两个网页服务。

## 迁移验收记录

以下结果对应 **2026-10-08 原生迁移验收**，不是每次运行时自动获得的结果。

| 项目 | 当时的实际结果 |
| --- | --- |
| 迁移前 Python 基线 | 190 项测试和 5 项子测试通过 |
| C++ 集成测试 | 12 个套件通过，包含可选 CAD |
| Rust 测试 | 8 项通过 |
| 网页控件测试 | 6 项通过 |
| 真实浏览器 | 两个页面的 Firefox 操作均通过 |
| 导航 | 7 个原有地形场景全部到达目标；另有在线建图平地场景通过 |
| 双夹爪魔方 | 20 步打乱后完成通用求解；434 个优化动作；无禁止接触和 MuJoCo 警告 |
| 数据与录像 | ROS1 数据包可由独立读取器解析；H.264 录像可解码 |
| CAD | 两份 STEP 可转换；坐标边界与基线匹配 |
| 归档后 Python 基线 | 再次通过 190 项测试和 5 项子测试 |

浏览器验收包含图像显示、设置、自动射击、视角、暂停、重置、地图目标和偏航滑块。
通用魔方求解验收包含无历史转动记录的状态。
接触控制验收包含禁用内部面电机的双夹爪执行。
数据包的 Python 读取器只用于验收，不由当前生产程序调用。

本机当时的报告、日志和截图位于 `output/migration/`。
这些文件被 Git 忽略，新取得的仓库可能没有这些历史产物。
复现步骤见[开发与验证](development.md)和各模块说明。

## 归档完整性与中文化

迁移时，78 个历史文件的哈希全部与迁移前一致。
原始记录保留在 `legacy/baseline-sha256.json`。
后续根据用户要求，把历史说明改为中文。
这些文档的字节已改变；原始哈希不再用于证明当前中文文档相同。
历史源程序、测试和依赖锁未因文档中文化而修改。

原始完整快照在本机 `output/migration/python-baseline-20261008.tar.gz`。
它是迁移证据，不是运行前提。
当前中文文档的校验清单单独记录在 `legacy/documentation-sha256.json`。

## 保留的能力边界

FAST-LIO 的 ROS 构建、Docker 回放和估计结果尚未在本机验证。
本机缺少 catkin，Docker 存储也不足。
已通过的是原生进程管理测试，不能据此宣称估计器可运行。

默认导航使用重力对齐的 ICP。
没有增加全局重定位或闭环检测。
赛场仍使用 2.5D 上表面地图。
传感器扫描方式和执行器损耗仍是仿真假设。

夹爪高速配置对应历史版本中的改装机构。
这些速度不是原厂 Robotiq 硬件的实测指标。
不同 OCCT 版本会生成不同三角形数量。
原始运行网格保持不变。录像文字和布局比历史版本简单。
