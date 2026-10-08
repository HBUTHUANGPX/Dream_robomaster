# 原生统一仓库设计

本文记录 2026-10-08 的迁移设计。
当前启动步骤以[仓库首页](../../../README.md)为准。

## 用户目标

把导航与底盘、自瞄对战、物理魔方三个任务统一到当前目录。
仿真使用 C++。服务与启动器使用 Rust。
保留已有交互、物理行为、模型来源和数据依据。

## 职责划分

| 部分 | 职责 |
| --- | --- |
| C++20 模块 | 仿真、渲染、估计、规划、控制和魔方求解 |
| Rust 服务 | HTTP、有限容量命令投递和 C++ 子进程生命周期 |
| HTML 与 JavaScript | 保留现有浏览器界面 |
| `libs/sim/` | 共用模型生命周期、EGL 渲染、路径和通信协议 |
| `cmake/` 与 `tools/` | 依赖发现、准备和构建 |
| `assets/` 与 `web/` | 保持共享路径与来源记录 |
| `legacy/python/` | 保留历史实现和测试，供对照使用 |

C++ 使用 MuJoCo C 接口、Eigen、OpenCV 和 nlohmann JSON。
当前入口不得调用 Python 解释器或 Python 子进程。
各模块必须可以独立测试。
迁移验证通过后，才把现有服务切换到原生实现。

## 原生进程协议

可执行文件位于 `build/bin/`。
核心文件名是 `rm_navigation`、`rm_duel` 和 `rm_cube`。

参数 `--root 路径 --rpc` 启动逐行 JSON 通信。
标准输出每次写一条响应。诊断信息写入标准错误。

请求示例：

~~~json
{"command":"state","args":{}}
~~~

成功响应包含 `ok: true` 和 `result`。
失败响应包含 `ok: false` 和 `error`。
帧响应使用 `mime: image/jpeg` 和 Base64 图像数据。

共享命令包括 `state`、`tick`、`reset`、`pause` 和 `frame`。
导航额外提供 `map`、`goal` 和 `yaw`。
对战额外提供 `settings`、`drive`、`aim` 和 `fire`。
保留已有浏览器字段和单位。
拒绝格式错误、非有限值和越界参数。

## 共用 C++ 接口

头文件为 `rm/sim.hpp`。CMake 目标为 `rm_sim`。

| 接口 | 职责 |
| --- | --- |
| `rm::Json` | JSON 类型别名 |
| `rm::Simulation(path)` | 管理模型与数据；禁止复制 |
| `rm::Renderer(model, width, height)` | 管理 EGL 和 MuJoCo 场景 |
| `render(...)`、`render_camera(...)` | 返回 RGB 图像 |
| `scene()` | 提供覆盖图形的场景入口 |
| `jpeg_response(...)` | 把图像编码为 JPEG 响应 |
| `repo_root(...)` | 依次使用命令参数、环境变量和构建位置解析根目录 |
| `rpc_loop(...)` | 读取请求、验证对象并封装结果或异常 |

MuJoCo 与渲染对象由同一所属线程操作。
Rust 串行调度请求。

## 验收要求

构建必须覆盖三个 C++ 模块和 Rust 工作区。
行为测试必须覆盖错误输入和正常物理执行。
HTTP 必须支持页面、状态、图像和本地端口转发。

导航验收检查接触驱动的底盘运动、传感器定位和路径控制。
对战验收检查真实 RGB 检测、PnP、跟踪、热量和命中。
魔方验收检查通用求解与物理夹持，不能只反转打乱历史。

未达到的功能必须写出限制。
迁移完成后，保留基线快照、校验值和逐项对应关系。
