# 物理魔方与双夹爪模块

技术名称和缩写见[术语表](../glossary.md)。

`rm_cube` 实现魔方 XML 生成、MuJoCo 推进、物理色块读取、通用求解、十二元动作搜索、接触反馈、渲染和 FFmpeg 录制。运行时不启动 Python 解释器。

双夹爪无窗口 `solve` 和 RPC `solve`、`plan` 默认使用新搜索。
纯软件求解默认最多三个线程，使用 `--search-threads 1|2|3` 设置。MuJoCo 操作仍留在创建线程。
完整参数、成本配置、机械域与最优性边界见[十二元动作搜索指南](../cube-primitive-search.md)。

RX 夹爪的窄指尖设计及独立单层诊断见[窄指尖验证指南](../rx-narrow-tip.md)。该诊断尚未接入正式还原入口。

## 启动与运行

首次使用时，请先阅读[根目录说明](../../README.md)和[入门说明](../getting-started.md)。下列命令均在仓库根目录运行。先完成 `./rm setup`。

```bash
./rm cube
```

无参数时，该命令打开原生窗口。窗口需要可用的桌面显示环境。附加参数会传给原生程序。魔方不提供网页服务。

以下命令分别执行无窗口求解、双夹爪录制和 RPC：

```bash
./rm cube --headless --scramble "R U F' L2" --solve
./rm cube --headless --dual --scramble "R U F' L2" --solve \
  --wrist-speed 8 --jaw-speed 32 --record --playback 1 \
  --search-ms 1000 --objective execution_time --terminal-policy stable \
  --search-memory-mb 64 \
  --output output/native-cube
./rm cube --rpc --dual
```

`--headless` 表示无窗口运行。`--record` 调用已安装的 FFmpeg，生成 H.264 视频。运行前须能在命令搜索路径中找到 `ffmpeg`。播放速度与运动速度分别设置。

`--export-xml 路径` 导出生成的场景。Menagerie 网格使用绝对路径，因此移动仓库后须重新导出。`--restore` 是通用求解的别名。

只规划时将 `--solve` 换为 `--plan-only`，并省略录像参数。
命令仍创建模型并打乱，不执行还原。找到解时检查 `robot_plan.json` 的 `found`。
预算不足时不会强行兜底；按[搜索排错步骤](../troubleshooting.md#双夹爪搜索失败或结果不符预期)处理。

## 物理与求解边界

魔方包含六个中心铰链、二十个自由块和动态切换的内部焊接约束。面电机和夹爪运动均通过 MuJoCo 积分。控制器不通过逐帧改写块体位置来制作动画。只有测得层对齐后，才更新离散槽位。

色块读取使用相对于核心的实测位置和朝向，不依赖移动历史或逻辑缓存。随仓库提供的 `muodov/kociemba` 1.2.1 C 求解器可处理任意合法色块状态。它检查块体奇偶性及朝向，并通过独立的几何贴纸置换重放验证答案。

求解器源码和剪枝表位于 `modules/cube/third_party/kociemba`，使用 GPL-2.0。请阅读该目录的许可证和来源说明。组合后的魔方可执行文件适用 GPL 分发条件。该条件不自动改变仓库内无关模块的许可证。Robotiq 几何及 BSD 声明保存在 `assets/robotiq_2f85` 中。

两个夹爪固定在 +X 和 -Y 方向。夹爪仅有腕部铰链，没有平移关节。夹爪与魔方之间没有焊接或连接约束。两侧抓握均通过双侧受力接触确认后，临时装载夹具才释放。此时六个内部面电机全部停用。

控制器检查实际指尖间隙，要求至少 89.136 mm。每个旋转物理步都检查禁触。层对齐、支撑、腕部跟踪、位姿误差或数值警告检查失败时，程序停止执行。

## 速度设置

`--speed` 保留旧版统一参考速度实验。`--wrist-speed` 和 `--jaw-speed` 分别设置腕部与夹爪速度。只有显式设置 `--jaw-speed > 1` 才启用修改后的平行手指配置。

该配置使用 0.1 ms 物理步长、较小的折算惯量、内部连杆导向、高带宽张开控制和接触触发的力矩限制。它代表修改后的硬件模型，不代表原厂 Robotiq 的额定性能。

十二元动作按计划串行执行，尚不支持动作重叠（overlap）。“平行手指”描述手指几何运动，不表示腕部与夹爪指令同时执行。
成本配置只影响预计执行时长与计划评价，不改变执行器控制速度。

## 原生进程接口

RPC 每行接收一个请求，并返回一个响应。所有状态修改都在同一原生进程线程执行。通用命令为 `state`、`tick`、`reset`、`pause` 和 `frame`。

`frame` 接受 `{"view":"scene"}` 或 `{"view":"closeup"}`，并返回 Base64 编码的 JPEG。

| 命令 | 参数 | 行为 |
| --- | --- | --- |
| `move` | `{"move":"R"}` 或 `{"moves":"R U2"}` | 执行物理面转动；夹爪初始化后改为夹爪计划 |
| `scramble` | `{"moves":"R U F'"}` | 执行指定序列；省略时使用默认演示打乱 |
| `plan` | 可选 `{"facelets":"…","max_search_ms":1000}` | 双夹爪返回十二元动作搜索结果及物理状态，不执行 |
| `solve` | 可选 `{"execute":false}` | 读取实测状态并求解；默认执行 |
| `gripper` | `{"action":"initialize"}` | 建立摩擦抓握，并释放装载夹具 |
| `gripper` | `{"action":"execute","moves":"R U"}` | 执行通过验证的优化计划 |
| `gripper` | `{"action":"execute_primitives","actions":["A_OPEN","A_CLOSE"]}` | 在已初始化且起态一致时执行元动作字符串数组；此处仅为字段示例 |

双夹爪 `solve`、`plan` 接受 `max_search_ms`、`objective`、`terminal_policy`、`memory_limit_mb` 和内联 JSON `cost_profile`。
结果须同时检查 `ok` 和 `found`。输出 `actions` 是对象数组；显式执行时提取各项 `action` 字符串。
不要手工拼接动作来绕过机械域；推荐直接使用 `solve` 规划并执行。

外部色块字符串可单独用于规划。但若状态与物理魔方不同，程序拒绝执行。非法转动、色块或动作返回结构化 RPC 错误。物理操作失败后，可用 `reset` 重建初始场景。

RPC 操作为同步执行。长时间求解执行会阻塞同一进程中的其他请求。
每次转动请求沿用上一请求结束时的魔方朝向。
搜索使用当前整体朝向、腕角和夹持状态。默认 `stable` 不要求末腕角归零。
需要两腕归零且双闭合时使用 `terminal_policy:"home"`，整体朝向仍任意。

下面的命令初始化双夹爪，然后连续执行 `U` 和 `R`：

```bash
./rm cube --rpc --dual --wrist-speed 8 --jaw-speed 32 <<'JSON'
{"command":"gripper","args":{"action":"initialize"}}
{"command":"move","args":{"move":"U"}}
{"command":"move","args":{"move":"R"}}
JSON
```

标准输出应有三行 JSON，且每行的 `ok` 均为 `true`。
构建提示写入标准错误。输入结束后，程序退出。
如果响应中的 `ok` 为 `false`，先查看同一行的 `error`，再按前述方式重建场景。

## 输出与操作

输出包括 `scene.xml`、`plan.json`、`plan_unoptimized.json`、`steps.csv`、`optimization.json`、`verification.json`、`grasp_checks.json`、`motion_checks.json` 和 `controller_profile.json`。启用录制时还生成 `demo.mp4`。执行失败时写入 `failure.json`。

双夹爪搜索另写 `robot_plan.json`。找到解时，该文件与 `plan.json` 的 `actions` 为十二元动作。
结果包含 `estimated_execution_s`、`search_ms`、`action_count`、`improvements`、`stop_reason` 和 `global_optimal:false`。
`plan_unoptimized.json` 保留旧编译格式作参考，不是新搜索执行入口。
没有解时先写搜索结果再失败退出，不保证生成新的其余计划文件。RPC 直接返回 JSON，不自动导出这些文件。

原生视频保留场景运动和状态文字，但未复刻旧版中文侧栏排版与画中画布局。

窗口按键 R/L/U/D/F/B 转动对应面。S 执行指定打乱序列。Z 求解实测状态。鼠标拖动用于环绕或平移，滚轮用于缩放。安全检查失败后停止执行。重新启动会重置场景。

## 验证范围与历史记录

2026-10-09 本次新搜索验收：相同20步打乱、腕部8倍、夹爪32倍、3000毫秒预算，以60个元动作完成还原，实际仿真执行13.368秒。禁触、数值警告及内部面电机力均为零。
本次结果见 `output/primitive-search-design/full-solve-final/`；新搜索的算法边界见下方入口。
当前操作入口见[十二元动作搜索指南](../cube-primitive-search.md)。
以下为旧编译器与独立优化流程的测试范围及历史记录，不是本次新搜索的运行结果。

原生回归覆盖六个物理面方向、四次转动还原、电机停用时拒绝动作、不依赖历史的通用求解、不可能的棱翻转拒绝、全部 24 种朝向，以及优化等价性和幂等性。

双夹爪测试覆盖原始速度与腕部 8 倍、夹爪 32 倍配置。测试还检查没有外部抓握约束、内部电机力为零、间隙与碰撞，以及去除摩擦后的滑落对照。

`./rm test` 还会执行接口回归。
回归覆盖非单位朝向下的连续 `move`、`gripper` 和 `scramble` 请求。
测试将双夹爪的实测色块与面电机执行同一序列的结果比较。
测试还检查实际 `./rm cube --rpc` 的输出流，并执行 Rust 帮助中的无窗口示例。

MuJoCo 3.15.0 验证中，20 步演示打乱在腕部 8 倍、夹爪 32 倍配置下还原全部 54 个色块。独立优化将 510 个动作减至 434 个。该原生运行检查 261,662 个旋转步，禁触、数值警告和内部面电机力均为零。最小间隙为 91.400 mm。最终相对位置误差为 0.000118 mm，朝向误差为 0.0322°。

在恰好半步的时长边界上，C++ 舍入可能与 Python 的偶数舍入相差一个 0.1 ms 物理步。因此不要求两者步数完全相同。本机 C++ 行为测试在 48.56 s 内通过。GLFW 的 R 后 Z 操作通过 Xvfb 验证。原生 MP4 通过完整 FFmpeg 解码。这些结果是仿真验证，不是硬件标定。
