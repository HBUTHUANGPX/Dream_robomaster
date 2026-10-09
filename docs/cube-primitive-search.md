# 双夹爪十二元动作搜索

## 使用条件与入口

先按[首次使用指南](getting-started.md#准备环境)完成 `./rm setup`。
需要完整源码、模型和仓库附带的 Kociemba 表。命令均在仓库根目录执行。
无窗口规划不需要图形桌面；录像还需要 FFmpeg 和可用的 OpenGL/EGL 驱动。

双夹爪的无窗口 `--solve`、RPC `solve` 和 `plan` 默认使用十二元动作搜索。
命令行搜索选项要求 `--dual`，不能与 `--viewer` 同用。
未启用双夹爪时，仍使用面转动求解。窗口操作见[模块说明](modules/cube.md)。

先只规划，不执行还原：

```bash
./rm cube --headless --dual --scramble "R U F' L2" \
  --wrist-speed 8 --jaw-speed 32 --plan-only \
  --search-ms 1000 --objective execution_time --terminal-policy stable \
  --search-memory-mb 64 --output output/cube-search-plan
```

`--plan-only` 自动启用求解，但仍创建模型并执行打乱。
它不执行夹爪还原，不能用本次 `verification.json` 证明计划已经物理还原。
成功时检查 `output/cube-search-plan/robot_plan.json` 的 `found` 为 `true`。
预算内没有解时，命令失败退出；按[搜索失败处理](troubleshooting.md#双夹爪搜索失败或结果不符预期)处理。

需要执行还原时，运行：

```bash
./rm cube --headless --dual --scramble "R U F' L2" --solve \
  --wrist-speed 8 --jaw-speed 32 --search-ms 1000 \
  --objective execution_time --terminal-policy stable --search-memory-mb 64 \
  --output output/cube-search-execute
```

命令结束后检查退出状态和 `verification.json`，不要仅凭 `found` 判断物理执行成功。
其中 `primitive_execution_s` 是本次模型中已执行元动作的实际仿真时长之和，不含装载和搜索时间。
需要录像时追加 `--record --playback 1`，视频写入同一输出目录的 `demo.mp4`。
各次运行建议使用不同输出目录，避免将旧文件误认为本次结果。

## 搜索参数与计时

| 命令行参数 | RPC 参数 | 默认值与含义 |
| --- | --- | --- |
| `--search-ms 1000` | `max_search_ms` | 1000 毫秒；范围为 0 到 3600000 |
| `--objective execution_time` | `objective` | `execution_time` 最小化预计动作执行秒数；`action_count` 最小化元动作数 |
| `--terminal-policy stable` | `terminal_policy` | `stable` 接受图内稳定终态；`home` 要求两腕归零且双爪闭合 |
| `--search-memory-mb 64` | `memory_limit_mb` | 64；必须为 4 到 4096 的整数，仅限制机器人图搜索工作内存 |
| `--cost-profile PATH` | `cost_profile` | 命令行读取 JSON 文件；RPC 直接传入 JSON 对象 |
| `--plan-only` | `solve` 的 `execute:false`，或 `plan` 命令 | 只返回计划，不执行还原 |

`execution_time` 只累加动作成本，不加搜索耗时。`action_count` 按每个元动作计 1。
动作严格串行，尚不支持重叠执行（overlap）。RPC 不接受 `overlap` 参数。

搜索期限包含首解生成、求解表读取与校验、图转换表准备和候选优化。
它不是“先无限等首解，再开始计时”。未还原状态使用 0 毫秒预算时返回 `found:false`，不会强行调用旧求解流程兜底。
期限通过检查点控制，不应把预算当成进程总耗时的硬上限。
程序冷启动、模型创建和搜索前的物理打乱不在搜索预算内。

元动作计价从已夹持的起态开始。初始化装载与建立抓握的时间不计入 `estimated_execution_s`。
`memory_limit_mb` 不限制整个进程的常驻内存（RSS），也不涵盖 MuJoCo 模型或 Kociemba 表等全部内存。

## 动作、机械域与终态

| 夹爪 | 六个动作 |
| --- | --- |
| A | `A_P90`、`A_N90`、`A_P180`、`A_N180`、`A_OPEN`、`A_CLOSE` |
| B | `B_P90`、`B_N90`、`B_P180`、`B_N180`、`B_OPEN`、`B_CLOSE` |

A 的正轴为 +X，B 的正轴为 -Y。`P` 和 `N` 表示绕各自正轴的正、负转角。
旋转动作是相对当前腕角的增量，不是绝对目标角。
正负半转在魔方色块上的效果可相同，但腕角终态和后续可行动作可能不同。

图状态包含 24 种整体朝向、两腕角度和夹持状态。
腕角以 quarter 表示，每个 quarter 为 90°，每腕范围为 -2 到 +2。
至少一侧必须闭合支撑。两腕同时为奇数 quarter 的状态禁止。
旋转任一腕时，另一腕必须处于偶数 quarter，即 0° 或 ±180°；另一爪张开也不能免除此限制。
这是依据物理探针采用的保守机械域，不代表已经搜索所有物理可行轨迹。

两爪闭合时旋转为单层转动 `face`；仅旋转侧闭合时为整块转动 `whole`；旋转侧张开时为空转 `empty`。
开合使用 `jaw` 模式。执行器仍检查间隙、支撑、层对齐、腕部跟踪和数值异常。

默认 `stable` 允许非零末腕角和单侧支撑，但必须满足图内约束。
`home` 要求两腕为 0 且双爪闭合，整体朝向仍可为任意一种朝向。
需要固定摆放方向时，不要把 `home` 理解为回到初始整体朝向。

## 自定义动作成本

在仓库根目录创建示例成本文件：

```bash
mkdir -p output
cat > output/cube-costs.json <<'JSON'
{
  "duration_s": {
    "A_P90": 0.2,
    "B_P90": 0.2
  },
  "mode_duration_s": {
    "A_P90": {"face": 0.4, "whole": 0.3, "empty": 0.2},
    "A_CLOSE": {"jaw": 0.03}
  }
}
JSON
```

`duration_s` 设置指定动作的各模式成本；`mode_duration_s` 覆盖对应模式。
未指定的动作或模式保留默认成本。旋转只接受 `face`、`whole`、`empty`；开合只接受 `jaw`。
数值单位为秒，须为 0 到 3600 的有限数。JSON 小数必须写为 `0.2`，不能写为 `.2`。

使用该文件规划：

```bash
./rm cube --headless --dual --scramble "R U" --plan-only \
  --wrist-speed 8 --jaw-speed 32 --cost-profile output/cube-costs.json \
  --search-ms 1000 --output output/cube-search-costs
```

预计时长不等于执行器控制速度。成本文件用于比较计划，不改变控制器运动时长。
实际控制速度由 `--wrist-speed`、`--jaw-speed` 等参数设置；默认成本会参考这些配置。
成本估计也不是实际墙钟耗时或硬件性能承诺。

## RPC 示例

先完成环境准备。在仓库根目录执行以下命令，打乱后只规划：

```bash
mkdir -p output
./rm cube --rpc --dual --wrist-speed 8 --jaw-speed 32 2> output/cube-search-rpc.log <<'JSON'
{"command":"scramble","args":{"moves":"R U"}}
{"command":"plan","args":{"max_search_ms":1000,"objective":"execution_time","terminal_policy":"stable","memory_limit_mb":64,"cost_profile":{"duration_s":{"A_P90":0.2},"mode_duration_s":{"A_P90":{"face":0.4,"whole":0.3,"empty":0.2},"A_CLOSE":{"jaw":0.03}}}}}
JSON
```

标准输出应有两行 JSON。先检查每行 `ok`，再检查规划结果的 `found`。
`ok:true` 仅表示请求处理成功，不保证预算内找到解。
输入结束后进程退出。RPC 直接返回结果，不自动生成命令行模式的计划文件。
需要执行时使用 `solve`，默认 `execute:true`；仅规划可传 `execute:false`。
请求可以覆盖启动命令指定的搜索默认参数。`cost_profile` 是内联对象，不是路径字符串。

复用计划时，执行入口是 `gripper` 命令的 `action:"execute_primitives"`。
`actions` 必须是元动作名称字符串数组，不能直接传规划结果中的动作对象数组。
例如，请求结构为 `{"command":"gripper","args":{"action":"execute_primitives","actions":["A_OPEN","A_CLOSE"]}}`。
这只是字段示例，不是还原序列。

执行前必须在同一进程初始化抓握，并确认色块、朝向、腕角和夹持状态仍与计划起态一致。
从规划结果的每个动作对象提取 `action` 字段，保持顺序。
优先使用 `solve` 完成规划与执行；不要拼接任意动作来绕过图约束。
执行器会先校验完整动作序列，再执行物理动作，但不会替客户端绑定旧计划的色块快照。
外部 `facelets` 可用于规划；与物理魔方不一致时，`solve` 拒绝执行。
错误时检查响应的 `error` 和标准错误日志，物理失败后用 `reset` 重建场景，再重新规划。

## 输出与最优性边界

命令行成功找到解时，`robot_plan.json` 和 `plan.json` 中的 `actions` 都是十二元动作对象数组。
每个对象包括 `action`、`mode`、`move`、`duration_s`、`start_s`、`end_s`、`before` 和 `after`。
起止时间是串行预计时间，不是执行采样记录。

| 字段 | 读取方式 |
| --- | --- |
| `found` | 是否找到并通过软件重放检查的计划 |
| `estimated_execution_s` | 元动作预计执行秒数，仅找到解时提供 |
| `search_ms` | 本次搜索实际耗时，单位为毫秒 |
| `action_count` | 十二元动作总数，仅找到解时提供 |
| `improvements` | 每次更优解的发现时间、动作数和 `execution_s` |
| `stop_reason` | `deadline` 为到达期限；`memory_limit` 为图工作内存受限；`candidate_search_finished` 为候选搜索结束 |
| `global_optimal` | 固定为 `false`，不宣称全局最优 |
| `fixed_sequence_optimal` | 是否已经证明返回方案在该固定面序列和机械域内最优 |

没有解时不要读取不存在的 `actions` 或预计时长。
命令行先写 `robot_plan.json`，随后失败退出并写 `failure.json`；此时不保证生成新的 `plan.json`。
有解后再到达期限或内存上限时，可返回已找到的最佳计划；仍须检查后续物理执行结果。
`steps.csv` 提供逐动作明细。`plan_unoptimized.json` 是旧编译格式的参考输出，不是新搜索的执行入口。

Kociemba 产生候选面序列。对每个固定面序列，Dijkstra 在上述有限机器人图中求最短路径，允许将半转拆为两个四分之一转。
完成该固定序列搜索时，最优性只相对于给定起态、动作成本、终态策略和机械域成立。
若终点已生成但尚未出队，期限或内存限制到达时仍保留可行方案，`fixed_sequence_optimal` 为 `false`。
超时未完成的候选不享有该结论。整个还原问题只返回预算内已找到的最佳解（best found）。
它不证明所有面序列中的全局最优，也不证明保守机械域之外或支持动作重叠时的最优性。

候选面序列和元动作对应的面转动都经过完整的软件色块重放检查。
搜索不会对每个候选运行 MuJoCo。软件验证通过不等于物理执行通过。
2026-10-09 本次验收：20 步打乱、腕部 8 倍、夹爪 32 倍、搜索预算 3000 毫秒，得到 60 个元动作并完成物理还原。
预计执行 13.419 秒，实际仿真执行 13.368 秒。禁触和数值警告均为零，内部面电机力为零。
记录位于 `output/primitive-search-design/full-solve-final/`。这是单个场景的本次结果，不代表全局最优或所有场景的性能保证。

## 表文件与失败处理

Kociemba 使用 `modules/cube/third_party/kociemba/cprunetables/` 中随仓库提供的表，并校验长度与内容。
缺表、坏表或无法读取会报错，不会现场生成替代表。
从同一仓库版本恢复报错路径的文件，并确认读取权限后重试；不要关闭校验。

新的有限状态转换表在进程内准备和缓存，不需要离线生成或额外下载。
首次准备会消耗搜索预算。完整排错见[双夹爪搜索故障处理](troubleshooting.md#双夹爪搜索失败或结果不符预期)。
