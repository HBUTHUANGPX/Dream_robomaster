# 魔方碰撞诊断资产

## 来源与用途

`retention_before_r.json` 来自2026年10月11日本仓库运行的 `optimized-six-solve`。
它保存首次R转动前的完整积分状态、控制目标、逻辑朝向和参数。
`retention_before_f2.json` 来自同日选择性分层的六步流程，保存还原第4步F2脱块前的状态。
`turn_before_r.json` 来自历史完整二十步流程首次R前的状态，供各几何方案进行普通转层计时对照。
`rim_before_f2.json` 来自承托环六步流程的还原F2前，供中心轴向位移及软限位诊断。
该计时快照的几何元数据已补为三层合并基准；状态数组和转动目标没有修改。
本资产由仿真程序生成，不包含RX网格；RX资产来源与许可证见[资产清单](../rx_gripper/manifest.json)。

该快照用于独立比较三层卡脚与选择性简化卡脚，不作为完整连续还原的起点或录像续录点。
快照中的参数描述采集时的配置；实际碰撞几何由传入的场景XML决定。
不同场景必须保持关节、执行器和状态布局一致。状态长度不匹配时停止，不补齐或截断状态。

| 文件 | 字节数 | SHA256 |
| --- | ---: | --- |
| `retention_before_r.json` | 14227 | `837e44de3c89e73cb2fa2d139f60d33e56db3b1e4070b0dba1c776f0972059c5` |
| `retention_before_f2.json` | 14375 | `6aa9b92ecb89b30b37e1a2e93f4266b809e2ee22907aaa5efce49d6b6026ac83` |
| `turn_before_r.json` | 13137 | `2ffa6d41f3d6be2f70a5db4651d7da768f720d8d4225ffbc0fcb01902effc110` |
| `rim_before_f2.json` | 14473 | `4c70eb6d529b623ee0eba72482a80a935e5f1a2b5a89ef8d65ef7460808d9e0c` |

## 复现

在仓库根目录按[准备说明](../../docs/mechanical-rx.md#准备与运行)安装依赖并校验RX资产。
生成选择性简化场景，空序列只执行初始夹持和读取检查：

```bash
./rm cube --mechanical \
  --rx-bundle assets/rx_gripper/mujoco_linkage_v5 \
  --mechanical-config modules/cube/configs/mechanical-rx-selective-rings-experimental.json \
  --scramble "" --output output/cube-mechanical/retention-scene
```

按[独立复现工具说明](../../docs/mechanical-rx.md#单步故障复现)构建诊断程序后执行：

```bash
build/bin/rm_cube_mechanical_replay \
  output/cube-mechanical/retention-scene/scene.xml \
  assets/mechanical_cube/retention_before_r.json \
  output/cube-mechanical/retention-check
```

输出目录已存在时换用新目录。退出码0且 `pose_passed:true` 仅代表单步位姿通过。
要重现原几何失败，以 `mechanical-rx-optimized.json` 生成另一份场景，使用同一快照诊断。
比较F2脱块时，使用 `retention_before_f2.json`；其原几何配置为 `mechanical-rx-selective-rings-experimental.json`。
比较普通R的性能时，改用 `turn_before_r.json`，只比较 `speed_comparable:true` 的运行。
诊断承托环自身的F2失败时，以 `mechanical-rx-rim-experimental.json` 生成场景，使用 `rim_before_f2.json`。
原几何此次单步诊断退出非零，是已知失败；不要把失败提前终止或脱块后的计时用于速度比较。
当前诊断工具每100步检查每块的径向脱出，早停时记录 `escaped_piece` 和 `stop_reason`。
历史版本没有逐块检查，可能在脱块后仍继续积分；其计时不能用于性能比较。
同时查看 `pose_passed`、`simulated_steps` 和 `speed_comparable`，不能仅凭没有早停判断通过。
测量结果和完整短流程验收见[碰撞实验](../../docs/mechanical-performance.md#减少卡脚径向分层)。
