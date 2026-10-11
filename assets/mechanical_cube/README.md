# 魔方碰撞诊断资产

## 来源与用途

`retention_before_r.json` 来自2026年10月11日本仓库运行的 `optimized-six-solve`。
它保存首次R转动前的完整积分状态、控制目标、逻辑朝向和参数。
本资产由仿真程序生成，不包含RX网格；RX资产来源与许可证见[资产清单](../rx_gripper/manifest.json)。

该快照用于独立比较三层卡脚与选择性简化卡脚，不作为完整连续还原的起点或录像续录点。
快照中的参数描述采集时的配置；实际碰撞几何由传入的场景XML决定。
不同场景必须保持关节、执行器和状态布局一致。状态长度不匹配时停止，不补齐或截断状态。

| 文件 | 字节数 | SHA256 |
| --- | ---: | --- |
| `retention_before_r.json` | 14227 | `837e44de3c89e73cb2fa2d139f60d33e56db3b1e4070b0dba1c776f0972059c5` |

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
原几何此次单步诊断退出非零，是已知失败；不要把失败提前终止或脱块后的计时用于速度比较。
诊断工具的 `stopped_early` 不检查每块的径向脱出，必须同时查看 `pose_passed` 和部件误差。
测量结果和完整短流程验收见[碰撞实验](../../docs/mechanical-performance.md#减少卡脚径向分层)。
