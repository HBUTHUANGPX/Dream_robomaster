# 魔方碰撞诊断资产

## 来源与用途

`retention_before_r.json` 来自2026年10月11日本仓库运行的 `optimized-six-solve`。
它保存首次R转动前的完整积分状态、控制目标、逻辑朝向和参数。
`retention_before_f2.json` 来自同日选择性分层的六步流程，保存还原第4步F2脱块前的状态。
`turn_before_r.json` 来自历史完整二十步流程首次R前的状态，供各几何方案进行普通转层计时对照。
`rim_before_f2.json` 来自承托环六步流程的还原F2前，供中心轴向位移及软限位诊断。
`rim_before_f2_centered_diagnostic.json` 由该快照派生，只在独立诊断初始化时整体居中。
它对20个自由棱角块和核心施加相同平移，保留内部相对位姿、速度和夹爪状态。
平移量为原核心位置相对 `(0,0,0.1)` 的负偏差，不得用于正式运行的中途纠偏。
`contact_redesign_results.json` 保存本轮实测摘要和场景参数，不依赖历史输出目录才能阅读结果。
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
| `rim_before_f2_centered_diagnostic.json` | 13668 | `960e83d3bd3dd451103757d2a213e3278aa84b084446f863a7c3554e355712de` |

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
偏心对照改用 `rim_before_f2_centered_diagnostic.json`，场景不变；这不是连续还原验收。
圆滑底托、双向倒扣和柔软指垫的配置及判据见[接触结构实验](../../docs/mechanical-contact-redesign.md)。
原几何此次单步诊断退出非零，是已知失败；不要把失败提前终止或脱块后的计时用于速度比较。
当前诊断工具每100步检查每块的径向脱出，早停时记录 `escaped_piece` 和 `stop_reason`。
历史版本没有逐块检查，可能在脱块后仍继续积分；其计时不能用于性能比较。
同时查看 `pose_passed`、`simulated_steps` 和 `speed_comparable`，不能仅凭没有早停判断通过。
测量结果和完整短流程验收见[碰撞实验](../../docs/mechanical-performance.md#减少卡脚径向分层)。

## 关节导向实验

关节模式的运行配置位于 `modules/cube/configs/mechanical-rx-guided-fast.json`。此模式状态维数与旧自由块快照不同，不能混用。

`drift_before_whole_b.json` 是旧接触模式软指垫实验的整块B转动前状态，仅用于历史滑移诊断。来源为 `compliant-tip008-normal-six/diagnostic_states/before_49.json`；SHA-256为 `139e60df25f9892bdf93d1bd132595ecd7ae5ce70f152b36b35b350fcc50678e`。

`joint_guided_results.json` 保存本次关节导向六步打乱及还原实测，包括完整参数、逐检查点误差、实际色块和结果；省去体积较大的规划与夹爪检查明细。原始输出目录仅用于本次追溯，重新运行不依赖这些目录。

`joint_guided_six_demo.mp4` 为本项目生成的六步多面打乱及完整还原录像，正常速度，采用仓库RX资产及关节导向模型；不是外部下载素材。运行命令见[关节导向实验](../../docs/mechanical-rx.md#关节导向实验)。

录像SHA-256：`8fa1a92ede059fe2ab0d1ae4d081c5a5aa9c1c69073dd91beaaa1804864333b3`。共1711帧、30帧/秒，播放时长约57.03秒；最后0.5秒为静置展示。

## 关节导向与腕轴TN：20步完整录像

以下文件由本仓库生成，采用RX资产及新关节导向模型；没有拼接诊断回放。

| 文件 | 内容 |
| --- | --- |
| `joint_guided_tn_20_demo.mp4` | 正常速度的20步打乱、21步求解还原；含中文步骤字幕 |
| `joint_guided_tn_20_report.json` | 完整参数、优化动作计划、41个检查点、实际色块和TN统计 |
| `joint_guided_tn_20_video.json` | 视频时长、打乱结束时刻、动作序列和验收摘要 |

视频为800×600、30帧/秒，共5869帧，约195.634秒。
视频SHA-256为 `5e6321b0ba714cb9d99a1087bfd5b90a42d55ff385101a3b2511345edfd52621`。
报告SHA-256为 `27486fac5d8ce947f7d1ebe4c5a0cf9398cf18d6dc97c9317619f85b1f1a92dc`。

复现入口见[腕轴TN完整录像](../../docs/mechanical-rx.md#腕轴tn与关节导向完整录像)。
步骤字幕由 `node modules/cube/tools/annotate-mechanical-video.mjs 输出目录` 生成，需先有验收通过的原录像。
报告中的原始输出路径仅用于来源追溯，复现不依赖该目录。

## 魔方质量基准

`mass_reference.json` 保存GAN356 M官方总质量、尺寸、来源和本模型的质量分配假设。
只参考总质量74克；仍使用55毫米仿真几何，各块质心惯性与核心质量按原比例缩放。
真实零部件质量分配未知，不将估计张量描述为厂家实测。

## 快速腕轴完整录像

此节为历史基准；新的无等待实验见下文[无等待完整录像与边界试验](#无等待完整录像与边界试验)。

`wrist_control_20_demo.mp4` 为74克模型20步打乱、21步还原的正常速度录像，2955帧、30帧/秒，约98.5秒。
`wrist_control_20_report.json` 保存完整动作计划、41个检查点、质量惯性和腕轴跟踪统计。
`wrist_control_results.json` 保存候选的成功与失败结果；最快通过本次完整验收的配置为10 rad/s、100 rad/s²。

`wrist_control_20_demo.mp4` SHA-256：`b7e7e933c192f5aea021aa542b167b9ca1a8eca46241139368f76f0ed5f881e2`。

`wrist_control_20_report.json` SHA-256：`7d3e700e51efd20ec4473d06044d3c160cd207368d3071872f960204eb6d0f94`。

复现命令见[快速腕轴控制与质量基准](../../docs/mechanical-rx.md#快速腕轴控制与质量基准)。本项目生成视频，使用仓库已有RX资产，没有拼接独立诊断回放。

## 无等待完整录像与边界试验

`no_wait_20_demo.mp4` 为20步打乱与21步还原的连续录像。1921帧、30帧/秒、800×600，正常速度64.034秒。
动作执行于63.5035秒完成，含初始装配及上料承重检查；随后保留0.5秒展示。
腕转、开合的动作后等待、对齐等待、重抓均关闭。展示前的位置误差0.362毫米，展示后0.352毫米，两次均通过原验收标准。
TN力矩越界量为零，动态边界未低于静态限制，不将此记录作为高速TN收紧验证。

视频SHA-256：`1a3e13dcb6b0c60e26dfa17ea43080a78d715716db62a19b9da301338f390f17`。
`no_wait_20_report.json` 保存完整参数、规划、动作末端速度和跟踪误差、实际色块读回及验收结果。
报告SHA-256：`0df80e52e5289ec7fabe990e095f8cde9d3fa8b4c231694aca44a500137ed089`。

`no_wait_cases.json` 保存18项可复测配置；`no_wait_results.json` 保存全部18项结果、初次基准以及最终录像信息。
失败不能作为速度提升，部分序列降低速度仍失败，因此这些记录不证明任意状态下的可靠上限。
本轮集成测试通过25项C++、19项Rust和35项网页与入口测试，另通过800步夹爪诊断回放。

资产由本项目生成，沿用仓库RX几何和74克质量基准的来源说明，没有新增第三方几何或外部视频。
复现条件、命令与失败处理见[无等待执行边界](../../docs/mechanical-rx.md#无等待执行边界)。

## 高腕速失败录像

`wrist_failure_14_demo.mp4` 和 `wrist_failure_16_demo.mp4` 为同一20步打乱的失败重录。
前段正常速度连续播放至保护停止，后段重复最后3秒，放大并以0.25倍播放，最后定格2秒。
慢放和定格均为后期处理，没有继续推进失败后的仿真，也不表示成功还原。

14 rad/s、140 rad/s²：第16步D2后，核心位移约3.31毫米，超过2毫米过程验收限制。
末端腕角误差约0.00229弧度，低于0.006弧度腕轴验收限制。
16 rad/s、160 rad/s²：第14步U2转动中，核心位移约5.17毫米，触发失去支撑保护。
两次录像的首次失败时间、错误和物理状态均复现了此前无录像结果。
直接检测到的是核心滑移；这组试验同时提高速度和加速度，不能据此单独确定两者各自的影响，亦未证明接触力已达到摩擦极限。

对应的 `wrist_failure_14_report.json`、`wrist_failure_16_report.json` 保存完整失败报告。
`wrist_failure_videos.json` 保存来源提交、原始时长、后期处理命令及视频、报告的SHA-256。
资产由本项目生成，沿用前述RX几何和质量来源，没有引入第三方视频。
程序及几何未修改；本次验证为两次预期失败复现、视频解码和字幕检查，未重复代码集成测试。

后续16组同状态干预回放见[失败原因分析](../../docs/mechanical-wrist-failure-analysis.md)。
`wrist_failure_analysis.json` 保留全部参数、原始回放报告、初始指尖接触和采样轨迹；所有回放均未通过严格最终姿态验收。
