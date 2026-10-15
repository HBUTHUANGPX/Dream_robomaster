# RX-P2 S 电机扭矩—转速数据

本目录保存用户指定本地资料中的两台 S 电机实测数据及独立参考结果。数据为减速器输出端的转速 rpm 和扭矩 Nm；腕关节直接对应输出轴，查询不再乘传动比。未从 F 电机曲线或其他规格推断 S 电机数据。

## 来源

本次读取位置为 `/home/jerry_huang/HPX_Loco/mondo-ai/md_gmt_lab_6_0/mimic_baseline/source/mimic_baseline/mimic_baseline/robots/`。

| 来源文件 | SHA-256 |
| --- | --- |
| `torque_speed.py` | `f313d24166e554ca2006ea5db067185183687773f734f9e470de360ff58609b3` |
| `rx_p2_actuator_data.py` | `c11d382eac0243790c965134227a7a5b5dd303e50b2ff8e809a2c5626c383cf6` |

数据来自 `RXP2_S_MOTOR_PROFILE`，样机编号分别为 `MNHWM02671000032`、`MNHWM02671000034`，每条曲线13个测量点。`s_motor_measurements.csv` 保留原始点及编号。来源记录不构成新的数据授权或许可证声明。

## 查询规则

1. 每条曲线补入零速锚点，扭矩等于该曲线第一个测量值。
2. 按来源的 PCHIP 端点斜率、加权调和斜率和三次多项式计算，不使用线性插值。
3. 公共测量上限取两曲线终点的较小值309.6 rpm。每0.05 rpm取样，共6193格，含零速与公共终点。
4. 每格先取两曲线的较小扭矩，再取从零速到该格的累积最小值，抑制测量噪声导致的回升。
5. 查询索引为 `ceil(abs(velocity_rad_s) * ((60 / (2*pi)) / 0.05))`。不使用 epsilon 将边界速度降到较低格；浮点格边界遵循该公式。
6. 最后附加零扭矩 sentinel。高于公共网格的索引返回零，不能利用第二台样机额外测到的310.3 rpm范围。
7. NaN及正负无穷抛出异常。巨大但有限的速度归零，避免转换成整数索引时溢出。

零速结果为21.31 Nm。原资料的 `static_effort_multiplier=1.2` 仅用于独立的静态上限，不参与动态 lookup，因此本查询不乘1.2。主程序需要的15 Nm硬上限由调用方取最小值。函数只查询扭矩，不修改速度，也不增加控制力。

## 实现与验证

接口为 `rm::cube::rx_s_torque_limit(double velocity_rad_s)`，声明在 `modules/cube/include/rm/torque_speed.hpp`。运行时只用 C++20 标准库：首次调用构建固定数组，后续查表；不读取 CSV，不调用 Python。

`s_motor_reference.csv` 在开发时离线生成：直接加载上述 `torque_speed.py`，以 Python AST 读取 `RXP2_S_MOTOR_PROFILE` 的两条原始曲线，再调用 `build_conservative_torque_speed_lookup(curves, speed_step_rpm=0.05)`，附加零 sentinel，以17位有效数字保存。该表是测试预期值，不由 C++实现反向生成。

集成构建完成后，在仓库根目录运行 `ctest --test-dir build -R '^cube_torque_speed$' --output-on-failure`。预期逐格匹配参考表，并通过零速、低速、交叉段、单调化、正反转、测量终点、超速及非有限输入检查。失败时保留参考表，核对来源哈希、公式和数据，不应通过改写参考结果掩盖差异。

`cube_wrist_tn` 是另一个包装器动力学测试入口；TN查询单测通过不代表完整魔方还原或录像验收通过。

## 全身 F 电机扩展

`f_motor_measurements.csv` 保存同一来源文件 `RXP2_F_MOTOR_PROFILE` 的19个原始点。
`f_motor_reference.csv` 是离线调用同一来源查表函数生成的3799格参考值和零 sentinel。
C++接口 `rx_f_torque_limit` 使用相同 PCHIP、累计最小值和0.05 rpm向上取格规则。
F电机测量终点为189.9 rpm；零速为59.817 Nm，不乘静态倍率。
`cube_torque_speed` 同时逐格检查 F 和 S 曲线。
运行时不调用 Python。

全身控制按来源分组：肩俯仰、髋俯仰、髋侧摆、膝和腰偏航使用 F；其余臂关节和髋偏航使用 S。
踝和腰俯仰、腰侧摆采用来源的双 S 电机等效倍率2。
颈部没有对应测量数据，保留 URDF 静态力矩限制。
所有关节再取 URDF effort 与动态限矩的较小值。
关节参考速度不得超过原始 URDF velocity。
这些是仿真控制限制，不代表已完成真实硬件控制器标定。

F表本次离线生成使用的 `torque_speed.py` SHA-256 为 `6874ae0c8eb7e3f65b0f200fdfd16784fd38ff2ca02420c5dec0cd53813e0e3c`。
`rx_p2_actuator_data.py` 仍为上表所列哈希。
上表中的 `torque_speed.py` 哈希是 S 表导入时的历史记录；S 表保留原参考值，并已重新通过逐格回归。
