# RX ASSY V2 连杆夹爪 · MuJoCo v5

这是可独立加载的 **MJCF 连杆与接触夹具**，含双手原厂网格、按零件拆分的碰撞体、4 个四连杆闭环、4 个空间拉杆闭环、把手及面板。不是整机 URDF，也不是策略成功演示。

- `contact_test.xml`：双爪与抽屉把手/面板测试夹具。
- `free_sweep.xml`：空载开合测试夹具。
- `inspect.py`：安装 MuJoCo、NumPy、SciPy 后，在有桌面的环境运行 `python inspect.py`。
- `gripper_linkage.py`：重置与几何计算；完整重新生成需要工程里的原始 CAD 网格。
- `contact_result.json`、`sweep_result.json`：物理测试原始统计与 50 Hz 数据。

每手唯一主动关节是 `l_gripper_motor_joint` / `r_gripper_motor_joint`，单位 rad，开合端点 −2.25 / 0；夹爪命令 c∈[0,1] 对应 −2.25×(1−c)。原 `finger*_joint` 现在是被动转动关节，不能再当作米单位的滑动关节驱动。闭环由 MuJoCo connect 约束实现，不能直接用普通 URDF mimic 替换。

测试夹具以受控滑台替代机器人手臂，只验收夹持/传力，不计入 WLA/GR00T 成绩。重力为 0，抽屉约束沿滑轨运动。滚动控制器使用 2 N 静摩擦、1.5 N 动摩擦、8 N·s/m 阻尼，无回位弹簧；单独打开 XML 时 frictionloss 保持 2 N。

几何由原厂三角网格推导；45 mm 连杆、35.794 mm 拉杆；motor zero / 转轴分支仍是闭合几何推定，未用实机编码器读数标定。驱动 kp=2、kd=0.1 来自实机配置；±0.6 Nm 力矩上限为仿真假设。材料摩擦和手指柔度仍未实测。
