# RX 全身固定基座模型

## 来源与许可

来源是用户提供的 `RX_ASSY_V2_with_gripper.zip`。
原包未提供明确许可证，保留原权利归属。
此转换不授予额外的分发或商用许可。
原包哈希、36 个原始 STL 和原始 URDF 的哈希见 [manifest.json](manifest.json)。
`files` 只记录来源文件；`generated_files` 单独记录生成文件。
转换不修改来源文件字节。

## 模型入口

[robot.xml](robot.xml) 是独立可加载的 MJCF。
移动模型时，复制本目录，并保留 `meshes/` 相对位置。
模型固定 `base_link`，不包含自由关节或虚拟关节。
模型保留 29 个原生转动关节和范围。
两侧原滑动夹爪、手指及其连接关节已在 URDF 编译前剔除。
`l_wrist_pitch_link` 和 `r_wrist_pitch_link` 保留，供父场景挂载现有四连杆。

原始机身的 30 份 visual、30 份 collision 和显式惯性均保留。
MuJoCo 对碰撞网格使用凸包；这不等价于原 CAD 的凹面碰撞。
`base_link.STL` 与 `waist_pitch_link.STL` 超过 STL 解码器的 20 万面限制。
它们逐面转存至 `meshes/generated/*.msh`，没有删面或简化。
原始 STL 仍保留。MSH 仅对 float32 位模式完全相同的坐标建立共享索引。
逐面还原坐标字节与原 STL 一致，三角面顺序和绕序不变。
未使用的 6 份夹爪网格仍作为来源保留，但模型不引用它们。

格式已核对 [MuJoCo 官方 MSH 说明](https://mujoco.readthedocs.io/en/stable/XMLreference.html#asset-mesh)。
MSH 使用小端 32 位整数头 `(nvertex, 0, 0, nface)`，随后为 float32 顶点数组和 int32 三角面索引数组。
文件长度为 `16 + 12*nvertex + 12*nface` 字节。
没有纹理数据；法线由 MuJoCo 重建，与 STL 导入方式一致。
原包 STL 中的面法线和属性字节仍可从未修改的来源文件追溯。

局部缺省类为 `rx_full_body`，不向主缺省类写参数。
关节转子惯量 `armature=0.01 kg·m²`。
每个位置执行器命名为 `关节名_drive`。
控制范围采用原始关节范围，单位为 rad。
力矩范围为原始 URDF `effort` 的正负值，单位为 N·m。
初始 `kp=5×effort`，单位为 N·m/rad；`kv=0.2×effort`，单位为 N·m·s/rad。
这些增益仅用于实验控制，尚未完成全身动作、稳定性或硬件控制标定。
URDF 的速度上限没有转换为速度约束，调用方需要规划速度。

零位检查发现局部装配凸包相交。
模型只为以下 8 对机身添加原生 `exclude`，其余碰撞继续有效：

- `base_link` 与 `waist_yaw_link`、`waist_roll_link`、`waist_pitch_link`。
- `base_link` 与 `r_hip_pitch_link`、`l_hip_pitch_link`。
- `waist_yaw_link` 与 `waist_pitch_link`。
- `r_knee_link` 与 `r_ankle_roll_link`。
- `l_knee_link` 与 `l_ankle_roll_link`。

## 生成方式

条件：Linux、支持 C++20 的 `g++`、`pkg-config` 和 libxml2 开发包。
仓库 `.deps/mujoco/include` 与 `.deps/mujoco/lib` 必须存在。
本次使用 MuJoCo 3.15.0。
Ubuntu 缺少系统编译工具时，执行 `sudo apt-get install g++ pkg-config libxml2-dev`。
缺少仓库 MuJoCo 时，在仓库根目录先执行 `./rm setup`。
不需要 Python，也不修改 CMake。

1. 在仓库根目录创建临时目录。

   ```bash
   mkdir -p .cache/tmp
   ```

2. 编译导入器。

   ```bash
   g++ -std=c++20 -Wall -Wextra -Werror modules/cube/tools/import-rx-full-body.cpp $(pkg-config --cflags --libs libxml-2.0) -I.deps/mujoco/include -L.deps/mujoco/lib -Wl,-rpath,"$PWD/.deps/mujoco/lib" -lmujoco -o .cache/tmp/import-rx-full-body
   ```

3. 生成并验证模型。

   ```bash
   .cache/tmp/import-rx-full-body assets/rx_full_body .cache/tmp/rx-full-body
   ```

预期退出码为 0，打印 29 个关节、29 个执行器和总质量。
完整列表及本次验证结果见 [validation.txt](validation.txt)。
程序验证关节范围、执行器力矩范围、机身质量、碰撞数量、腕部末端和夹爪删除情况。
程序复制模型及网格到临时目录后重新加载，并执行 100 步有限值及警告检查。
临时 URDF 明确设置 `discardvisual=false`、`fusestatic=false` 和 `meshdir`。
程序使用 libxml2 修改临时副本，由 MuJoCo 编译 URDF 后导出 MJCF。

失败时停止使用新生成的模型。根据标准错误检查输入文件、依赖版本和网格路径，然后重新生成。
输出可能已被覆盖；失败输出不能当作验收通过的模型。
命令在完成后自行退出，不启动服务。
更新生成文件后，在仓库根目录执行以下命令校验来源并刷新生成哈希；需要 Node.js 18 或更新版本。

```bash
node <<'JS'
const fs = require('node:fs');
const crypto = require('node:crypto');
const root = 'assets/rx_full_body/';
const manifest = JSON.parse(fs.readFileSync(root + 'manifest.json', 'utf8'));
const hash = p => crypto.createHash('sha256').update(fs.readFileSync(root + p)).digest('hex');
for (const [p, expected] of Object.entries(manifest.files)) {
  if (hash(p) !== expected) throw new Error('来源哈希不符：' + p);
}
const walk = dir => fs.readdirSync(root + dir, {withFileTypes: true}).flatMap(e => {
  const p = dir + e.name;
  return e.isDirectory() ? walk(p + '/') : [p];
});
const generated = walk('').filter(p => p !== 'manifest.json' && !(p in manifest.files)).sort();
manifest.generated_files = Object.fromEntries(generated.map(p => [p, hash(p)]));
fs.writeFileSync(root + 'manifest.json', JSON.stringify(manifest, null, 2) + '\n');
JS
```

独立导入器只验证模型导入和短步进。完整控制器及集成验收见 [动态验收记录](validation/README.md)。

## 当前单向抓持候选与运动范围

当前采用 [reachability.json](reachability.json) 中的原单向候选。
A 使用右臂，B 使用左臂。A 的零位工具朝向绕自身 x 轴翻转 π，B 不翻转。
夹爪相对原腕 pitch 的固定安装变换不变，不增加腕轴。
基座位置为 `[0.2242328671, -0.1700890261, -0.1424096949]` m。
基座 yaw 为 `2.2105342284` rad。
腰 roll、pitch、yaw 分别为 `[-0.0672509462, -0.1007208600, 0]` rad。

规划器只使用以下原机关节实现的工具状态：

| 手 | 全局旋转轴 | 已验证工具角度 | 离散状态 |
| --- | --- | --- | --- |
| A（右臂） | `+x` | `[-π/2, 0]` rad | `[-1, 0]` |
| B（左臂） | `-y` | `[0, π/2]` rad | `[0, 1]` |

状态 `-1`、`0`、`1` 分别表示工具角度 `-π/2`、`0`、`π/2`。
这些是工具姿态状态，不是某一个原生关节的角度。
同一初始逆运动学分支的反向路径，A 在约 `+70°`、B 在约 `-63°` 触及肩 yaw 限位。
规划器不能将这些反向路径视为可用动作。
回零、松爪和换手顺序由动作规划器验证。

### 本次完整场景运动学验证

历史记录：早期检查使用生成的 `initial2/scene.xml`；当前复现入口见下文。
完整场景包含原机身、实际四连杆夹爪和魔方。
离线检查工具持久化在 [rx-full-body-check](../../modules/cube/tools/rx-full-body-check/README.md)。
逐姿态记录保存在 [validation/sweep.json](validation/sweep.json)。
生成场景只作为检查输入，不作为仓库关键资产；可用下述命令重新生成。
场景和工具的 SHA-256 记录在 `reachability.json`。

本次使用 MuJoCo 3.15.0，共检查 872 个静态姿态：

- A 从零到 `-90°`、B 从零到 `+90°`，每步 `1°`，对侧腕保持零位。
- 每条路径检查双夹爪开/闭端点的四种组合。
- 在腕角 `0°`、`90°` 端点，将每只夹爪的开闭行程分成十等份，检查九个内部位置；另一只夹爪分别保持开或闭。
- 夹爪电机开位为 `-2.25` rad，闭位为 `-1.20` rad。被动连杆按生产初始化算法求解。

这 872 个姿态未检测到超过 `10 μm` 的机身自碰撞、机身与夹爪碰撞或双夹爪碰撞。
腰 pitch 机身与夹爪的最小几何间距为 `98.98 mm`。
工具位置误差最大约 `3.57×10⁻⁸ m`，姿态误差最大约 `3.64×10⁻⁷ rad`。
双臂关节最小限位余量为 `0.1160 rad`。
逐度关节路径和验证摘要见 `reachability.json`。

两手同时偏转不在上述通过范围内。
此前双夹爪开位的 `5°` 双臂角度组合检查中，361 个组合有 79 个发生手指碰撞，最大穿透约 `8.05 mm`。
不得将“单手转动、对侧零位通过”推广为“任意双手同步转动通过”。

该工具不调用 `mj_step`。
它记录夹爪与魔方的接触，但不把预期夹持接触作为机身避障失败。
有限角度采样不能证明采样点之间绝对无碰撞，也不能证明摩擦夹持、控制跟踪或整套动作成功。
独立动态验收中，R 打乱和还原通过，执行时间为27.162秒。
报告与视频见 [R 验收](validation/r_turn_restore.json)、[R 视频](validation/r_turn_restore.mp4)。
U 打乱和还原也通过，包含换手和整块转动，执行时间为39.943秒。
对应记录见 [U 验收](validation/u_turn_restore.json)、[U 视频](validation/u_turn_restore.mp4)。
两次报告的 `passed` 和 `solved` 均为 `true`。
动态验证边界见 [验收说明](validation/README.md)。

### 复现完整场景检查

条件：在 Linux 仓库根目录执行，完成 `./rm setup`。
编译离线工具需要 C++20、Eigen 和 nlohmann-json 开发头文件。
Ubuntu 缺少编译依赖时执行 `sudo apt-get install g++ libeigen3-dev nlohmann-json3-dev`。
MuJoCo 头文件和动态库必须位于 `.deps/mujoco/include` 与 `.deps/mujoco/lib`。
不需要旧的缓存、场景快照或 Python。

1. 使用当前整机入口生成场景。空打乱仅执行初始抓持检查，不录制视频。

   ```bash
   ./rm cube --mechanical \
     --rx-bundle assets/rx_gripper/mujoco_linkage_v5 \
     --rx-full-body assets/rx_full_body \
     --mechanical-config modules/cube/configs/mechanical-rx-full-body.json \
     --scramble "" --output output/cube-mechanical/rx-full-body-check
   ```

2. 在仓库根目录编译持久化离线工具。

   ```bash
   g++ -O2 -DNDEBUG -std=c++20 -I/usr/include/eigen3 modules/cube/tools/rx-full-body-check/sweep.cpp modules/cube/tools/rx-full-body-check/gripper-poses.cpp -I.deps/mujoco/include -L.deps/mujoco/lib -Wl,-rpath,"$PWD/.deps/mujoco/lib" -lmujoco -o output/cube-mechanical/rx-full-body-check/check
   ```

3. 将新生成的场景、已保存的候选和结果路径传入工具。

   ```bash
   output/cube-mechanical/rx-full-body-check/check output/cube-mechanical/rx-full-body-check/scene.xml assets/rx_full_body/reachability.json output/cube-mechanical/rx-full-body-check/sweep.json
   ```

预期退出码为 0，打印 `passed=true`、`cases=872`、`robot_gripper_collision_cases=0`。
工具将结果和逐度关节路径写入第三个参数指定的 JSON 文件。
退出码 1 表示输入、载入或路径求解失败，退出码 2 表示碰撞或误差检查未通过。
失败时保留报告，检查场景版本、候选姿态和网格路径，不沿用历史通过结论。
程序完成后自行退出，不启动服务。
场景中的绝对网格路径由本机入口生成，移动仓库后需重新生成场景。
候选支持范围、闭环初始化公式来源和验证边界见工具目录说明。

## 当前完整动态演示

快速配置已通过20步打乱与求解器生成的20步还原，执行时间214.198秒。
工具峰值2 rad/s，90°约1.473秒；夹爪开合0.25秒，没有额外动作间等待。
见 [完整视频](validation/full20_fast.mp4)、[物理报告](validation/full20_fast.json) 和 [验收条件](validation/README.md#当前完整打乱与还原)。
固定骨盆，仍使用原机关节执行器和摩擦夹持。
