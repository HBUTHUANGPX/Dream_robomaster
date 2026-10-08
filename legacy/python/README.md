# RoboMaster 四麦克纳姆轮 MuJoCo 原型

> **历史记录，不用于当前启动。** 首次使用请阅读[根目录说明](../../README.md)和[入门说明](../../docs/getting-started.md)。下文保留 Python 原型的功能、参数和验收记录。文中的“当前”均指历史版本。

## 归档环境

仅在需要比较旧版时准备此环境。
先完成根目录的 `./rm setup`，取得编译库、渲染开发库和 FFmpeg。
以下步骤需要网络、可用的 EGL 驱动和主目录写入权限。
历史验证使用 Python 3.12。依赖版本由本目录的 `uv.lock` 锁定。

如果当前终端不能执行 `uv --version`，按[官方安装说明](https://docs.astral.sh/uv/getting-started/installation/)安装 `uv`。
安装完成后，按安装器提示把其程序目录加入当前终端的 `PATH`。
再次执行 `uv --version`，确认它可以运行。
下方的 `--python 3.12` 会选择或下载相应 Python 版本，无需预先手工安装。
该行为见[官方 Python 管理说明](https://docs.astral.sh/uv/guides/install-python/)。

从仓库根目录进入归档，再创建环境：

```bash
cd legacy/python
uv sync --locked --python 3.12 --group dev --group cube --group lio
```

下文及本目录 `docs/` 中的命令均以 `legacy/python` 为工作目录。`assets`、`web` 和 `output` 链接到仓库共享目录。生成模型或录制会写入共享目录。如需保留现有文件，请先复制相关输出。

桌面窗口需要可用的显示环境。无窗口渲染需要 EGL。
视频录制还需要命令搜索路径中的 FFmpeg 和 Noto CJK 字体。
缺少字体时，在 Ubuntu 中执行 `sudo apt-get install fonts-noto-cjk`，或请管理员安装该包。
网页操作需要浏览器。Node 交互测试需要 Node.js。
当前原生入口不安装历史版本的 Python 依赖。
CAD 转换另需 `cad` 依赖组。该组包含较大的 OpenCASCADE 和 VTK 依赖。

`output/` 中的图像、视频和验证记录是本机历史产物，不保证随仓库提供。迁移时的 SHA256 清单保留原文摘要。文档中文化不改变历史源程序。

按 RoboMaster 官方 2026 资料搭建的**简化步兵底盘**：四驱、每轮 12 个被动 45° 滚子、四面 AM02 小装甲、八个 A 型支撑架和低位保险杠。装甲/支撑架外观直接转换自官方 STEP。轮轴执行器通过真实轮地接触产生运动，没有直接给底盘施力，也没有平面约束。

本机历史预览图路径：`output/preview.png`。

## 启动

完成上述环境准备后，在 `legacy/python` 目录执行：

```bash
uv sync --locked
uv run python simulate.py
```

窗口 需可用的桌面显示。点击仿真窗口后：W/S 前进/后退，A/D 左移/右移，Q/E 左转/右转，空格停止，P 切换循环演示。**指令持续生效，松开按键不会停车**。窗口关闭即退出。鼠标操作为 MuJoCo 默认操作。

无窗口跑完整 12 秒演示：

```bash
uv run python simulate.py --headless --duration 12
```

恒定车体速度（m/s、m/s、rad/s），例如左移：

```bash
uv run python simulate.py --headless --duration 2 --velocity 0 0.4 0
```

静态预览（本机已验证 EGL）：

```bash
MUJOCO_GL=egl uv run python simulate.py --headless --duration .1 --velocity 0 0 0 --snapshot output/preview.png
```

`--duration` 仅限制无窗口运行，窗口 运行至关闭窗口。窗口 默认静止，无窗口默认循环前进、左移、左转、停止，每段 3 秒。

## 文件和参数

- `assets/robot.xml`：可直接由 `mujoco.MjModel.from_xml_path` 加载的完整 MJCF，保留旁边 `meshes/`。
- `scripts/build_model.py`：参数化生成器。修改后运行 `uv run python scripts/build_model.py`。
- `simulate.py`：加载、车体速度转轮速、无窗口/交互演示。
- `assets/official/`：官方 PDF、STEP 和带 SHA256 的 `sources.json`。
- `assets/meshes/`：米制 OBJ 和网格包围盒记录。
- `docs/design.md`：兵种选择、规范条款、安装尺寸与简化边界。
- `output/validation.json`：六方向与静止的实测记录。

| 参数 | 数值 |
| --- | --- |
| 外包络 | 约 570 × 570 × 320 mm |
| 轴距 / 轮距 | 360 / 360 mm |
| 名义轮径 | 152 mm |
| 仿真总质量 | 20.326 kg（含简化装甲与配重） |
| 装甲模块 | 4 × AM02，官方标称 140 × 125 mm、359 g/块 |
| 装甲面法向 | 向外、向上 15° |
| 装甲下缘高度 | 约 188 mm，四面等高 |
| 步长 | 1 ms |
| 驱动限制 | ±35 rad/s，±5 N·m/轮 |

坐标采用 MuJoCo/机器人常见右手系：X 前、Y 左、Z 上。官方制作规范的 Z 轴向下。对应转换为 `(x, -y, -z)`，请在对接官方协议时注意区别。

## 验证及重建

```bash
uv run --group dev python -m pytest -q tests/test_simulation.py
uv run python scripts/validate.py
# 仅重建 CAD 网格时需要 cadquery（含较大的 OpenCASCADE/VTK 依赖）
uv run --group cad python scripts/convert_cad.py
uv run python scripts/build_model.py
```

已通过 8 项测试：六方向运动、10 秒静止加 12 秒演示稳定性、装甲方向/倾角/高度检查。渲染在 EGL 下通过。滚子为椭球近似，惯量、摩擦、电机参数尚未实车标定，存在接触波动和滑移。

这是可运行的底盘仿真原型，不是完整参赛步兵：未实现 17 mm 发射机构、裁判系统电子功能、电池/热量/功率规则。装甲遮挡角和制造装配尚未做完整检录级校核，不能据此直接认定实物合规。

## 2023 赛场点选导航

新增独立导航场景与双 MID-360 雷达：

```bash
uv run python -m navigation.server
```

浏览器打开 http://127.0.0.1:8765，在绿色可达区域点击目标。全局路径、局部预测、激光点云、三维跟随画面实时更新。

使用方式、雷达官方依据、实测记录以及先验辅助建图/2.5D 地形的边界见 [导航说明](docs/navigation.md)。

## 一对一 图像自瞄

```bash
MUJOCO_GL=egl uv run python -m duel.server --port 8766
```

浏览器打开 http://127.0.0.1:8766/ 。真实 RGB 灯条检测与 PnP、云台角度/理想自身里程计融合 EKF、重力弹道、装甲命中裁判与 2026 热量限制。支持键盘驾驶、手动/自动射击、移动目标和双方视觉反击。

首次进入开启「对准后自动开火」，再切换「往返横移」。[详细操作、官方参数与仿真边界](docs/duel.md)。原有底盘和导航入口不变。
