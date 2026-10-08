# Robotiq 2F-85 模型说明（MJCF）

本模型需要 MuJoCo 2.2.2 或更高版本。仓库首次使用说明见[根目录说明](../../README.md)和[入门说明](../../docs/getting-started.md)。双夹爪运行方法见[魔方模块](../../docs/modules/cube.md)。

## 概述

本目录包含 [Robotiq](https://robotiq.com/) 的 [85 mm 双指自适应夹爪](https://robotiq.com/products/2f85-140-adaptive-robot-gripper)简化 MJCF 模型。模型由[公开的 URDF 描述](https://github.com/ros-industrial/robotiq/tree/kinetic-devel/robotiq_2f_85_gripper_visualization)转换而来。

原上游说明引用的 `2f85.png` 未随本仓库提供。

原上游说明引用的 `CHANGELOG.md` 未随本仓库提供。本说明已翻译为中文。原始模型、版权和许可证保留。来源记录中的原始文件校验值对应引入时的版本，不表示翻译后的说明仍与原文逐字节相同。

## 从 URDF 转换为 MJCF 的步骤

以下步骤记录上游模型的制作过程，不是当前仓库启动所需操作。

1. 在 URDF 的 `<robot>` 中加入 `<mujoco> <compiler discardvisual="false"/> </mujoco>`，保留视觉几何。
2. 将 URDF 加载到 MuJoCo，并保存为 MJCF。
3. 手动编辑 MJCF，将共用属性提取到 `<default>`。
4. 添加 `<exclude>`，排除连杆刚体之间的碰撞。
5. 将碰撞垫拆分为两个垫块，增加接触点。
6. 提高垫块摩擦系数和接触优先级。
7. 设置 `impratio=10`，改善防滑表现。
8. 添加 `scene.xml`，放入机器人、带纹理的地面、天空盒和薄雾。
9. 在 `scene.xml` 中加入悬挂盒体。

## 许可证

模型使用 [BSD-2-Clause 许可证](LICENSE)。本说明的中文化不修改该许可证。
