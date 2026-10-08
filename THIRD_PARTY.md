# 第三方来源与许可证

本仓库保留第三方许可证、版权声明和来源记录。
下表说明各组件的位置和固定版本。
组件的授权条件以随附许可证原文为准。
中文说明不替代许可证，也不为全部第三方内容赋予新的统一许可证。

| 组件 | 来源或位置 | 版本与授权记录 |
| --- | --- | --- |
| MuJoCo 原生开发包 | [官方发布](https://github.com/google-deepmind/mujoco/releases/tag/3.15.0)，安装到 `.deps/mujoco/` | 3.15.0；保留 Apache-2.0 许可证与第三方声明 |
| OpenCV 原生开发包 | [官方发布](https://github.com/opencv/opencv/releases/tag/4.12.0)，安装到 `.deps/opencv/` | 4.12.0；Apache-2.0；源码许可证保留在依赖目录 |
| Eigen 线性代数库 | 系统开发包 | MPL-2.0；授权记录由软件包提供 |
| nlohmann JSON 库 | 系统开发包 | MIT；授权记录由软件包提供 |
| GLFW 窗口库与 libxml2 解析库 | 系统开发包 | 分别使用 zlib 和 MIT 许可证 |
| Rust 依赖库 | 版本记录在 `Cargo.lock` | 具体许可证见各依赖包的元数据和随附文件 |
| Kociemba 原生求解器 | `modules/cube/third_party/kociemba/` | 1.2.1；保留 GPL-2.0 许可证、来源和校验清单 |
| Robotiq 夹爪几何 | `assets/robotiq_2f85/` | 保留 `source.json` 和许可证 |
| 自瞄权重与装甲数字图像 | `assets/rm_auto_aim/`、`assets/armor_labels/` | 保留原始来源、哈希和许可证 |
| RoboMaster 官方 CAD 与规则资料 | `assets/official/`、`assets/meshes/` | 保留来源记录和坐标元数据 |
| 赛场与同济参考资料 | `assets/arena/`、`assets/tongji_reference/` | 保留来源记录和许可证 |
| FAST-LIO 与 ikd-Tree | 可选容器构建自动获取 | 固定提交见[集成说明](integrations/fast_lio/README.md)；上游许可证随源码保留 |
| 可选 OCCT 开发包 | `.deps/occt/` | Ubuntu 22.04 的 OCCT 7.5.1；LGPL-2.1 与 OCCT 例外条款 |

## 原始文件与中文说明

维护的使用说明、模型说明和历史说明使用中文。
程序名、协议字段、命令、版本号和网址保留原样。
第三方许可证、版权声明及厂商原始资料不作改写。
这些原始文件用于核对授权与资产来源。

历史检出的 FAST-LIO 源码保留在本机忽略目录
`.deps/references/fast-lio-upstream/`。
该目录不是构建前提，也不纳入本仓库版本控制。
可选容器会自行获取固定提交，用户不需要初始化根 Git 子模块。

## 再分发前的检查

魔方求解器由仓库内的 C 源码编译，不调用 Python 包。
再分发包含该求解器的程序前，阅读随附 GPL-2.0 许可证。
对其他代码、模型、权重和官方资料，分别检查对应授权。

生成录像、测试报告、下载缓存和容器归档不纳入源码索引。
原始网格与 ONNX 权重仍是当前程序的运行输入。
