# STEP 原生转换工具

技术名称和缩写见[术语表](../glossary.md)。

`rm_convert_cad` 使用 C++20 和 OpenCASCADE（OCCT）。它替代旧版 `scripts/convert_cad.py`，不调用 Python 进程或库。默认仿真使用仓库内的 OBJ 网格，不需要 OCCT。

首次使用仓库时，请先阅读[根目录说明](../../README.md)和[入门说明](../getting-started.md)。下列命令均在仓库根目录运行。先完成 `./rm setup`。仅在需要重新转换 STEP 时执行以下附加步骤。

## 准备与运行

以下依赖脚本适用于 Ubuntu 22.04。系统须能运行 `apt-get download` 和 `dpkg-deb`，并能访问发行版软件源。脚本只下载并解包到已忽略的 `.deps/occt`，不向系统安装软件包。

```bash
bash tools/cad/bootstrap-occt.sh
cmake -S tools/cad -B build-cad -G Ninja -DCMAKE_BUILD_TYPE=Release
cmake --build build-cad
build-cad/rm_convert_cad --root "$PWD" --output output/native-cad \
  --compare assets/meshes/metadata.json
ctest --test-dir build-cad --output-on-failure
```

独立构建需要 OCCT 的 STEP、网格化和建模库，以及 nlohmann JSON。它不链接仿真或图形库。使用其他 OCCT 开发库时，通过 `OCCT_ROOT` 指定安装前缀。附带脚本使用 Ubuntu 22.04 的 OCCT 7.5 软件包名称。

解包目录保留 Debian 软件包的版权和许可证文件。脚本在本地记录版本及 SHA-256。OCCT 使用 LGPL-2.1 及其附加例外条款。具体条款以开发库中的版权声明为准。

## 参数与输出

程序接受 `--root`、`--input-dir`、`--output`、`--compare` 和 `--strict-triangles`。输入为 `armor_am02.step` 和 `armor_frame_a.step`。

OBJ 坐标保留九位小数，索引从 1 开始。`metadata.json` 保留 `min_m`、`max_m` 和 `triangles` 字段。默认输出目录为 `output/native-cad`。该目录与仓库内的运行网格分开。

`conversion.json` 记录 OCCT 版本和网格化约定。指定 `--compare` 后，`comparison.json` 记录边界和三角形数差异。边界误差须不超过 1 µm，比较才算成功。`--strict-triangles` 还要求三角形数量完全一致。STEP 缺失或格式错误时，程序在写出网格之前报错。

## 坐标与网格化约定

- AM02 先减去中心 `[-17.40697118333, 63.89617963739, -45.00439550208]` mm。再将 `[x,y,z]` 映射为 `[x,-z,y]`。最后将毫米转换为米。
- A 型支架先从原始 Y 坐标减去 32 mm。再映射为 `[-sin(15°) y + cos(15°) z, x, cos(15°) y + sin(15°) z]`。最后转换为米。
- STEP 按毫米导入。首个转换形状对应旧版 `importStep(...).val()`。程序按 OCCT 的索引形状表遍历面，并应用网格位置变换。反向面的三角形顶点顺序随之反转。

旧版 CadQuery 的 `tessellate(0.25, 0.15)` 实际调用 `BRepMesh_IncrementalMesh(shape, 0.25, true, 0.15)`。其中 `true` 启用**相对偏差**。因此，0.25 是原始相对网格化参数，不保证绝对偏差为 0.25 mm。原生工具保留此行为，也保留 0.15 rad 的角度参数。

## 验证与限制

本机使用 OCCT 7.5.1 完成两份官方 STEP 的转换。输出边界与仓库元数据的最大差异为 `6.94e-18 m`。A 型支架的三角形数量均为 13,490。AM02 的输出为 144,352 个三角形，旧版网格为 110,276 个。

旧版 CadQuery 环境使用 OCCT 7.9.3。即使网格化参数相同，内核版本变化仍会改变三角剖分。本工具保留尺寸、坐标和网格化参数，不承诺不同 OCCT 版本产生相同的三角形连接关系。仓库原有网格仍是默认运行资源。
