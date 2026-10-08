# Kociemba 原生求解器

本目录保存已安装 `kociemba` 1.2.1 软件包中的原始 C 源码和预计算表。上游为 [muodov/kociemba](https://github.com/muodov/kociemba)，作者包括 muodov、Herbert Kociemba 和 Alexandre Buisse。

C 源码位于 `ckociemba/`，生成的剪枝数据位于 `cprunetables/`。本目录不包含或使用原始 Python 包装层，也不使用下载的可执行二进制文件。源码与数据未因本说明中文化而修改。

许可证为 GPL-2.0，全文见 [LICENSE](LICENSE)。将该求解器链接到 `rm_cube` 后，组合可执行文件须满足 GPL 的分发条件。这项说明不改变仓库内无关模块的许可证。

仓库首次使用说明见[根目录说明](../../../../README.md)和[入门说明](../../../../docs/getting-started.md)。运行和验证方法见[魔方模块](../../../../docs/modules/cube.md)。
