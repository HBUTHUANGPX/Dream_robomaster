# Python 历史归档

**历史记录，不用于当前启动。** 首次使用仓库时，请阅读[根目录说明](../README.md)和[入门说明](../docs/getting-started.md)。当前原生应用不会导入归档，也不会执行 Python。

`python/` 保留三项原始任务的代码、测试、依赖锁定文件和历史文档。文档现已中文化，源程序保持归档内容。`baseline-sha256.json` 记录迁移时的原始文件内容。该清单保留不变，因此其中的文档摘要不用于证明中文化后的文件与原文相同。

完整初始快照还保存在本机历史产物 `output/migration/python-baseline-20261008.tar.gz` 中。该文件被 Git 忽略，不保证新检出的仓库包含它。

归档用于比较和回退参考。`python/assets`、`python/web` 和 `python/output` 是指向仓库共享目录的符号链接。历史命令以 `legacy/python` 为工作目录。

确需复现旧版时，请按[归档环境说明](python/README.md)准备环境，然后运行旧版测试。原有根目录 `.venv` 是本机已忽略的历史环境，不是新用户的前置依赖。新环境应使用归档内的 `pyproject.toml` 和 `uv.lock` 重建。
