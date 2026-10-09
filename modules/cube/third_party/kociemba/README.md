# Kociemba 原生求解器

本目录保存已安装 `kociemba` 1.2.1 软件包中的原始 C 源码和预计算表。上游为 [muodov/kociemba](https://github.com/muodov/kociemba)，作者包括 muodov、Herbert Kociemba 和 Alexandre Buisse。

C 源码位于 `ckociemba/`，生成的剪枝数据位于 `cprunetables/`。本目录不包含或使用原始 Python 包装层，也不使用下载的可执行二进制文件。预计算表保持原样。源码的本地修改见下文。

许可证为 GPL-2.0，全文见 [LICENSE](LICENSE)。将该求解器链接到 `rm_cube` 后，组合可执行文件须满足 GPL 的分发条件。这项说明不改变仓库内无关模块的许可证。

仓库首次使用说明见[根目录说明](../../../../README.md)和[入门说明](../../../../docs/getting-started.md)。运行和验证方法见[魔方模块](../../../../docs/modules/cube.md)。

## 本地修改

`ckociemba/search.c` 和 `ckociemba/include/search.h` 增加候选回调、取消上下文和 `enumerate_search` 入口。两阶段搜索都会检查取消。枚举模式对每个第一阶段叶节点只取第二阶段首解，然后返回第一阶段继续深度优先搜索。候选由第一阶段统一交付，避免预算耗尽在单个第二阶段子树。原 `solution` 的函数签名、首解流程和秒级超时语义保持不变；失败退出现在释放搜索资源。

C++ 入口见 [solution_candidates.hpp](../../include/rm/solution_candidates.hpp) 和 [solution_candidates.cpp](../../src/solution_candidates.cpp)。调用时传入仓库根目录、单调截止时间和候选回调。回调返回 `true` 继续，返回 `false` 停止。最多搜索 24 步，不保证候选最优或穷举。已还原状态交付一个空序列后结束。过期预算不读取表，也不交付候选。

新入口不调用 `initPruning`。每次请求按 4096 字节分块读取附带表，验证文件长度和固定 FNV-1a 64 校验值。缺失、截断或损坏表会抛出异常，不会生成替代表。表全部验证成功后才更新全局数据，失败不会发布部分表。

截止时间包含等待锁、表加载、首解搜索和回调耗时。取消采用协作检查，不能中断正在执行的文件系统调用或用户回调。调用者应使用本地常规文件，并让回调及时返回。回调异常在 C 层资源清理后重新抛出。多个枚举请求串行执行；不要与旧 `solution` 或 `initPruning` 并发调用。

`source.json` 保留引入时的原始校验记录，用于追溯来源；其中搜索文件的记录不是本地修改后的校验值。许可证不变。独立行为测试位于 [test_candidates.cpp](../../tests/test_candidates.cpp)，覆盖候选贴纸重放、多个候选、截止时间、停止回调、连续请求和表错误。
