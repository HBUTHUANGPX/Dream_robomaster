# RoboMaster 仿真仓库

本仓库提供导航、自瞄对战和双夹爪魔方仿真。
仿真与算法使用 C++。启动器和网页服务使用 Rust。
运行当前版本无需安装 Python。

## 首次使用

支持的平台是 **Ubuntu 22.04、x86_64 处理器**。
首次准备需要网络。安装缺少的系统软件时，需要管理员权限。
网页渲染需要可用的 OpenGL/EGL 驱动。
魔方窗口还需要 Linux 图形桌面。

从[GitHub 仓库](https://github.com/HBUTHUANGPX/Dream_robomaster)取得包含 `assets/` 的完整源码。
在仓库根目录打开终端。该目录包含本文件和 `rm` 文件。
如果还未取得仓库，或不清楚终端位置，请先读[首次使用指南](docs/getting-started.md)。

执行准备命令：

~~~bash
./rm setup
~~~

准备成功后，启动两个网页服务：

~~~bash
./rm
~~~

等待终端显示两个地址。在运行程序的同一台电脑上打开对应地址：

- [导航页面](http://127.0.0.1:8765/)
- [自瞄对战页面](http://127.0.0.1:8766/)

保持终端打开。按 `Ctrl+C` 停止本次启动的两个服务。
也可以在运行主机上打开另一个终端。进入同一仓库目录，执行：

~~~bash
./rm stop
~~~

该命令停止当前用户在本仓库启动的所有网页服务，包括单独启动和自定义端口的服务。
命令会等待服务退出。没有运行中的服务时，命令也会正常结束。
如果程序运行在远程 Linux 主机上，请先按[远程访问步骤](docs/getting-started.md#远程访问)设置端口转发。

## 日常使用

| 目的 | 在仓库根目录执行的命令 |
| --- | --- |
| 启动两个网页服务 | `./rm` |
| 停止本仓库的所有网页服务 | `./rm stop` |
| 只启动导航页面 | `./rm navigation` |
| 只启动自瞄对战页面 | `./rm duel` |
| 打开魔方窗口 | `./rm cube` |
| 打开底盘窗口 | `./rm robot` |
| 检查环境 | `./rm doctor` |
| 构建程序 | `./rm build` |
| 运行测试 | `./rm test` |
| 查看命令帮助 | `./rm help` |

启动命令会检查构建结果。代码更新后，无需另记一条构建命令。
停止命令使用已有的 Rust 启动器。它不检查开发环境，不重新构建。
窗口模式需要图形桌面。远程网页模式不需要 Linux 桌面。
已有程序占用端口时，新程序会退出并说明原因。
使用 `./rm cube --rpc` 时，标准输出只写逐行 JSON，构建和诊断写入标准错误。
接入示例见[逐行 JSON 协议步骤](docs/getting-started.md#接入逐行-json-协议)。

双夹爪无窗口求解默认使用十二元动作搜索。可用 `--plan-only` 只规划，
用 `--search-ms 1000` 设置搜索预算。搜索耗时不计入动作成本，当前不支持动作重叠。
搜索默认最多使用三个线程；用 `--search-threads 1` 可切换为串行搜索。物理动作仍按顺序执行。
命令、成本配置和最优性边界见[双夹爪搜索指南](docs/cube-primitive-search.md)。

## 按任务查阅

- [首次使用、环境条件和远程访问](docs/getting-started.md)
- [启动失败和常见问题](docs/troubleshooting.md)
- [技术名称与缩写](docs/glossary.md)
- [导航操作、定位和数据导出](docs/modules/navigation.md)
- [对战操作、视觉和射击](docs/modules/duel.md)
- [魔方操作、双夹爪和录像](docs/modules/cube.md)
- [RX 窄指尖设计与单层验证](docs/rx-narrow-tip.md)
- [双夹爪十二元动作搜索、预算和成本](docs/cube-primitive-search.md)
- [开发、目录和测试](docs/development.md)
- [迁移记录与已验证的功能](docs/migration.md)
- [可选 CAD 转换](docs/modules/cad.md)
- [可选 FAST-LIO 离线回放](integrations/fast_lio/README.md)
- [第三方来源与许可证](THIRD_PARTY.md)
- [中文文档写作规范](docs/writing-guide.md)

导航默认使用 ICP 定位。FAST-LIO 不属于日常启动流程。
FAST-LIO 的 ROS 构建和容器回放尚未通过本机运行验收。
历史 Python 代码只用于对照，见[历史版本说明](legacy/README.md)。
