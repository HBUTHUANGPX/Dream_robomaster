# 开发与验证

## 开始条件

在仓库根目录打开终端。
先完成[首次使用指南](getting-started.md)中的环境准备。
如果需要运行测试，执行：

~~~bash
./rm setup --with-tests
~~~

该命令在普通准备的基础上检查 Node.js。
当前网页脚本测试需要 Node.js 18 或更新版本。
运行网页服务本身不需要 Node.js。
准备脚本会复用合适的已安装工具。

## 目录职责

| 目录或文件 | 职责 |
| --- | --- |
| `rm` | 用户统一入口；检查环境并分派命令 |
| `modules/navigation/` | 底盘、地形、雷达、定位、规划、控制和数据导出 |
| `modules/duel/` | 装甲识别、目标跟踪、弹道、热量和裁判逻辑 |
| `modules/cube/` | 魔方状态、通用求解、动作规划和双夹爪执行 |
| `libs/sim/` | 模型生命周期、无窗口渲染、图像编码和进程间协议 |
| `crates/robomaster/` | Rust 启动器、HTTP 接口和原生进程管理 |
| `web/` | 导航与对战网页 |
| `assets/` | 共享模型、网格、图片、权重和来源记录 |
| `tools/` | 环境准备、依赖下载、文档检查和可选 CAD 工具 |
| `integrations/fast_lio/` | 可选 ROS1 离线回放的源码和容器配置 |
| `tests/` | 根入口、网页控件和真实浏览器测试 |
| `docs/` | 中文使用、开发、迁移和设计文档 |
| `legacy/python/` | 历史实现、历史测试、依赖锁和中文历史说明 |
| `.deps/`、`.cache/` | 本机依赖和缓存，不纳入版本控制 |
| `build/`、`target/` | C++ 与 Rust 构建结果，不纳入版本控制 |
| `output/` | 录像、报告等生成文件，不纳入版本控制 |

## 构建

在仓库根目录执行：

~~~bash
./rm build
~~~

成功后，C++ 程序位于 `build/bin/`。
Rust 程序位于 `target/release/robomaster`。
运行命令会再次检查增量构建，因此日常使用无需单独构建。

已有 SDK 和工具链完整时，仍可使用 `make build`。
`make` 接口用于开发兼容，不负责首次安装系统依赖。
移动整个仓库后，需要重新生成含绝对路径的构建缓存，见[故障处理](troubleshooting.md#下载或构建失败)。

## 统一测试

执行：

~~~bash
./rm test
~~~

该命令构建程序，再运行 C++、Rust、网页脚本、入口和文档检查。
原生渲染测试需要可用的 EGL 驱动。
测试成功时，所有测试命令均返回零。
任一命令失败时，统一入口返回非零状态。

可选 CAD 测试只在启用 CAD 构建时出现。
默认测试数量与启用 CAD 后的数量不同。
完整导航场景验收和双夹爪长序列不属于每次快速测试，执行方法见模块文档。

## 查看原生接口

完成构建后，在仓库根目录执行：

~~~bash
target/release/robomaster --help
~~~

底层启动器支持以下用途：

| 用途 | 命令 |
| --- | --- |
| 同时启动网页服务 | `target/release/robomaster up` |
| 指定两个网页端口 | `target/release/robomaster up --navigation-port 18765 --duel-port 18766` |
| 启动一个网页服务 | `target/release/robomaster serve navigation` |
| 运行原生魔方任务 | `target/release/robomaster run cube -- --headless --scramble R --solve` |

从其他目录调用底层程序时，用 `--root` 指定仓库的绝对路径。
普通用户使用根入口即可，无需记住这些底层路径。

## 原生进程协议

C++ 进程使用标准输入接收逐行 JSON。
每个请求产生一条 JSON 响应。
标准输出只承载协议数据。标准错误承载诊断信息。

请求示例：

~~~json
{"command":"state","args":{}}
~~~

成功响应使用 `ok` 和 `result` 字段。
失败响应使用 `ok` 和 `error` 字段。
协议字段是程序标识，保持原样。

Rust 将命令串行发送到 C++ 进程。
MuJoCo 和 EGL 对象保留在所属线程。
HTTP 请求体上限为 4 KiB。命令队列容量为 64。
对战底盘命令有 350 ms 有效期，过期命令不继续驱动车体。

## 真实浏览器验收

这项验收需要 Firefox、geckodriver 和两个正在运行的网页服务。
普通 `./rm test` 不会自动安装或启动这些外部程序。
先从各自发布方取得与当前平台匹配的 Firefox 和 geckodriver。
把 geckodriver 放到可执行路径，或用其绝对路径代替下面的命令。

1. 在第一个终端执行 `./rm`，等待两个服务启动。
2. 在第二个终端执行 `geckodriver --port 14446`，保持终端打开。
3. 在第三个终端进入仓库根目录，执行以下命令：

~~~bash
PATH="$PWD/.deps/node/bin:$PATH" WEBDRIVER_URL=http://127.0.0.1:14446 NAV_URL=http://127.0.0.1:8765 DUEL_URL=http://127.0.0.1:8766 node --test tests/native_web.test.mjs
~~~

如果服务使用其他端口，同步修改 `NAV_URL` 和 `DUEL_URL`。
测试通过时，两项浏览器测试均成功。
结束后，在前两个终端分别按 `Ctrl+C`。
未设置这些环境变量时，浏览器测试会跳过。跳过不能算验收通过。

## 文档维护

遵守[中文写作规范](writing-guide.md)。
完成 `./rm setup --with-tests` 后，在仓库根目录执行以下命令：

~~~bash
PATH="$PWD/.deps/node/bin:$PATH" node tools/check-docs.mjs
~~~

命令会优先使用准备脚本安装的本地 Node.js。
如果没有本地安装，则使用当前终端可找到的 Node.js。
检查器发现缺失的本地链接或明显英文段落时，会返回非零状态。
自动检查不能替代人工确认命令、前提和结果。

历史文档可以更新中文表达。
历史源程序、测试和依赖锁的校验依据保留在 `legacy/baseline-sha256.json`。
中文化后的文档与原始文档字节不同，不再宣称全部历史文件保持同一校验值。

## 可选工具

CAD 转换的 SDK 准备、独立构建和误差边界见[CAD 说明](modules/cad.md)。
FAST-LIO 的构建和回放见[集成说明](../integrations/fast_lio/README.md)。
这两项都不属于首次启动网页的必要步骤。
