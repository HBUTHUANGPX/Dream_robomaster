# RX夹爪运行资产

本目录保存双RX夹爪仿真所需的原始MJCF、20个网格、机构清单及来源校验记录。
文件直接由用户提供的 `RX_ASSY_V2_with_gripper.zip` 提取，并逐字节对照原包。
运行无需下载目录、旧 `output/`、原压缩包或Python脚本。
窄指尖扩展由C++生成，其设计记录见[窄指尖资产](../rx_narrow_tip/README.md)。

## 来源与许可状态

原包SHA-256和本目录文件SHA-256见 [manifest.json](manifest.json)。
原始校验清单与说明保存在 `provenance/`，内容未经改写。
原始清单包含本次未收录的独立测试夹具与Python工具，不作为当前运行依赖清单。
原包未附明确许可证。本次按用户要求保存运行资产，不新增许可授权，不改变原权利归属。
仓库代码的许可证不自动替代这些来源资产的许可状态。

## 校验与使用

在仓库根目录操作。校验需要Node.js 18或更新版本：

```bash
node tools/check-rx-assets.mjs
```

预期显示24个文件校验通过，且场景网格引用完整。
失败时检查Git检出是否完整，以及资产是否被本地修改；不要用旧输出目录中的文件绕过校验。

运行前按[首次准备](../../docs/getting-started.md)安装原生依赖。
将 `--rx-bundle` 指向 `assets/rx_gripper/mujoco_linkage_v5`。
完整命令、参数和结果边界见[连续接触RX指南](../../docs/mechanical-rx.md#准备与运行)。
