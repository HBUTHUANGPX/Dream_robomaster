#pragma once

#include <libxml/tree.h>
#include <mujoco/mujoco.h>

#include <filesystem>

namespace rm::cube {
// bundle 为只读 mujoco_linkage_v5 目录，读取其中 free_sweep.xml。
// 向现有 radian 场景追加两只 RX、窄指尖及闭环约束，不追加魔方或固定装置。
// 两手固定 roll +90° 对应逻辑 yaw=0；父场景负责魔方碰撞掩码和仿真 option。
// 重复导入、命名冲突、缺失资产或非法 XML 抛出异常。
void append_rx_grippers(xmlNodePtr root, const std::filesystem::path& bundle);

// 仅设置 RX 电机、被动闭环关节 qpos 和夹爪 ctrl，电机初始角为 -2.25 rad。
// 不改变模型参数、逻辑 yaw、魔方状态或仿真时间；调用者随后执行 mj_forward。
void initialize_rx_grippers(mjModel* model, mjData* data);
}  // namespace rm::cube
