#pragma once

#include "rm/cube.hpp"
#include "rm/primitive.hpp"

namespace rm::cube {
struct MechanicalPlan {
  Plan actions;
  Json report;
};
// 起态为双爪闭合、双腕归零；仅生成连续接触执行器使用的软件动作。
// costs 非空时为每次动作的完整耗时（含等待），不再额外加 .15/.65 秒。
// 成本表全部元素必须为有限正数且不超过3600秒；空指针保持旧四参行为。
// wrist_ranges 为 A、B 的 quarter 闭区间，默认保持原行为。
// 收窄范围后只交付受限搜索结果；预算内未找到方案则抛出 runtime_error。
MechanicalPlan plan_mechanical_moves(const std::vector<std::string>& moves, Mat orientation,
                                     double turn_duration_s, double search_ms,
                                     const PrimitiveCosts* costs = nullptr,
                                     std::array<std::array<int, 2>, 2> wrist_ranges =
                                         {{{-2, 2}, {-2, 2}}});
}  // namespace rm::cube
