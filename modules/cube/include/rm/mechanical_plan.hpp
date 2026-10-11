#pragma once

#include "rm/cube.hpp"

namespace rm::cube {
struct MechanicalPlan {
  Plan actions;
  Json report;
};
// 起态为双爪闭合、双腕归零；仅生成连续接触执行器使用的软件动作。
MechanicalPlan plan_mechanical_moves(const std::vector<std::string>& moves, Mat orientation,
                                     double turn_duration_s, double search_ms);
}  // namespace rm::cube
