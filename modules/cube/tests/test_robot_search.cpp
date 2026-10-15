#include <iostream>

#include "rm/robot_search.hpp"
using namespace rm::cube;
void need(bool v, const char* text) {
  if (!v) throw std::runtime_error(text);
}
int main() {
  try {
    SearchOptions o;
    o.max_search_ms = 1000;
    o.costs = PrimitiveCosts::defaults(8, 32, true);
    auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
    auto p = optimize_robot_moves({"R'", "R'"}, RobotState{}, o, deadline);
    need(p.has_value(), "plan missing");
    need(p->actions.size() == 2, "unnecessary reset");
    need(primitive_moves(p->actions, {}) == std::vector<std::string>({"R'", "R'"}),
         "wrong face replay");
    auto half = optimize_robot_moves({"R2"}, {}, o, deadline);
    need(half && half->actions.size() == 1, "half turn primitive");
    o.costs.apply_json({{"duration_s", {{"A_P180", 9.0}, {"A_N180", .01}}}});
    half = optimize_robot_moves({"R2"}, {}, o, deadline);
    need(half && half->actions == std::vector<Primitive>{Primitive::A_N180}, "cost selection");
    auto f = optimize_robot_moves({"F"}, {}, o, deadline);
    need(f && f->actions == std::vector<Primitive>{Primitive::B_N90}, "B face selection");
    o.home = true;
    auto home = optimize_robot_moves({"R'"}, {}, o, deadline);
    need(home && home->end.wrist == std::array<int, 2>{0, 0} &&
             home->end.closed == std::array<bool, 2>{true, true},
         "home goal");
    need(!optimize_robot_moves({"U"}, {}, o, std::chrono::steady_clock::now()), "expired deadline");
    o.wrist_ranges = {{{-1, 0}, {0, 1}}};
    for (const auto& move : split_moves("R R' R2 L L' L2 U U' U2 D D' D2 F F' F2 B B' B2")) {
      auto limited = optimize_robot_moves({move}, {}, o,
          std::chrono::steady_clock::now() + std::chrono::seconds(2));
      need(limited.has_value(), "受限单面搜索无解");
      RobotState state;
      for (auto action : limited->actions) {
        state = primitive_transition(state, action)->state;
        for (int h = 0; h < 2; ++h)
          need(state.wrist[h] >= o.wrist_ranges[h][0] &&
                   state.wrist[h] <= o.wrist_ranges[h][1], "搜索腕角越限");
      }
      need(replay_facelet_moves(solved, primitive_moves(limited->actions, {})) ==
               replay_facelet_moves(solved, {move}), "受限搜索逻辑不等价");
    }
    RobotState invalid_start;
    invalid_start.wrist[0] = 1;
    bool rejected = false;
    try {
      optimize_robot_moves({}, invalid_start, o, std::chrono::steady_clock::now());
    } catch (const std::invalid_argument&) { rejected = true; }
    need(rejected, "越限起态未拒绝");
    rejected = false;
    try {
      search_robot_solution(solved, invalid_start, o, {});
    } catch (const std::invalid_argument&) { rejected = true; }
    need(rejected, "完整求解未拒绝越限起态");
    for (auto range : std::array<std::array<int, 2>, 3>{{{-3, 0}, {0, 3}, {1, 0}}}) {
      o.wrist_ranges[0] = range;
      rejected = false;
      try {
        optimize_robot_moves({}, {}, o, std::chrono::steady_clock::now());
      } catch (const std::invalid_argument&) { rejected = true; }
      need(rejected, "非法腕范围未拒绝");
    }
    o.wrist_ranges = {{{0, 0}, {0, 0}}};
    need(!optimize_robot_moves({"R"}, {}, o,
             std::chrono::steady_clock::now() + std::chrono::seconds(2)), "锁腕仍找到动作");
    std::cout << "Robot search tests passed\n";
  } catch (const std::exception& e) {
    std::cerr << e.what() << '\n';
    return 1;
  }
}
