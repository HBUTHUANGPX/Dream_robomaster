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
    std::cout << "Robot search tests passed\n";
  } catch (const std::exception& e) {
    std::cerr << e.what() << '\n';
    return 1;
  }
}
