#include "rm/mechanical_plan.hpp"

#include "rm/robot_search.hpp"

namespace rm::cube {
namespace {
using Clock = std::chrono::steady_clock;

Json measure(const Plan& plan, double turn_duration_s, const PrimitiveCosts* costs) {
  size_t count = 0;
  double seconds = 0;
  std::array<double, 2> wrist{};
  for (const auto& a : plan) {
    if (a.kind == "jaw" || a.kind == "yaw") {
      ++count;
      if (!costs) {
        seconds += a.kind == "jaw" ? .65 : turn_duration_s + .15;
        continue;
      }
      if (a.hand != "A" && a.hand != "B")
        throw std::invalid_argument("Unknown mechanical action hand");
      const int hand = a.hand == "A" ? 0 : 1;
      int kind;
      if (a.kind == "jaw") {
        if (a.target != 0 && a.target != 115)
          throw std::invalid_argument("Invalid mechanical jaw target");
        kind = a.target == 0 ? 4 : 5;
      } else {
        const double quarters = (a.target - wrist[hand]) / (pi / 2);
        if (!std::isfinite(quarters) || std::abs(quarters) > 2. + 1e-9)
          throw std::invalid_argument("Invalid mechanical wrist delta");
        const int turn = std::lround(quarters);
        if (turn == 0 || std::abs(quarters - turn) > 1e-9)
          throw std::invalid_argument("Mechanical wrist delta must be 90 or 180 degrees");
        kind = (std::abs(turn) == 2 ? 2 : 0) + (turn < 0 ? 1 : 0);
        wrist[hand] = a.target;
      }
      seconds += costs->seconds(Primitive(hand * 6 + kind), a.kind == "jaw" ? "jaw" : a.mode);
    }
  }
  return {{"action_count", count}, {"estimated_execution_s", seconds}};
}

// 严格保留输入边界：R2 可拆为 R R 或 R' R'，但不能跨过一个输入动作。
bool advance_move(const std::vector<std::string>& moves, size_t& index, int& partial,
                  const std::string& move) {
  if (index == moves.size()) throw std::runtime_error("Unexpected mechanical face move");
  auto [face, count] = parse_move(move);
  auto [wanted_face, wanted_count] = parse_move(moves[index]);
  if (face != wanted_face) throw std::runtime_error("Mechanical face move order mismatch");
  partial = (partial + count + 4) % 4;
  if (partial != (wanted_count + 4) % 4) return false;
  ++index;
  partial = 0;
  return true;
}

Plan convert(const std::vector<Primitive>& primitives, RobotState state,
             const std::vector<std::string>& moves) {
  // 转换前完整验证逻辑等价性，不实例化 Cube，也不使用旧焊接执行器。
  size_t index = 0;
  int partial = 0;
  for (const auto& move : primitive_moves(primitives, state))
    advance_move(moves, index, partial, move);
  if (index != moves.size() || partial != 0)
    throw std::runtime_error("Incomplete mechanical face sequence");

  Plan plan;
  index = 0;
  for (auto primitive : primitives) {
    const auto t = *primitive_transition(state, primitive);
    int h = int(primitive) / 6, kind = int(primitive) % 6;
    const std::string hand = h == 0 ? "A" : "B";
    if (kind >= 4)
      plan.push_back({"jaw", hand, kind == 4 ? 0. : 115.});
    else {
      plan.push_back({"yaw", hand, t.state.wrist[h] * pi / 2, t.mode, t.move});
      if (t.mode == "face") {
        plan.push_back({"layer_lock", hand, 0, "", t.move});
        if (advance_move(moves, index, partial, t.move))
          plan.push_back({"checkpoint", "", 0, "", moves[index - 1]});
      }
    }
    state = t.state;
  }
  if (state.wrist != std::array<int, 2>{0, 0} || state.closed != std::array<bool, 2>{true, true})
    throw std::runtime_error("Mechanical plan does not end home");
  return plan;
}
}  // namespace

MechanicalPlan plan_mechanical_moves(const std::vector<std::string>& moves, Mat orientation,
                                     double turn_duration_s, double search_ms,
                                     const PrimitiveCosts* costs,
                                     std::array<std::array<int, 2>, 2> wrist_ranges) {
  if (!std::isfinite(turn_duration_s) || turn_duration_s <= 0 || turn_duration_s > 3599.85)
    throw std::invalid_argument("Invalid mechanical turn duration");
  if (!std::isfinite(search_ms) || search_ms < 0 || search_ms > 60000)
    throw std::invalid_argument("Invalid mechanical search budget");
  if (costs)
    for (const auto& row : costs->duration)
      for (double seconds : row)
        if (!std::isfinite(seconds) || seconds <= 0 || seconds > 3600)
          throw std::invalid_argument("Mechanical action costs must be finite and in (0,3600]");
  if (wrist_ranges != SearchOptions{}.wrist_ranges) {
    SearchOptions options;
    options.home = true;
    options.wrist_ranges = wrist_ranges;
    options.max_search_ms = search_ms;
    if (costs)
      options.costs = *costs;
    else
      for (int a = 0; a < 12; ++a)
        options.costs.duration[a].fill(a % 6 >= 4 ? .65 : turn_duration_s + .15);
    const RobotState start{orientation_index(orientation), {0, 0}, {true, true}};
    const auto began = Clock::now();
    const auto budget = std::chrono::duration_cast<Clock::duration>(
        std::chrono::duration<double, std::milli>(search_ms));
    const auto deadline = began + budget;
    std::optional<RobotPlan> candidate;
    std::string planner = "robot_search";
    try {
      // 留一半总预算给逐动作搜索；它也使用相同腕范围和归零终态。
      candidate = optimize_robot_moves(moves, start, options, began + budget / 2);
    } catch (const std::runtime_error& e) {
      if (std::string(e.what()) != "Robot search memory limit reached") throw;
    }
    if (!candidate && Clock::now() < deadline) {
      RobotPlan combined;
      combined.end = start;
      bool complete = true;
      for (const auto& move : moves) {
        auto step = optimize_robot_moves({move}, combined.end, options, deadline);
        if (!step) {
          complete = false;
          break;
        }
        combined.actions.insert(combined.actions.end(), step->actions.begin(), step->actions.end());
        combined.end = step->end;
      }
      if (complete) {
        combined.stop_reason = "per_move_fallback";
        candidate = std::move(combined);
        planner = "robot_search_per_move";
      }
    }
    if (!candidate)
      throw std::runtime_error("No mechanical plan found within wrist_ranges and search budget");
    MechanicalPlan result{convert(candidate->actions, start, moves), Json::object()};
    auto& report = result.report;
    report["moves"] = moves;
    report["start_orientation"] = matrix_json(orientation);
    report["wrist_ranges"] = wrist_ranges;
    report["raw"] = nullptr;
    report["peephole"] = nullptr;
    report["chosenplanner"] = planner;
    report["candidate"] = primitive_plan_json(candidate->actions, start, options.costs);
    report["search_ms"] = std::chrono::duration<double, std::milli>(Clock::now() - began).count();
    report["search_stop_reason"] = candidate->stop_reason;
    report["chosen"] = measure(result.actions, turn_duration_s, costs);
    report["estimated_execution_s"] = report["chosen"]["estimated_execution_s"];
    return result;
  }
  const auto raw = compile_moves(moves, orientation, true);
  MechanicalPlan result{optimize_plan(raw, orientation), Json::object()};
  auto& report = result.report;
  report["moves"] = moves;
  report["start_orientation"] = matrix_json(orientation);
  report["raw"] = measure(raw, turn_duration_s, costs);
  report["peephole"] = measure(result.actions, turn_duration_s, costs);
  report["chosenplanner"] = "peephole";
  report["candidate"] = Json::array();
  report["search_ms"] = 0.;
  report["search_stop_reason"] = "disabled";

  if (search_ms > 0) {
    SearchOptions options;
    options.home = true;
    options.max_search_ms = search_ms;
    if (costs)
      options.costs = *costs;
    else
      for (int a = 0; a < 12; ++a)
        options.costs.duration[a].fill(a % 6 >= 4 ? .65 : turn_duration_s + .15);
    const RobotState start{orientation_index(orientation), {0, 0}, {true, true}};
    const auto began = Clock::now();
    const auto deadline = began + std::chrono::duration_cast<Clock::duration>(
                                      std::chrono::duration<double, std::milli>(search_ms));
    std::optional<RobotPlan> candidate;
    try {
      candidate = optimize_robot_moves(moves, start, options, deadline);
    } catch (const std::runtime_error& e) {
      // 搜索资源不足时仍交付已验证的回退方案。
      report["search_stop_reason"] = e.what();
    }
    report["search_ms"] = std::chrono::duration<double, std::milli>(Clock::now() - began).count();
    if (candidate) {
      auto plan = convert(candidate->actions, start, moves);
      report["candidate"] = primitive_plan_json(candidate->actions, start, options.costs);
      report["search_stop_reason"] = candidate->stop_reason;
      const auto metrics = measure(plan, turn_duration_s, costs);
      if (metrics["estimated_execution_s"].get<double>() <=
          report["peephole"]["estimated_execution_s"].get<double>()) {
        result.actions = std::move(plan);
        report["chosenplanner"] = "robot_search";
      }
    } else if (report["search_stop_reason"] == "disabled")
      report["search_stop_reason"] = "deadline";
  }
  report["chosen"] = measure(result.actions, turn_duration_s, costs);
  report["estimated_execution_s"] = report["chosen"]["estimated_execution_s"];
  return result;
}
}  // namespace rm::cube
