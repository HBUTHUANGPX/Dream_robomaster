#include <iostream>

#include "rm/mechanical_plan.hpp"
#include "rm/robot_search.hpp"

using namespace rm::cube;
using rm::Json;
namespace {
void need(bool value, const char* text) {
  if (!value) throw std::runtime_error(text);
}

Json measured(const Plan& plan, double duration) {
  size_t count = 0;
  double seconds = 0;
  for (const auto& a : plan) {
    if (a.kind == "yaw") {
      ++count;
      seconds += duration + .15;
    } else if (a.kind == "jaw") {
      ++count;
      seconds += .65;
    }
  }
  return {{"action_count", count}, {"estimated_execution_s", seconds}};
}

// 仅重放连续接触动作，不创建 Cube，不修改旧重放器的锁或 checkpoint 约束。
void replay(const Plan& plan, const std::vector<std::string>& moves, Mat orientation) {
  std::array<int, 2> wrist{0, 0};
  std::array<bool, 2> closed{true, true};
  std::string pending, actual = solved, expected = solved;
  size_t progress = 0;
  int partial = 0;
  for (const auto& a : plan) {
    const int h = a.hand == "A" ? 0 : 1, other = 1 - h;
    if (a.kind == "jaw") {
      need(a.target == 0 || a.target == 115, "夹爪目标错误");
      closed[h] = a.target == 115;
      need(closed[0] || closed[1], "失去全部支撑");
    } else if (a.kind == "yaw") {
      need(pending.empty(), "上一层动作未更新状态");
      const int next = std::lround(a.target / (pi / 2)), delta = next - wrist[h];
      need(std::abs(next) <= 2 && std::abs(delta) >= 1 && std::abs(delta) <= 2, "腕角超界或空动作");
      need(std::abs(a.target - next * pi / 2) < 1e-9 && wrist[other] % 2 == 0,
           "腕角未对齐或横腕碰撞");
      if (a.mode == "face") {
        need(closed[h] && closed[other], "层转缺少双爪支撑");
        auto [face, count] = parse_move(a.move);
        auto [axis, sign] = face_axis(face);
        const Vec normal = orientation * Vec::Unit(axis) * sign;
        need(normal.isApprox(h == 0 ? Vec(1, 0, 0) : Vec(0, -1, 0)), "层转面错误");
        need((delta + count + 4) % 4 == 0, "层转方向错误");
        pending = a.move;
      } else if (a.mode == "whole") {
        need(closed[h] && !closed[other] && a.move.empty(), "整块转动支撑错误");
        orientation = wrist_rotation(a.hand, delta * pi / 2) * orientation;
      } else {
        need(a.mode == "empty" && !closed[h] && closed[other], "空转支撑错误");
      }
      wrist[h] = next;
    } else if (a.kind == "layer_lock") {
      need(!pending.empty() && a.move == pending, "层状态更新错误");
      need(progress < moves.size(), "多余层动作");
      auto [face, count] = parse_move(a.move);
      need(face == parse_move(moves[progress]).first, "输入顺序错误");
      partial = (partial + count + 4) % 4;
      actual = replay_facelet_moves(actual, {a.move});
      pending.clear();
    } else if (a.kind == "checkpoint") {
      need(progress < moves.size() && a.move == moves[progress], "检查点未保留原输入");
      need(pending.empty() && partial == (parse_move(a.move).second + 4) % 4,
           "原输入未完成便产生检查点");
      expected = replay_facelet_moves(expected, {a.move});
      need(actual == expected, "逐动作面片状态不一致");
      ++progress;
      partial = 0;
    } else {
      need(a.kind == "release" || a.kind == "grasp" || a.kind == "layer_unlock", "未知动作");
    }
  }
  need(progress == moves.size() && pending.empty() && partial == 0, "输入未全部完成");
  need(wrist == std::array<int, 2>{0, 0} && closed == std::array<bool, 2>{true, true},
       "终态没有归零双闭");
}

MechanicalPlan check(const std::vector<std::string>& moves, Mat orientation = Mat::Identity(),
                     double duration = .4, double budget = 1000) {
  const auto result = plan_mechanical_moves(moves, orientation, duration, budget);
  need(result.report.at("moves") == moves, "报告输入序列错误");
  need(result.report.at("start_orientation") == matrix_json(orientation), "报告起始朝向错误");
  const auto raw = compile_moves(moves, orientation, true);
  const auto peephole = optimize_plan(raw, orientation);
  need(result.report.at("raw") == measured(raw, duration), "原始成本错误");
  need(result.report.at("peephole") == measured(peephole, duration), "回退成本错误");
  need(result.report.at("chosen") == measured(result.actions, duration), "选用成本错误");
  need(result.report.at("estimated_execution_s") ==
           measured(result.actions, duration)["estimated_execution_s"],
       "总成本错误");
  need(result.report.at("estimated_execution_s").get<double>() <=
           measured(peephole, duration)["estimated_execution_s"].get<double>(),
       "搜索使成本增加");
  double candidate_seconds = 0;
  for (const auto& a : result.report.at("candidate")) {
    const double seconds = a.at("mode") == "jaw" ? .65 : duration + .15;
    need(std::abs(a.at("duration_s").get<double>() - seconds) < 1e-10, "候选使用旧速度成本");
    candidate_seconds += seconds;
  }
  if (result.report.at("chosenplanner") == "robot_search") {
    need(result.report.at("candidate").size() == measured(result.actions, duration)["action_count"],
         "候选与实际执行动作数不同");
    need(std::abs(candidate_seconds - result.report.at("estimated_execution_s").get<double>()) <
             1e-9,
         "候选与实际执行成本不同");
  }
  replay(result.actions, moves, orientation);
  return result;
}
}  // namespace

int main(int argc, char** argv) {
  try {
    // 保持现有测试的 argv root 入口；软件验证无需模型或资产。
    if (argc > 1) need(std::filesystem::is_directory(argv[1]), "仓库目录不存在");
    const auto f = check({"F"});
    for (const auto& a : f.actions)
      if (a.kind == "yaw" && a.mode == "face") need(a.hand == "B", "F 未使用 B 腕");
    const auto repeated = check({"R", "R"});
    need(repeated.report.at("chosenplanner") == "robot_search", "连续 R 未选用搜索");
    need(repeated.report.at("chosen")["action_count"] == 5, "连续 R 中途多余回腕");
    need(repeated.report.at("chosen")["action_count"] <
             repeated.report.at("peephole")["action_count"],
         "连续 R 未节省动作");
    const auto half = check({"R2"});
    size_t checkpoints = 0;
    for (const auto& a : half.actions) checkpoints += a.kind == "checkpoint";
    need(checkpoints == 1, "半转检查点数量错误");
    check({"R'", "R2", "R", "F2", "F'"});
    const auto many = split_moves("R U F2 L' D B R2 U' F L2 D' B2 R' U2 F' L D2 B' R F2");
    check(many);
    check(many, robot_orientations()[7]);
    check({"F", "R2"}, robot_orientations()[13], 1.2);
    need(check({}).actions.empty(), "空序列生成多余动作");
    const auto fallback = check(many, Mat::Identity(), .4, 0);
    need(fallback.actions == optimize_plan(compile_moves(many, Mat::Identity(), true)),
         "零预算回退错误");
    need(fallback.report.at("candidate").empty() && fallback.report.at("search_ms") == 0 &&
             fallback.report.at("chosenplanner") == "peephole",
         "零预算仍然搜索");
    check(many, Mat::Identity(), .4, .000001);
    for (const auto& values :
         std::array<std::array<double, 2>, 5>{{{0., 0.},
                                               {-1., 0.},
                                               {.4, -1.},
                                               {.4, 60001.},
                                               {std::numeric_limits<double>::quiet_NaN(), 0.}}}) {
      bool rejected = false;
      try {
        plan_mechanical_moves({}, Mat::Identity(), values[0], values[1]);
      } catch (const std::invalid_argument&) {
        rejected = true;
      }
      need(rejected, "非法规划参数未拒绝");
    }
    // 旧锁模型仍按原规则重放原计划。
    need(replay_plan(compile_moves({"R2", "F"}, Mat::Identity(), true))["moves"] ==
             std::vector<std::string>({"R2", "F"}),
         "旧锁模型回归");
    std::cout << "连续接触规划软件测试通过。\n";
  } catch (const std::exception& e) {
    std::cerr << "失败：" << e.what() << '\n';
    return 1;
  }
}
