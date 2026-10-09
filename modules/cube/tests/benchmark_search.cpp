#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <iostream>
#include <random>
#include <stdexcept>

#include "rm/robot_search.hpp"

#ifndef RM_BENCH_REVISION
#define RM_BENCH_REVISION "unknown"
#endif
namespace {
using namespace rm::cube;
using rm::Json;
using Clock = std::chrono::steady_clock;
using V = std::array<int, 3>;
constexpr std::array<V, 6> normal{
    {{0, 1, 0}, {1, 0, 0}, {0, 0, 1}, {0, -1, 0}, {-1, 0, 0}, {0, 0, -1}}};
constexpr std::array<V, 6> right{
    {{1, 0, 0}, {0, 0, -1}, {1, 0, 0}, {1, 0, 0}, {0, 0, 1}, {-1, 0, 0}}};
constexpr std::array<V, 6> down{
    {{0, 0, 1}, {0, -1, 0}, {0, -1, 0}, {0, 0, -1}, {0, -1, 0}, {0, -1, 0}}};
const std::string face_order = "URFDLB";
void need(bool ok, const std::string& why) {
  if (!ok) throw std::runtime_error(why);
}
int dot(V a, V b) { return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]; }
V turn(V v, V n) {
  V cross{n[1] * v[2] - n[2] * v[1], n[2] * v[0] - n[0] * v[2], n[0] * v[1] - n[1] * v[0]};
  const int d = dot(v, n);
  for (int i = 0; i < 3; ++i) v[i] = n[i] * d - cross[i];
  return v;
}
// 独立贴纸几何回放，不调用生产回放或 Kociemba 移动表。
std::string replay(std::string state, const std::vector<std::string>& moves) {
  for (const auto& move : moves) {
    const size_t face = face_order.find(move.at(0));
    need(face < 6, "非法面名称");
    const int count = move.size() == 1 ? 1 : move[1] == '2' ? 2 : 3;
    for (int t = 0; t < count; ++t) {
      std::string next(54, '?');
      for (int f = 0; f < 6; ++f)
        for (int r = 0; r < 3; ++r)
          for (int c = 0; c < 3; ++c) {
            V p = normal[f], n = normal[f];
            for (int a = 0; a < 3; ++a) p[a] += (r - 1) * down[f][a] + (c - 1) * right[f][a];
            if (dot(p, normal[face]) == 1) {
              p = turn(p, normal[face]);
              n = turn(n, normal[face]);
            }
            size_t target = 0;
            while (target < 6 && normal[target] != n) ++target;
            need(target < 6, "贴纸法向量错误");
            next[9 * target + 3 * (dot(p, down[target]) + 1) + dot(p, right[target]) + 1] =
                state[9 * f + 3 * r + c];
          }
      state = next;
    }
  }
  return state;
}
// 机械坐标：+X=R，-Y=F，+Z=U。所有基准从默认双闭合零腕起态开始。
struct Checked {
  std::vector<std::string> moves;
  double seconds = 0;
};
Checked check_primitives(const RobotPlan& plan, const PrimitiveCosts& costs) {
  std::array<V, 6> face{{{0, 0, 1}, {1, 0, 0}, {0, -1, 0}, {0, 0, -1}, {-1, 0, 0}, {0, 1, 0}}};
  std::array<int, 2> wrist{0, 0};
  std::array<bool, 2> closed{true, true};
  Checked checked;
  for (auto primitive : plan.actions) {
    int a = int(primitive);
    need(a >= 0 && a < 12, "非法原语编号");
    int h = a / 6, k = a % 6, o = 1 - h, mode = 0;
    if (k == 4) {
      need(closed[h] && closed[o], "打开动作失去支撑或重复打开");
      closed[h] = false;
    } else if (k == 5) {
      need(!closed[h] && !(wrist[h] % 2 && wrist[o] % 2), "非法闭合动作");
      closed[h] = true;
    } else {
      constexpr int delta[]{1, -1, 2, -2};
      const int d = delta[k];
      need(wrist[o] % 2 == 0, "旋转时另一腕横置");
      wrist[h] += d;
      need(std::abs(wrist[h]) <= 2, "腕角越界");
      if (!closed[h])
        mode = 2;
      else if (!closed[o]) {
        for (int t = 0; t < (d + 4) % 4; ++t)
          for (auto& v : face) {
            const auto old = v;
            v = h == 0 ? V{old[0], -old[2], old[1]} : V{-old[2], old[1], old[0]};
          }
      } else {
        mode = 1;
        const V axis = h == 0 ? V{1, 0, 0} : V{0, -1, 0};
        const auto it = std::find(face.begin(), face.end(), axis);
        need(it != face.end(), "找不到夹持面");
        checked.moves.push_back(std::string(1, face_order[it - face.begin()]) + (std::abs(d) == 2
                                                                                     ? "2"
                                                                                 : d > 0 ? "'"
                                                                                         : ""));
      }
    }
    checked.seconds += costs.duration[a][mode];
  }
  need(wrist == plan.end.wrist && closed == plan.end.closed, "终态腕角或开合错误");
  const auto& q = robot_orientations().at(plan.end.orientation);
  constexpr std::array<V, 6> initial{
      {{0, 0, 1}, {1, 0, 0}, {0, -1, 0}, {0, 0, -1}, {-1, 0, 0}, {0, 1, 0}}};
  for (int f = 0; f < 6; ++f)
    for (int i = 0; i < 3; ++i) {
      double v = 0;
      for (int j = 0; j < 3; ++j) v += q(i, j) * initial[f][j];
      need(std::abs(v - face[f][i]) < 1e-9, "终态朝向错误");
    }
  need(std::abs(checked.seconds - plan.execution_s) < 1e-8, "执行时间与独立成本累计不符");
  need(std::abs(plan.cost - checked.seconds) < 1e-8, "目标包含非执行时间成本");
  return checked;
}
std::vector<std::string> inverse(const std::vector<std::string>& moves) {
  std::vector<std::string> out;
  for (auto it = moves.rbegin(); it != moves.rend(); ++it)
    out.push_back(it->size() == 1 ? *it + "'" : (*it)[1] == '2' ? *it : it->substr(0, 1));
  return out;
}
struct Config {
  std::filesystem::path root, output;
  unsigned count = 12;
  uint32_t seed = 20261009;
  double budget = 500;
  std::string mode = "both";
  bool prepare = false;
};
unsigned integer(const std::string& value) {
  need(!value.empty() && value.find_first_not_of("0123456789") == std::string::npos,
       "需要非负整数");
  size_t used = 0;
  auto n = std::stoull(value, &used);
  need(used == value.size() && n <= UINT32_MAX, "整数越界");
  return unsigned(n);
}
Config arguments(int argc, char** argv) {
  Config c;
  for (int i = 1; i < argc; ++i) {
    std::string key = argv[i];
    if (key == "--prepare-only") {
      c.prepare = true;
      continue;
    }
    need(i + 1 < argc, "参数缺少值：" + key);
    std::string value = argv[++i];
    if (key == "--root")
      c.root = value;
    else if (key == "--output")
      c.output = value;
    else if (key == "--count") {
      c.count = integer(value);
      need(c.count <= 10000, "count超限");
    } else if (key == "--seed")
      c.seed = integer(value);
    else if (key == "--budget-ms") {
      c.budget = integer(value);
      need(c.budget <= 3600000, "预算超限");
    } else if (key == "--mode")
      c.mode = value;
    else
      throw std::runtime_error("未知参数：" + key);
  }
  need(!c.root.empty(), "需要 --root");
  need(c.mode == "both" || c.mode == "search" || c.mode == "optimize",
       "mode必须为both/search/optimize");
  return c;
}
}  // namespace
int main(int argc, char** argv) {
  try {
    if (argc == 2 && std::string(argv[1]) == "--help") {
      std::cout << "--root path --count 12 --budget-ms 500 --seed 20261009 --output path "
                   "[--mode both|search|optimize] [--prepare-only]\n"
                   "count为20步打乱数量；另加5个短边界。prepare-only只生成输入，不调用搜索。\n";
      return 0;
    }
    const Config c = arguments(argc, argv);
    std::ofstream file;
    if (!c.output.empty()) {
      need(!std::filesystem::exists(c.output), "输出已存在，拒绝覆盖：" + c.output.string());
      file.open(c.output);
      need(bool(file), "无法创建输出文件");
    }
    std::ostream& out = c.output.empty() ? std::cout : file;
    auto emit = [&](const Json& row) {
      out << row.dump() << '\n';
      out.flush();
      need(bool(out), "JSONL写入失败");
    };
    emit({{"type", "metadata"},
          {"schema", 1},
          {"revision", RM_BENCH_REVISION},
          {"seed", c.seed},
          {"count", c.count},
          {"budget_ms", c.budget},
          {"mode", c.mode},
          {"generator", "mt19937_u32_mod18_reject_same_face_v1"},
          {"objective", "execution_time"},
          {"wall_excluded_from_objective", true},
          {"timing", "search call only; input generation, verification and JSONL excluded"},
          {"warmup", false},
          {"cache_policy", "process-local caches retained; case order fixed"},
          {"root", c.root.string()},
          {"prepare_only", c.prepare}});
    std::vector<std::vector<std::string>> inputs{{}, {"R"}, {"F"}, {"R2"}, {"R'", "F"}};
    std::mt19937 rng(c.seed);
    for (unsigned i = 0; i < c.count; ++i) {
      std::vector<std::string> moves;
      while (moves.size() < 20) {
        // mt19937输出直接取模，避免各标准库的distribution实现差异。
        const unsigned n = rng() % 18;
        const char f = face_order[n / 3];
        if (!moves.empty() && moves.back()[0] == f) continue;
        moves.push_back(std::string(1, f) + (n % 3 == 0 ? "" : n % 3 == 1 ? "2" : "'"));
      }
      inputs.push_back(std::move(moves));
    }
    bool failed = false;
    for (size_t i = 0; i < inputs.size(); ++i) {
      const auto& scramble = inputs[i];
      const auto fixed = inverse(scramble);
      const auto state = replay_facelet_moves(solved, scramble);
      need(state == replay(solved, scramble) && replay(state, fixed) == solved, "输入独立回放失败");
      Json base = {{"case_id", i},           {"kind", i < 5 ? "boundary" : "scramble20"},
                   {"seed", c.seed},         {"budget_ms", c.budget},
                   {"scramble", scramble},   {"facelets", state},
                   {"fixed_solution", fixed}};
      if (c.prepare) {
        base["type"] = "input";
        emit(base);
        continue;
      }
      for (const std::string mode : {"search", "optimize"}) {
        if (c.mode != "both" && c.mode != mode) continue;
        Json row = base;
        row["type"] = "result";
        row["api"] = mode;
        row["found"] = false;
        row["cost"] = nullptr;
        row["steps"] = nullptr;
        row["verified_solved"] = nullptr;
        row["candidates"] = mode == "optimize" ? Json(1) : Json(nullptr);
        SearchOptions options;
        options.max_search_ms = c.budget;
        const auto began = Clock::now();
        try {
          std::optional<RobotPlan> plan;
          if (mode == "search") {
            auto r = search_robot_solution(state, {}, options, c.root);
            row["search_ms"] = r.search_ms;
            row["candidates"] = r.candidates;
            row["optimized_candidates"] = r.optimized_candidates;
            row["stop_reason"] = r.stop_reason;
            if (r.found) plan = std::move(r.plan);
          } else {
            auto deadline = began + std::chrono::duration_cast<Clock::duration>(
                                        std::chrono::duration<double, std::milli>(c.budget));
            plan = optimize_robot_moves(fixed, {}, options, deadline);
          }
          const double measured =
              std::chrono::duration<double, std::milli>(Clock::now() - began).count();
          row["call_wall_ms"] = measured;
          if (mode == "optimize") {
            row["search_ms"] = measured;
            row["stop_reason"] = plan ? plan->stop_reason : "no_plan";
          }
          row["found"] = bool(plan);
          if (plan) {
            row["cost"] = plan->cost;
            row["steps"] = plan->actions.size();
            row["execution_s"] = plan->execution_s;
            row["expanded"] = plan->expanded;
            row["optimal_for_sequence"] = plan->optimal_for_sequence;
            row["actions"] = Json::array();
            for (auto a : plan->actions) row["actions"].push_back(int(a));
            row["verified_solved"] = false;
            const auto checked = check_primitives(*plan, options.costs);
            row["primitive_solution"] = checked.moves;
            row["verified_solved"] = replay(state, checked.moves) == solved;
            need(row["verified_solved"].get<bool>(), "返回原语未独立还原");
          }
        } catch (const std::exception& e) {
          row["error"] = e.what();
          failed = true;
          if (!row.contains("search_ms"))
            row["search_ms"] =
                std::chrono::duration<double, std::milli>(Clock::now() - began).count();
        }
        emit(row);
      }
    }
    return failed ? 1 : 0;
  } catch (const std::exception& e) {
    std::cerr << "基准失败：" << e.what() << '\n';
    return 1;
  }
}
