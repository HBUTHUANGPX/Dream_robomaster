#include <array>
#include <cmath>
#include <deque>
#include <iostream>
#include <map>
#include <tuple>

#include "rm/robot_search.hpp"

namespace {
using namespace rm::cube;
using Clock = std::chrono::steady_clock;
using V = std::array<int, 3>;
constexpr std::array<V, 6> normals{
    {{1, 0, 0}, {-1, 0, 0}, {0, 0, 1}, {0, 0, -1}, {0, -1, 0}, {0, 1, 0}}};
constexpr char face_names[] = "RLUDFB";
void need(bool ok, const std::string& message) {
  if (!ok) throw std::runtime_error(message);
}
// 直接保存六个面的世界法向量；不使用生产状态编号、旋转函数或动作图。
struct State {
  std::array<V, 6> face = normals;
  std::array<int, 2> wrist{0, 0};
  std::array<bool, 2> closed{true, true};
  size_t progress = 0;
  int partial = 0;
  auto operator<=>(const State&) const = default;
};
State import_state(const RobotState& s) {
  State r;
  r.wrist = s.wrist;
  r.closed = s.closed;
  // 接口边界只读取朝向矩阵；参考图内部始终使用整数几何。
  const auto& q = robot_orientations().at(s.orientation);
  for (size_t f = 0; f < 6; ++f)
    for (int i = 0; i < 3; ++i) {
      r.face[f][i] = 0;
      for (int j = 0; j < 3; ++j) r.face[f][i] += int(std::lround(q(i, j))) * normals[f][j];
    }
  return r;
}
bool terminal(const State& s, size_t count, bool home) {
  return s.progress == count && (!home || (s.wrist == std::array<int, 2>{0, 0} &&
                                           s.closed == std::array<bool, 2>{true, true}));
}
struct Edge {
  State state;
  int mode;
};
std::optional<Edge> step(State s, int action, const std::vector<std::string>& target) {
  const int hand = action / 6, kind = action % 6, other = 1 - hand;
  int mode = 0;
  if (kind == 4) {
    if (!s.closed[hand] || !s.closed[other]) return {};
    s.closed[hand] = false;
  } else if (kind == 5) {
    if (s.closed[hand] || (s.wrist[hand] % 2 != 0 && s.wrist[other] % 2 != 0)) return {};
    s.closed[hand] = true;
  } else {
    if (s.wrist[other] % 2 != 0) return {};
    constexpr int delta[]{1, -1, 2, -2};
    const int d = delta[kind], next = s.wrist[hand] + d;
    if (next < -2 || next > 2) return {};
    s.wrist[hand] = next;
    if (!s.closed[hand])
      mode = 2;
    else if (!s.closed[other]) {
      // 右手系：A 绕 +X，B 绕 -Y；每次只做整数的正90°置换。
      for (int n = 0; n < (d + 4) % 4; ++n)
        for (auto& v : s.face) {
          const auto old = v;
          v = hand == 0 ? V{old[0], -old[2], old[1]} : V{-old[2], old[1], old[0]};
        }
    } else {
      if (s.wrist[other] % 2 != 0 || s.progress == target.size()) return {};
      mode = 1;
      const V axis = hand == 0 ? V{1, 0, 0} : V{0, -1, 0};
      size_t f = 0;
      while (f < 6 && s.face[f] != axis) ++f;
      need(f < 6, "参考几何缺少夹持面");
      const auto& move = target[s.progress];
      if (face_names[f] != move[0]) return {};
      const int wanted = move.size() == 1 ? 1 : move[1] == '2' ? 2 : 3;
      s.partial = (s.partial - d + 4) % 4;
      if (s.partial == wanted) {
        ++s.progress;
        s.partial = 0;
      }
    }
  }
  return Edge{s, mode};
}
// FIFO 标签修正法：没有优先队列、生产邻接图或生产进度编号。
// 非负成本与严格改善保证零成本环也会终止。
double reference(const std::vector<std::string>& moves, const RobotState& start,
                 const SearchOptions& o) {
  const State initial = import_state(start);
  std::map<State, double> distance{{initial, 0}};
  std::deque<State> work{initial};
  double best = std::numeric_limits<double>::infinity();
  while (!work.empty()) {
    const State s = work.front();
    work.pop_front();
    const double cost = distance.at(s);
    if (terminal(s, moves.size(), o.home)) {
      best = std::min(best, cost);
      continue;
    }
    if (cost >= best) continue;
    for (int a = 0; a < 12; ++a)
      if (auto e = step(s, a, moves)) {
        const double next = cost + (o.action_count ? 1 : o.costs.duration[a][e->mode]);
        auto [it, inserted] = distance.emplace(e->state, next);
        if (inserted || next < it->second) {
          it->second = next;
          work.push_back(e->state);
        }
      }
  }
  return best;
}
RobotPlan compare(const std::string& name, const std::vector<std::string>& moves,
                  const RobotState& start, const SearchOptions& o) {
  const double expected = reference(moves, start, o);
  auto p = optimize_robot_moves(moves, start, o, Clock::now() + std::chrono::seconds(10));
  need(p.has_value(), name + "：未返回解");
  need(std::abs(p->cost - expected) < 1e-9, name + "：不是参考最短路径");
  State s = import_state(start);
  double elapsed = 0;
  for (auto a : p->actions) {
    auto e = step(s, int(a), moves);
    need(e.has_value(), name + "：独立重放发现非法动作");
    elapsed += o.costs.duration[int(a)][e->mode];
    s = e->state;
  }
  need(terminal(s, moves.size(), o.home), name + "：未达到终态");
  auto end = import_state(p->end);
  end.progress = s.progress;
  end.partial = s.partial;
  need(end == s, name + "：报告终态与独立重放不符");
  need(std::abs(elapsed - p->execution_s) < 1e-9, name + "：执行时间错误");
  need(std::abs(p->cost - (o.action_count ? double(p->actions.size()) : elapsed)) < 1e-9,
       name + "：目标成本错误");
  return *p;
}
template <class F>
void rejects(F f) {
  bool caught = false;
  try {
    f();
  } catch (const std::invalid_argument&) {
    caught = true;
  }
  need(caught, "未拒绝非法输入");
}
}  // namespace

int main(int argc, char** argv) {
  try {
    State collision;
    collision.closed = {false, true};
    collision.wrist = {0, 1};
    for (int a = 0; a < 4; ++a) need(!step(collision, a, {}), "闭合横置支撑腕未阻止空转");
    collision.closed = {true, false};
    for (int a = 0; a < 4; ++a) need(!step(collision, a, {}), "打开横置腕未阻止整块旋转");
    SearchOptions o;
    const std::vector<std::vector<std::string>> targets{{"R"}, {"F"}, {"R2"}, {"R'", "F"}};
    const std::array<RobotState, 3> starts{
        {{}, {0, {1, -2}, {true, true}}, {0, {-2, 0}, {false, true}}}};
    size_t cases = 0;
    for (bool count : {false, true})
      for (bool home : {false, true}) {
        o.action_count = count;
        o.home = home;
        for (const auto& start : starts)
          for (const auto& moves : targets) {
            compare("默认成本 " + std::to_string(++cases), moves, start, o);
          }
      }
    compare("打开横置腕起态", {"R"}, RobotState{0, {0, 1}, {true, false}}, o);
    compare("非单位朝向", {"R'", "F"}, RobotState{7, {0, 0}, {true, true}}, o);
    o = SearchOptions{};
    for (int a = 0; a < 12; ++a)
      for (int m = 0; m < 3; ++m) o.costs.duration[a][m] = double((a * 7 + m * 3) % 11) / 8;
    for (bool count : {false, true})
      for (bool home : {false, true}) {
        o.action_count = count;
        o.home = home;
        for (const auto& moves : targets) compare("非对称含零成本", moves, starts[1], o);
      }
    o = SearchOptions{};
    for (auto& row : o.costs.duration) row.fill(10);
    o.costs.duration[int(Primitive::A_N90)][1] = 0.125;
    auto split = compare("180拆为两次90", {"R2"}, {}, o);
    need(split.actions == std::vector<Primitive>{Primitive::A_N90, Primitive::A_N90},
         "未选择唯一便宜拆分");
    o.action_count = true;
    need(compare("动作数目标", {"R2"}, {}, o).actions.size() == 1, "动作数目标仍选择拆分");
    o.action_count = false;
    for (auto action : {Primitive::A_P180, Primitive::A_N180}) {
      o.costs.duration[int(action)][1] = 0;
      need(compare("180方向", {"R2"}, {}, o).actions == std::vector<Primitive>{action},
           "180方向选择错误");
      o.costs.duration[int(action)][1] = 10;
    }
    for (auto& row : o.costs.duration) row.fill(0);
    o.home = true;
    compare("零成本环", {"R'", "F"}, {}, o);
    compare("空目标归位", {}, starts[1], o);
    o.home = false;
    need(compare("稳定终态保留非零腕", {}, starts[1], o).actions.empty(), "稳定终态强制归位");
    need(!optimize_robot_moves({"R"}, {}, o, Clock::now()), "截止时间已到仍搜索");
    o = SearchOptions{};
    o.memory_limit_mb = 4;
    bool memory = false;
    try {
      optimize_robot_moves(std::vector<std::string>(100, "R"), {}, o,
                           Clock::now() + std::chrono::seconds(10));
    } catch (const std::runtime_error& e) {
      memory = std::string(e.what()) == "Robot search memory limit reached";
    }
    need(memory, "内存限制没有生效");
    compare("内存失败后状态隔离", {"R"}, {}, o);
    if (argc > 1) {
      const std::filesystem::path root = argv[1];
      o.max_search_ms = 0;
      auto result = search_robot_solution(solved, {}, o, root);
      need(result.found && result.plan.actions.empty() && result.plan.cost == 0,
           "零预算丢失已还原解");
      // 保持颜色计数的合法 R 转后状态，直接给出面串，不使用生产回放构造输入。
      const std::string scrambled = "UUFUUFUUFRRRRRRRRRFFDFFDFFDDDBDDBDDBLLLLLLLLLUBBUBBUBB";
      result = search_robot_solution(scrambled, {}, o, root);
      need(!result.found && result.stop_reason == "deadline", "零预算未报告超时");
      rejects([&] { search_robot_solution("invalid", {}, o, root); });
      rejects([&] { search_robot_solution(solved, RobotState{0, {3, 0}, {true, true}}, o, root); });
      rejects(
          [&] { search_robot_solution(solved, RobotState{0, {0, 0}, {false, false}}, o, root); });
      rejects([&] { search_robot_solution(solved, RobotState{0, {1, 1}, {true, true}}, o, root); });
      // 4MB 限制不能抹掉无需搜索空间的已知空解。
      result = search_robot_solution(solved, {}, o, root);
      need(result.found && result.improvements.size() == 1, "内存限制丢失已知解记录");
    }
    std::cout << "独立最短路测试通过：默认成本48例、非对称成本16例及边界例。"
              << (argc > 1 ? "入口测试已执行。\n" : "未提供root，入口测试未执行。\n");
  } catch (const std::exception& e) {
    std::cerr << "失败：" << e.what() << '\n';
    return 1;
  }
}
