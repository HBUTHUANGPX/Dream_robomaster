#include "rm/robot_search.hpp"

#include <queue>

#include "rm/solution_candidates.hpp"
namespace rm::cube {
namespace {
using Clock = std::chrono::steady_clock;
constexpr int states = 1800;
struct Edge {
  int state = -1;
  char face = 0;
  int count = 0;
  std::string mode;
};
using Graph = std::array<std::array<Edge, 12>, states>;
const Graph* graph(Clock::time_point deadline) {
  static std::optional<Graph> cached;
  if (cached) return &*cached;
  Graph next;
  for (int s = 0; s < states; ++s) {
    if (s % 16 == 0 && Clock::now() >= deadline) return nullptr;
    auto state = robot_state_from_id(s);
    if (std::abs(state.wrist[0]) % 2 && std::abs(state.wrist[1]) % 2) continue;
    for (int a = 0; a < 12; ++a)
      if (auto t = primitive_transition(state, Primitive(a))) {
        Edge e;
        e.state = robot_state_id(t->state);
        e.mode = t->mode;
        if (!t->move.empty()) {
          auto [f, c] = parse_move(t->move);
          e.face = f;
          e.count = (c + 4) % 4;
        }
        next[s][a] = std::move(e);
      }
  }
  cached = std::move(next);
  return &*cached;
}
void validate_options(const SearchOptions& o) {
  if (!std::isfinite(o.max_search_ms) || o.max_search_ms < 0 || o.max_search_ms > 3600000)
    throw std::invalid_argument("max_search_ms must be in [0,3600000]");
  if (o.memory_limit_mb < 4 || o.memory_limit_mb > 4096)
    throw std::invalid_argument("memory_limit_mb must be in [4,4096]");
  for (const auto& row : o.costs.duration)
    for (double d : row)
      if (!std::isfinite(d) || d < 0 || d > 3600)
        throw std::invalid_argument("Invalid primitive duration");
}
}  // namespace
SearchOptions search_options(const Json& args, const PrimitiveCosts& costs) {
  SearchOptions o;
  o.costs = costs;
  o.max_search_ms = args.value("max_search_ms", 1000.);
  auto obj = args.value("objective", std::string("execution_time"));
  if (obj != "execution_time" && obj != "action_count")
    throw std::invalid_argument("objective must be execution_time or action_count");
  o.action_count = obj == "action_count";
  auto terminal = args.value("terminal_policy", std::string("stable"));
  if (terminal != "stable" && terminal != "home")
    throw std::invalid_argument("terminal_policy must be stable or home");
  o.home = terminal == "home";
  if (args.contains("memory_limit_mb")) {
    if (!args["memory_limit_mb"].is_number_integer())
      throw std::invalid_argument("memory_limit_mb must be integer");
    auto m = args["memory_limit_mb"].get<long long>();
    if (m < 4 || m > 4096) throw std::invalid_argument("memory_limit_mb outside range");
    o.memory_limit_mb = size_t(m);
  }
  if (args.contains("cost_profile")) o.costs.apply_json(args["cost_profile"]);
  if (args.contains("overlap"))
    throw std::invalid_argument("Overlapping execution is not supported");
  validate_options(o);
  return o;
}
std::optional<RobotPlan> optimize_robot_moves(const std::vector<std::string>& moves,
                                              const RobotState& start, const SearchOptions& o,
                                              Clock::time_point deadline, double upper_bound) {
  validate_options(o);
  int initial = robot_state_id(start);
  if (moves.size() > 1000) throw std::invalid_argument("At most 1000 face moves");
  std::vector<std::pair<char, int>> expected;
  for (auto& m : moves) {
    auto [f, c] = parse_move(m);
    expected.emplace_back(f, (c + 4) % 4);
  }
  auto goal = [&](int state) {
    auto s = robot_state_from_id(state);
    return !o.home ||
           (s.wrist == std::array<int, 2>{0, 0} && s.closed == std::array<bool, 2>{true, true});
  };
  if (moves.empty() && goal(initial)) {
    RobotPlan p;
    p.end = start;
    p.optimal_for_sequence = true;
    p.stop_reason = "optimal";
    return p;
  }
  if (Clock::now() >= deadline) return {};
  const Graph* g = graph(deadline);
  if (!g) return {};
  // 每个面动作保留模4的已完成转角，允许一个180°拆成两个90°。
  size_t n = (moves.size() * 4 + 1) * states, limit = o.memory_limit_mb * 1024 * 1024;
  struct Label {
    double cost = std::numeric_limits<double>::infinity();
    int parent = -1, action = -1, steps = 0;
  };
  if (n * sizeof(Label) + sizeof(Graph) > limit)
    throw std::runtime_error("Robot search memory limit reached");
  double min_face = o.action_count ? 1 : std::numeric_limits<double>::infinity();
  if (!o.action_count)
    for (int a = 0; a < 12; ++a)
      if (a % 6 < 4) min_face = std::min(min_face, o.costs.seconds(Primitive(a), "face"));
  std::vector<Label> labels(n);
  struct Item {
    double cost;
    int steps, node;
    bool operator>(const Item& b) const {
      if (cost != b.cost) return cost > b.cost;
      if (steps != b.steps) return steps > b.steps;
      return node > b.node;
    }
  };
  std::priority_queue<Item, std::vector<Item>, std::greater<Item>> queue;
  labels[initial] = {0, -1, -1, 0};
  queue.push({0, 0, initial});
  size_t expanded = 0, iterations = 0;
  std::optional<RobotPlan> incumbent;
  auto remember = [&](int node) {
    RobotPlan p;
    p.end = robot_state_from_id(node % states);
    p.cost = labels[node].cost;
    p.expanded = expanded;
    for (int at = node; labels[at].parent >= 0; at = labels[at].parent)
      p.actions.push_back(Primitive(labels[at].action));
    std::reverse(p.actions.begin(), p.actions.end());
    RobotState s = start;
    for (auto action : p.actions) {
      auto t = *primitive_transition(s, action);
      p.execution_s += o.costs.seconds(action, t.mode);
      s = t.state;
    }
    if (!incumbent || p.cost < incumbent->cost ||
        (p.cost == incumbent->cost && p.actions.size() < incumbent->actions.size()))
      incumbent = std::move(p);
  };
  auto stopped = [&](const std::string& reason) {
    if (incumbent) {
      incumbent->stop_reason = reason;
      incumbent->expanded = expanded;
    }
    return incumbent;
  };
  while (!queue.empty()) {
    if ((iterations++ & 63) == 0 && Clock::now() >= deadline) return stopped("deadline");
    auto item = queue.top();
    queue.pop();
    auto& label = labels[item.node];
    if (item.cost != label.cost || item.steps != label.steps) continue;
    ++expanded;
    int state = item.node % states, stage = item.node / states, progress = stage / 4,
        partial = stage % 4;
    if (size_t(progress) == moves.size() && goal(state)) {
      remember(item.node);
      incumbent->optimal_for_sequence = true;
      incumbent->stop_reason = "optimal";
      incumbent->expanded = expanded;
      return incumbent;
    }
    for (int a = 0; a < 12; ++a) {
      const Edge& e = (*g)[state][a];
      if (e.state < 0) continue;
      int next_stage = stage;
      if (e.face) {
        if (size_t(progress) >= expected.size() || e.face != expected[progress].first) continue;
        int sum = (partial + e.count) % 4;
        next_stage = sum == expected[progress].second ? (progress + 1) * 4 : progress * 4 + sum;
      }
      int next = next_stage * states + e.state;
      double cost = item.cost + (o.action_count ? 1 : o.costs.seconds(Primitive(a), e.mode));
      int steps = item.steps + 1;
      auto& to = labels[next];
      if (cost + (moves.size() - size_t(next_stage / 4)) * min_face > upper_bound ||
          cost > to.cost || (cost == to.cost && steps >= to.steps))
        continue;
      to = {cost, item.node, a, steps};
      if (size_t(next_stage / 4) == moves.size() && goal(e.state)) remember(next);
      queue.push({cost, steps, next});
      if (labels.size() * sizeof(Label) + queue.size() * sizeof(Item) * 2 + sizeof(Graph) > limit) {
        if (incumbent) return stopped("memory_limit");
        throw std::runtime_error("Robot search memory limit reached");
      }
    }
  }
  return {};
}
RobotSearchResult search_robot_solution(const std::string& state, const RobotState& start,
                                        const SearchOptions& o, const std::filesystem::path& root) {
  auto began = Clock::now();
  validate_options(o);
  robot_state_id(start);
  validate_facelets(state);
  auto deadline = began + std::chrono::duration_cast<Clock::duration>(
                              std::chrono::duration<double, std::milli>(o.max_search_ms));
  RobotSearchResult result;
  result.start = start;
  auto consider = [&](const std::vector<std::string>& moves) {
    ++result.candidates;
    if (replay_facelet_moves(state, moves) != solved)
      throw std::runtime_error("Candidate failed independent facelet replay");
    auto p = optimize_robot_moves(
        moves, start, o, deadline,
        result.found ? result.plan.cost : std::numeric_limits<double>::infinity());
    if (p) {
      if (p->stop_reason == "memory_limit") result.stop_reason = "memory_limit";
      ++result.optimized_candidates;
      if (!result.found || p->cost < result.plan.cost ||
          (p->cost == result.plan.cost && p->actions.size() < result.plan.actions.size())) {
        if (replay_facelet_moves(state, primitive_moves(p->actions, start)) != solved)
          throw std::runtime_error("Primitive solution failed independent replay");
        result.found = true;
        result.plan = std::move(*p);
        result.improvements.push_back(
            {{"search_ms", std::chrono::duration<double, std::milli>(Clock::now() - began).count()},
             {"action_count", result.plan.actions.size()},
             {"execution_s", result.plan.execution_s}});
      }
    }
    return result.stop_reason != "memory_limit" && Clock::now() < deadline &&
           (!result.found || result.plan.cost > 0);
  };
  try {
    if (state == solved)
      consider({});
    else if (Clock::now() < deadline)
      enumerate_solutions(state, root, deadline, [&](const std::vector<std::string>& moves) {
        if (!consider(moves)) return false;
        // 对立面可交换，但机器人的执行代价可能不同。
        for (size_t i = 1; i < moves.size(); ++i) {
          auto [a, sa] = face_axis(moves[i - 1][0]);
          auto [b, sb] = face_axis(moves[i][0]);
          if (a == b && sa != sb) {
            auto variant = moves;
            std::swap(variant[i - 1], variant[i]);
            if (!consider(variant)) return false;
          }
        }
        return true;
      });
    if (result.stop_reason.empty())
      result.stop_reason = Clock::now() >= deadline ? "deadline" : "candidate_search_finished";
  } catch (const std::runtime_error& e) {
    if (std::string(e.what()) != "Robot search memory limit reached") throw;
    result.stop_reason = "memory_limit";
  }
  result.search_ms = std::chrono::duration<double, std::milli>(Clock::now() - began).count();
  return result;
}
Json RobotSearchResult::json(const SearchOptions& o) const {
  Json r = {{"found", found},
            {"objective", o.action_count ? "action_count" : "execution_time"},
            {"search_ms", search_ms},
            {"max_search_ms", o.max_search_ms},
            {"stop_reason", stop_reason},
            {"candidates", candidates},
            {"optimized_candidates", optimized_candidates},
            {"improvements", improvements},
            {"optimality", "best_found"},
            {"fixed_sequence_optimal", found && plan.optimal_for_sequence},
            {"global_optimal", false},
            {"terminal_policy", o.home ? "home" : "stable"},
            {"start", robot_state_json(start)},
            {"cost_profile", o.costs.json()},
            {"overlap_supported", false},
            {"mechanical_domain", "other_wrist_even_quarters_during_rotation"},
            {"search_time_in_objective", false}};
  if (found) {
    r["actions"] = primitive_plan_json(plan.actions, start, o.costs);
    r["action_count"] = plan.actions.size();
    r["estimated_execution_s"] = plan.execution_s;
    r["cost"] = plan.cost;
    r["end"] = robot_state_json(plan.end);
    r["solution"] = primitive_moves(plan.actions, start);
    r["expanded"] = plan.expanded;
  }
  return r;
}
}  // namespace rm::cube
