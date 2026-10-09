#include "rm/robot_search.hpp"

#include <mutex>
#include <queue>
#include <thread>
#include <unordered_set>

#include "rm/solution_candidates.hpp"
#include "rm/solution_variants.hpp"
namespace rm::cube {
namespace {
using Clock = std::chrono::steady_clock;
constexpr int states = 1800;
struct Edge {
  int state = -1;
  char face = 0;
  int count = 0;
  int mode = 0;
};
using Graph = std::array<std::array<Edge, 12>, states>;
const Graph* graph(Clock::time_point deadline) {
  static std::optional<Graph> cached;
  static std::atomic<const Graph*> published{nullptr};
  static std::mutex mutex;
  if (auto ready = published.load(std::memory_order_acquire)) return ready;
  std::unique_lock lock(mutex, std::defer_lock);
  while (!lock.try_lock()) {
    auto now = Clock::now();
    if (now >= deadline) return nullptr;
    std::this_thread::sleep_until(std::min(deadline, now + std::chrono::microseconds(50)));
  }
  if (auto ready = published.load(std::memory_order_acquire)) return ready;
  Graph next;
  for (int s = 0; s < states; ++s) {
    if (s % 16 == 0 && Clock::now() >= deadline) return nullptr;
    auto state = robot_state_from_id(s);
    if (std::abs(state.wrist[0]) % 2 && std::abs(state.wrist[1]) % 2) continue;
    for (int a = 0; a < 12; ++a)
      if (auto t = primitive_transition(state, Primitive(a))) {
        Edge e;
        e.state = robot_state_id(t->state);
        e.mode = t->mode == "face" ? 1 : t->mode == "empty" ? 2 : 0;
        if (!t->move.empty()) {
          auto [f, c] = parse_move(t->move);
          e.face = f;
          e.count = (c + 4) % 4;
        }
        next[s][a] = std::move(e);
      }
  }
  cached = std::move(next);
  published.store(&*cached, std::memory_order_release);
  return &*cached;
}
// 合并同奇偶的有符号腕角，保留朝向、夹爪和横腕约束。
// 忽略腕行程只会降低成本；实际搜索和执行仍使用完整机械状态。
constexpr int relaxed_states = 216;
int relaxed_state(int state) {
  const int a = std::abs(state / 15 % 5 - 2) % 2;
  const int b = std::abs(state / 3 % 5 - 2) % 2;
  return (state / 75 * 3 + state % 3) * 3 + (a ? 1 : b ? 2 : 0);
}
struct Incoming {
  int source, action;
  Edge edge;
};
using IncomingGraph = std::array<std::vector<Incoming>, relaxed_states>;
const IncomingGraph& relaxed_graph(const Graph& g) {
  static const IncomingGraph incoming = [&] {
    IncomingGraph out;
    std::array<std::array<bool, 12>, relaxed_states> seen{};
    for (int s = 0; s < states; ++s)
      for (int a = 0; a < 12; ++a) {
        const auto& e = g[s][a];
        const int source = relaxed_state(s);
        if (e.state >= 0 && !seen[source][a]) {
          seen[source][a] = true;
          out[relaxed_state(e.state)].push_back({source, a, e});
        }
      }
    return out;
  }();
  return incoming;
}
std::optional<std::vector<double>> suffix_bounds(const Graph& graph,
                                                 const std::vector<std::pair<char, int>>& moves,
                                                 const SearchOptions& options,
                                                 Clock::time_point deadline,
                                                 size_t available_memory,
                                                 const SearchFeedback* feedback) {
  const auto& incoming = relaxed_graph(graph);
  const size_t count = (moves.size() * 4 + 1) * relaxed_states;
  std::vector<double> distance(count, std::numeric_limits<double>::infinity());
  using Item = std::pair<double, int>;
  std::priority_queue<Item, std::vector<Item>, std::greater<Item>> work;
  for (int s = 0; s < relaxed_states; ++s)
    if (!options.home || s % 9 == 0) {
      int node = int(moves.size()) * 4 * relaxed_states + s;
      distance[node] = 0;
      work.push({0, node});
    }
  size_t iterations = 0;
  while (!work.empty()) {
    if ((iterations++ & 63) == 0 &&
        (Clock::now() >= deadline || (feedback && feedback->cancel.load())))
      return {};
    const auto [cost, node] = work.top();
    work.pop();
    if (cost != distance[node]) continue;
    int stage = node / relaxed_states, progress = stage / 4, partial = stage % 4;
    for (const auto& in : incoming[node % relaxed_states]) {
      const auto& e = in.edge;
      const double next_cost =
          cost + (options.action_count ? 1 : options.costs.duration[in.action][e.mode]);
      auto relax = [&](int source_stage) {
        int source = source_stage * relaxed_states + in.source;
        if (next_cost < distance[source]) {
          distance[source] = next_cost;
          work.push({next_cost, source});
        }
      };
      if (!e.face) {
        relax(stage);
      } else {
        // 尚未完成当前面时的反向边，以及恰好完成上一个面的反向边。
        if (size_t(progress) < moves.size() && e.face == moves[progress].first &&
            partial != moves[progress].second)
          relax(progress * 4 + (partial - e.count + 4) % 4);
        if (progress > 0 && partial == 0 && e.face == moves[progress - 1].first)
          relax((progress - 1) * 4 + (moves[progress - 1].second - e.count + 4) % 4);
      }
    }
    if (distance.size() * sizeof(double) + work.size() * sizeof(Item) * 2 > available_memory)
      throw std::runtime_error("Robot search memory limit reached");
  }
  return distance;
}
void validate_options(const SearchOptions& o) {
  if (o.threads < 1 || o.threads > 3) throw std::invalid_argument("threads must be in [1,3]");
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
  if (args.contains("threads")) {
    if (!args["threads"].is_number_integer())
      throw std::invalid_argument("threads must be integer");
    auto n = args["threads"].get<long long>();
    if (n < 1 || n > 3) throw std::invalid_argument("threads must be in [1,3]");
    o.threads = int(n);
  }
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
                                              Clock::time_point deadline, double upper_bound,
                                              const SearchFeedback* feedback) {
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
  if (Clock::now() >= deadline || (feedback && feedback->cancel.load())) return {};
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
  const auto& incoming = relaxed_graph(*g);
  size_t incoming_bytes = sizeof(IncomingGraph);
  for (const auto& row : incoming) incoming_bytes += row.capacity() * sizeof(Incoming);
  const size_t graph_bytes = sizeof(Graph) + incoming_bytes;
  const size_t bound_bytes = (moves.size() * 4 + 1) * relaxed_states * sizeof(double);
  const size_t base_bytes = n * sizeof(Label) + graph_bytes + bound_bytes;
  if (base_bytes > limit) throw std::runtime_error("Robot search memory limit reached");
  auto bounds =
      suffix_bounds(*g, expected, o, deadline, limit - n * sizeof(Label) - graph_bytes, feedback);
  if (!bounds) return {};
  auto h = [&](int node) {
    return (*bounds)[node / states * relaxed_states + relaxed_state(node % states)];
  };
  std::vector<Label> labels(n);
  struct Item {
    double cost, estimate;
    int steps, node;
    bool operator>(const Item& b) const {
      if (estimate != b.estimate) return estimate > b.estimate;
      if (cost != b.cost) return cost > b.cost;
      if (steps != b.steps) return steps > b.steps;
      return node > b.node;
    }
  };
  std::priority_queue<Item, std::vector<Item>, std::greater<Item>> queue;
  labels[initial] = {0, -1, -1, 0};
  queue.push({0, h(initial), 0, initial});
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
    if ((iterations++ & 63) == 0) {
      if (Clock::now() >= deadline) return stopped("deadline");
      if (feedback) {
        if (feedback->cancel.load()) return stopped("cancelled");
        upper_bound = std::min(upper_bound, feedback->upper_bound.load());
      }
    }
    auto item = queue.top();
    queue.pop();
    auto& label = labels[item.node];
    if (item.cost != label.cost || item.steps != label.steps) continue;
    ++expanded;
    int state = item.node % states, stage = item.node / states, progress = stage / 4,
        partial = stage % 4;
    if (size_t(progress) == moves.size() && goal(state)) {
      remember(item.node);
      incumbent->optimal_for_sequence = incumbent->cost <= upper_bound;
      incumbent->stop_reason = incumbent->optimal_for_sequence ? "optimal" : "bound";
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
      double cost = item.cost + (o.action_count ? 1 : o.costs.duration[a][e.mode]);
      int steps = item.steps + 1;
      auto& to = labels[next];
      const double estimate = cost + h(next);
      // 正反向浮点求和的误差随路径长度和成本尺度增长；只放宽剪枝，不修改真实成本。
      const double slack = 8 * std::numeric_limits<double>::epsilon() * double(n) *
                           std::max({1., std::abs(estimate), std::abs(upper_bound)});
      if (!std::isfinite(estimate) ||
          (std::isfinite(upper_bound) && estimate - upper_bound > slack) || cost > to.cost ||
          (cost == to.cost && steps >= to.steps))
        continue;
      to = {cost, item.node, a, steps};
      if (size_t(next_stage / 4) == moves.size() && goal(e.state)) remember(next);
      queue.push({cost, cost + h(next), steps, next});
      if (base_bytes + queue.size() * sizeof(Item) * 2 > limit) {
        if (incumbent) return stopped("memory_limit");
        throw std::runtime_error("Robot search memory limit reached");
      }
    }
  }
  return {};
}
RobotSearchResult search_robot_solution(const std::string& state, const RobotState& start,
                                        const SearchOptions& o, const std::filesystem::path& root) {
  const auto began = Clock::now();
  validate_options(o);
  robot_state_id(start);
  validate_facelets(state);
  const auto deadline = began + std::chrono::duration_cast<Clock::duration>(
                                    std::chrono::duration<double, std::milli>(o.max_search_ms));
  RobotSearchResult result;
  result.start = start;
  SearchFeedback feedback;
  std::mutex mutex;
  std::unordered_set<std::string> seen;
  std::exception_ptr error;
  std::atomic<bool> memory_limited{false};
  struct Worker {
    std::vector<size_t> frames;
    size_t candidates = 0, optimized = 0;
    bool memory_limited = false;
  };
  std::vector<Worker> workers;
  auto running = [&] { return !feedback.cancel.load() && Clock::now() < deadline; };
  auto publish = [&](RobotPlan p) {
    // 先独立检查，再在短临界区内发布；没有搜索队列或 MuJoCo 对象进入临界区。
    if (replay_facelet_moves(state, primitive_moves(p.actions, start)) != solved)
      throw std::runtime_error("Primitive solution failed independent replay");
    std::lock_guard lock(mutex);
    if (!result.found || p.cost < result.plan.cost ||
        (p.cost == result.plan.cost && p.actions.size() < result.plan.actions.size())) {
      result.found = true;
      result.plan = std::move(p);
      feedback.upper_bound.store(result.plan.cost);
      result.improvements.push_back(
          {{"search_ms", std::chrono::duration<double, std::milli>(Clock::now() - began).count()},
           {"action_count", result.plan.actions.size()},
           {"execution_s", result.plan.execution_s}});
      if (result.plan.cost == 0) feedback.cancel.store(true);
    }
  };
  try {
    if (state == solved) {
      result.threads_used = 1;
      result.worker_memory_limit_mb = o.memory_limit_mb;
      result.candidates = 1;
      if (auto p = optimize_robot_moves({}, start, o, deadline)) {
        result.optimized_candidates = 1;
        publish(std::move(*p));
      }
    } else if (running()) {
      // 表校验和图发布在创建线程前完成；随后工作线程只读共享数据。
      auto tables = prepare_candidate_tables(root, deadline);
      const auto* g = tables && running() ? graph(deadline) : nullptr;
      if (g && running()) relaxed_graph(*g);
      result.preparation_ms =
          std::chrono::duration<double, std::milli>(Clock::now() - began).count();
      if (g && running()) {
        const unsigned hardware = std::max(1u, std::thread::hardware_concurrency());
        const int count = std::min({o.threads, int(std::min(hardware, 3u)),
                                    int(std::max<size_t>(1, o.memory_limit_mb / 8))});
        result.threads_used = count;
        result.worker_memory_limit_mb = o.memory_limit_mb / count;
        SearchOptions worker_options = o;
        worker_options.memory_limit_mb = result.worker_memory_limit_mb;
        workers.resize(count);
        std::array<bool, 3> axes{};
        size_t frame_number = 0;
        for (size_t frame = 0; frame < 24; ++frame) {
          const auto& destination = cube_rotations()[frame].face_destination;
          const size_t up =
              std::find(destination.begin(), destination.end(), 'U') - destination.begin();
          if (!axes[up % 3]) {
            axes[up % 3] = true;
            // 两线程时：一个持续探索恒等坐标系，另一个分时探索其余两个主轴。
            workers[std::min(frame_number++, size_t(count - 1))].frames.push_back(frame);
          }
        }
        if (count == 1 && o.max_search_ms < 250) workers[0].frames.resize(1);
        auto run_worker = [&](int index) {
          auto& worker = workers[index];
          try {
            auto consider = [&](const std::vector<std::string>& moves, Clock::time_point until) {
              if (!running() || Clock::now() >= until) return false;
              std::string key;
              for (const auto& move : moves) {
                key += move;
                key += ' ';
              }
              {
                std::lock_guard lock(mutex);
                if (seen.contains(key)) return true;
              }
              ++worker.candidates;
              if (replay_facelet_moves(state, moves) != solved)
                throw std::runtime_error("Candidate failed independent facelet replay");
              auto p = optimize_robot_moves(moves, start, worker_options, until,
                                            feedback.upper_bound.load(), &feedback);
              {
                std::lock_guard lock(mutex);
                // 只去重已完成的评估。进行中或受时间/内存打断的候选允许其他线程评估，
                // 避免短分片抢占长分片的候选后超时，导致该候选永久漏评。
                if ((p && p->optimal_for_sequence) ||
                    (!p && Clock::now() < until && !feedback.cancel.load())) {
                  if (seen.size() >= 4096) seen.clear();
                  seen.insert(std::move(key));
                }
              }
              if (p) {
                ++worker.optimized;
                if (p->stop_reason == "memory_limit") {
                  worker.memory_limited = true;
                  memory_limited.store(true);
                }
                publish(std::move(*p));
              }
              return !worker.memory_limited && running() && Clock::now() < until;
            };
            for (size_t i = 0; i < worker.frames.size() && running(); ++i) {
              const auto now = Clock::now();
              const auto remaining = deadline - now;
              const auto until = worker.frames.size() == 1 || i + 1 == worker.frames.size()
                                     ? deadline
                                 : worker.frames.size() == 3 && i == 0 ? now + remaining * 3 / 5
                                                                       : now + remaining / 2;
              enumerate_solutions(
                  conjugate_facelets(state, worker.frames[i]), *tables, until,
                  [&](const auto& variant_moves) {
                    const auto moves = map_moves_to_original(variant_moves, worker.frames[i]);
                    if (!consider(moves, until)) return false;
                    for (size_t j = 1; j < moves.size(); ++j) {
                      auto [a, sa] = face_axis(moves[j - 1][0]);
                      auto [b, sb] = face_axis(moves[j][0]);
                      if (a == b && sa != sb) {
                        auto reordered = moves;
                        std::swap(reordered[j - 1], reordered[j]);
                        if (!consider(reordered, until)) return false;
                      }
                    }
                    return running() && Clock::now() < until;
                  },
                  &feedback.cancel);
              if (worker.memory_limited) break;
            }
          } catch (const std::runtime_error& e) {
            if (std::string(e.what()) == "Robot search memory limit reached") {
              worker.memory_limited = true;
              memory_limited.store(true);
            } else {
              std::lock_guard lock(mutex);
              if (!error) error = std::current_exception();
              feedback.cancel.store(true);
            }
          } catch (...) {
            std::lock_guard lock(mutex);
            if (!error) error = std::current_exception();
            feedback.cancel.store(true);
          }
        };
        std::vector<std::jthread> threads;
        try {
          for (int i = 1; i < count; ++i) threads.emplace_back(run_worker, i);
          run_worker(0);
        } catch (...) {
          // 创建线程失败也先取消并 join，避免引用离开作用域的协调状态。
          feedback.cancel.store(true);
          threads.clear();
          throw;
        }
        threads.clear();
        if (error) std::rethrow_exception(error);
        for (const auto& worker : workers) {
          result.candidates += worker.candidates;
          result.optimized_candidates += worker.optimized;
          result.workers.push_back({{"frames", worker.frames},
                                    {"candidates", worker.candidates},
                                    {"optimized_candidates", worker.optimized},
                                    {"memory_limited", worker.memory_limited}});
        }
      }
    }
  } catch (const std::runtime_error& e) {
    if (std::string(e.what()) != "Robot search memory limit reached") throw;
    memory_limited.store(true);
  }
  result.stop_reason = memory_limited.load()      ? "memory_limit"
                       : Clock::now() >= deadline ? "deadline"
                                                  : "candidate_search_finished";
  result.search_ms = std::chrono::duration<double, std::milli>(Clock::now() - began).count();
  return result;
}
Json RobotSearchResult::json(const SearchOptions& o) const {
  Json r = {{"found", found},
            {"objective", o.action_count ? "action_count" : "execution_time"},
            {"search_ms", search_ms},
            {"max_search_ms", o.max_search_ms},
            {"threads_requested", o.threads},
            {"threads_used", threads_used},
            {"worker_memory_limit_mb", worker_memory_limit_mb},
            {"preparation_ms", preparation_ms},
            {"workers", workers},
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
